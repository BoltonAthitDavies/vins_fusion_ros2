// Offline VINS driver: reads a rosbag DIRECTLY with rosbag2_cpp SequentialReader
// (no ros2 bag play, no wall-clock tick) and feeds messages straight into the
// Estimator. Optional GPS mode drives global_fusion's GlobalOptimization in-process.
// Usage: vins_bag_reader <config.yaml> <bag_dir> <out.csv> [gps] [gnss_topic]
//   plain : out CSV = VIO trajectory      (t_sec,x,y,z)
//   gps   : out CSV = GPS-fused global path(t_sec,x,y,z)
#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <vins/estimator/estimator.h>
#include <vins/estimator/parameters.h>

#include "globalOpt.h"  // global_fusion GlobalOptimization (compiled in)

namespace {
TimeStampSec stampSec(const builtin_interfaces::msg::Time &t) {
  return static_cast<TimeStampSec>(t.sec + t.nanosec * 1e-9);
}
IMUData toImu(const sensor_msgs::msg::Imu &m) {
  IMUData d;
  d.timestamp = stampSec(m.header.stamp);
  d.angular_velocity = Eigen::Vector3d(m.angular_velocity.x, m.angular_velocity.y,
                                       m.angular_velocity.z);
  d.linear_acceleration = Eigen::Vector3d(
      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
  d.orientation = Eigen::Quaterniond(m.orientation.w, m.orientation.x,
                                     m.orientation.y, m.orientation.z);
  return d;
}
cv::Mat toMono(const sensor_msgs::msg::Image &img) {
  cv_bridge::CvImageConstPtr p;
  if (img.encoding == "8UC1") {
    sensor_msgs::msg::Image c = img;
    c.encoding = sensor_msgs::image_encodings::MONO8;
    p = cv_bridge::toCvCopy(c, sensor_msgs::image_encodings::MONO8);
  } else {
    p = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  return p->image.clone();
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    fprintf(stderr, "usage: %s <config> <bag> <out.csv> [gps] [gnss_topic]\n", argv[0]);
    return 1;
  }
  const std::string config = argv[1], bag = argv[2], out = argv[3];
  const bool gps_mode = (argc > 4 && std::string(argv[4]) == "gps");
  const std::string gnss_topic = (argc > 5) ? argv[5] : "/carla/ego_vehicle/gnss";

  cv::theRNG().state = 42;  // seed RANSAC RNG (cf. upstream EuRoCRawTest)
  auto options = std::make_shared<VINSOptions>();
  options->readParameters(config);
  Estimator estimator;
  estimator.initialize(options);

  const bool use_imu = options->hasImu();
  const std::string imu_topic = options->imuTopic();
  const std::string img0_topic = options->imageTopic();
  const std::string img1_topic = options->image1Topic();

  // logger: capture each fresh VIO pose (full pose for GPS, CSV for plain mode)
  std::ofstream fcsv;
  if (!gps_mode) fcsv.open(out);
  std::vector<OdomData> vio_poses;
  std::mutex vmtx;
  std::atomic<bool> running{true};
  std::atomic<double> last_logged{0.0};
  std::thread logger([&] {
    OdomData od;
    double prev = -1.0;
    while (running.load()) {
      if (estimator.getVisualInertialOdom(od) && od.timestamp != prev) {
        if (!gps_mode)
          fcsv << std::fixed << std::setprecision(9) << od.timestamp << ","
               << od.position.x() << "," << od.position.y() << ","
               << od.position.z() << "," << od.orientation.w() << ","
               << od.orientation.x() << "," << od.orientation.y() << ","
               << od.orientation.z() << "\n";
        { std::lock_guard<std::mutex> l(vmtx); vio_poses.push_back(od); }
        prev = od.timestamp;
        last_logged.store(od.timestamp);
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });

  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions so;
  so.uri = bag; so.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions co;
  co.input_serialization_format = "cdr"; co.output_serialization_format = "cdr";
  reader.open(so, co);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gps_ser;

  const double SYNC_TOL = 0.06;
  std::map<double, cv::Mat> b0, b1;
  std::vector<std::array<double, 5>> gps_buf;  // t,lat,lon,alt,acc
  double last_img = 0.0;
  auto pair_up = [&] {
    while (!b0.empty() && !b1.empty()) {
      double t0 = b0.begin()->first;
      if (b1.rbegin()->first < t0) break;
      auto hi = b1.lower_bound(t0);
      auto best = hi; double bestdt = 1e18;
      if (hi != b1.end()) bestdt = std::abs(hi->first - t0);
      if (hi != b1.begin()) { auto lo = std::prev(hi);
        if (std::abs(lo->first - t0) < bestdt) { best = lo; bestdt = std::abs(lo->first - t0); } }
      if (bestdt <= SYNC_TOL) {
        ImageData img; img.timestamp = t0; img.image0 = b0.begin()->second; img.image1 = best->second;
        estimator.inputImage(img); last_img = t0; b1.erase(best);
      }
      b0.erase(b0.begin());
    }
  };

  while (reader.has_next()) {
    auto bm = reader.read_next();
    rclcpp::SerializedMessage sm(*bm->serialized_data);
    const std::string &tp = bm->topic_name;
    if (use_imu && tp == imu_topic) {
      sensor_msgs::msg::Imu m; imu_ser.deserialize_message(&sm, &m);
      estimator.inputIMU(toImu(m));
    } else if (tp == img0_topic) {
      sensor_msgs::msg::Image m; img_ser.deserialize_message(&sm, &m);
      b0[stampSec(m.header.stamp)] = toMono(m); pair_up();
    } else if (tp == img1_topic) {
      sensor_msgs::msg::Image m; img_ser.deserialize_message(&sm, &m);
      b1[stampSec(m.header.stamp)] = toMono(m); pair_up();
    } else if (gps_mode && tp == gnss_topic) {
      sensor_msgs::msg::NavSatFix m; gps_ser.deserialize_message(&sm, &m);
      double acc = m.position_covariance[0]; if (acc <= 0) acc = 1.0;
      gps_buf.push_back({stampSec(m.header.stamp), m.latitude, m.longitude, m.altitude, acc});
    }
  }

  auto t0 = std::chrono::steady_clock::now();
  while (last_logged.load() < last_img - 1e-3) {
    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(60)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  running.store(false);
  logger.join();

  if (gps_mode) {
    GlobalOptimization globalOpt;  // its 2s thread re-solves the full graph deterministically
    std::vector<double> ots;
    for (const auto &od : vio_poses) {
      globalOpt.inputOdom(od.timestamp, od.position, od.orientation);
      ots.push_back(od.timestamp);
    }
    // match each GPS to the nearest VIO pose timestamp (so the optimizer links them by key)
    size_t matched = 0;
    for (const auto &g : gps_buf) {
      auto it = std::lower_bound(ots.begin(), ots.end(), g[0]);
      double best = -1, bd = 1e18;
      if (it != ots.end() && std::abs(*it - g[0]) < bd) { bd = std::abs(*it - g[0]); best = *it; }
      if (it != ots.begin()) { double v = *std::prev(it);
        if (std::abs(v - g[0]) < bd) { bd = std::abs(v - g[0]); best = v; } }
      if (best > 0 && bd <= SYNC_TOL) { globalOpt.inputGPS(best, g[1], g[2], g[3], g[4]); ++matched; }
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));  // let the batch optimize converge
    std::ofstream fo(out);
    for (const auto &ps : globalOpt.global_path.poses) {
      double t = ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9;
      fo << std::fixed << std::setprecision(9) << t << "," << ps.pose.position.x
         << "," << ps.pose.position.y << "," << ps.pose.position.z << ","
         << ps.pose.orientation.w << "," << ps.pose.orientation.x << ","
         << ps.pose.orientation.y << "," << ps.pose.orientation.z << "\n";
    }
    fprintf(stderr, "[vins_bag_reader] GPS: vio_poses=%zu gps=%zu matched=%zu path=%zu\n",
            vio_poses.size(), gps_buf.size(), matched, globalOpt.global_path.poses.size());
  } else {
    fprintf(stderr, "[vins_bag_reader] VIO: poses=%zu last_img=%.3f\n", vio_poses.size(), last_img);
  }
  return 0;
}
