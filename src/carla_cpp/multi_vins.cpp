#include "multi_vins.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

namespace carla_cpp {

static nav_msgs::msg::Odometry toOdom(double t, const Eigen::Vector3d &p,
                                      const Eigen::Quaterniond &q,
                                      const std::string &frame) {
  nav_msgs::msg::Odometry o;
  o.header.stamp.sec = static_cast<int32_t>(t);
  o.header.stamp.nanosec = static_cast<uint32_t>((t - o.header.stamp.sec) * 1e9);
  o.header.frame_id = frame;
  o.child_frame_id = "body";
  o.pose.pose.position.x = p.x();
  o.pose.pose.position.y = p.y();
  o.pose.pose.position.z = p.z();
  o.pose.pose.orientation.x = q.x();
  o.pose.pose.orientation.y = q.y();
  o.pose.pose.orientation.z = q.z();
  o.pose.pose.orientation.w = q.w();
  return o;
}

MultiVins::MultiVins(rclcpp::Node::SharedPtr node, const std::string &cfg_dir,
                     const std::set<std::string> &enabled)
    : node_(node) {
  auto want = [&](const std::string &k) { return enabled.empty() || enabled.count(k); };
  // name, config, use_imu, mono, gps, vio_topic, gps_topic
  if (want("mono"))
    addVariant("mono",       cfg_dir + "/carla_native_mono.yaml",   true,  true,  false,
               "/vins_mono/odometry",       "");
  if (want("stereo"))
    addVariant("stereo",     cfg_dir + "/carla_native_stereo.yaml", false, false, true,
               "/vins_stereo/odometry",     "/vins_stereo_gps/odometry");
  if (want("stereo_imu"))
    addVariant("stereo_imu", cfg_dir + "/carla_native.yaml",        true,  false, true,
               "/vins_stereo_imu/odometry", "/vins_stereo_imu_gps/odometry");
}

MultiVins::~MultiVins() { stop(); }

void MultiVins::addVariant(const std::string &name, const std::string &cfg,
                           bool use_imu, bool mono, bool gps,
                           const std::string &topic,
                           const std::string &gps_topic) {
  auto v = std::make_unique<Variant>();
  v->name = name;
  v->opts = std::make_shared<VINSOptions>();
  v->opts->readParameters(cfg);
  v->est = std::make_shared<Estimator>();
  v->est->initialize(v->opts);
  v->bridge = std::make_unique<VinsBridge>(*v->est, use_imu, mono);
  v->pub = node_->create_publisher<nav_msgs::msg::Odometry>(
      topic, rclcpp::QoS(rclcpp::KeepLast(1000)));
  v->gps = gps;
  if (gps) {
    v->gopt = std::make_shared<GlobalOptimization>();
    v->pub_gps = node_->create_publisher<nav_msgs::msg::Odometry>(
        gps_topic, rclcpp::QoS(rclcpp::KeepLast(1000)));
  }
  Variant *raw = v.get();
  variants_.push_back(std::move(v));
  raw->out = std::thread(&MultiVins::outLoop, this, raw);
  printf("[multi] %-11s -> %s%s%s\n", name.c_str(), topic.c_str(),
         gps ? "  +  " : "", gps ? gps_topic.c_str() : "");
}

void MultiVins::pushImu(const ImuSample &s) {
  for (auto &v : variants_) v->bridge->pushImu(s);  // bridge drops it if !use_imu
}

void MultiVins::pushStereo(const cv::Mat &left, const cv::Mat &right, double t) {
  for (auto &v : variants_) v->bridge->pushStereo(left, right, t);  // bridge handles mono
}

void MultiVins::pushGnss(double t, double lat, double lon, double alt) {
  std::lock_guard<std::mutex> lk(gnss_m_);
  g_t_ = t; g_lat_ = lat; g_lon_ = lon; g_alt_ = alt;
}

void MultiVins::outLoop(Variant *v) {
  double prev = -1.0;
  OdomData od;
  while (running_.load()) {
    if (v->est->getVisualInertialOdom(od) && od.timestamp != prev) {
      prev = od.timestamp;
      v->pub->publish(toOdom(od.timestamp, od.position, od.orientation, "world"));

      if (v->gps) {
        v->gopt->inputOdom(od.timestamp, od.position, od.orientation);
        // Feed the latest GNSS if it lines up with this VIO pose; key it by the
        // odom timestamp so global_fusion links the GPS and odom constraints.
        bool have = false; double lat = 0, lon = 0, alt = 0;
        {
          std::lock_guard<std::mutex> lk(gnss_m_);
          if (g_t_ > 0 && std::fabs(g_t_ - od.timestamp) < 0.06) {
            have = true; lat = g_lat_; lon = g_lon_; alt = g_alt_;
          }
        }
        if (have) v->gopt->inputGPS(od.timestamp, lat, lon, alt, 1.0);
        Eigen::Vector3d gp; Eigen::Quaterniond gq;
        v->gopt->getGlobalOdom(gp, gq);
        v->pub_gps->publish(toOdom(od.timestamp, gp, gq, "world"));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void MultiVins::stop() {
  if (!running_.exchange(false)) return;
  for (auto &v : variants_) if (v->bridge) v->bridge->stop();  // stop feeding
  for (auto &v : variants_) if (v->out.joinable()) v->out.join();  // then output threads
}

}  // namespace carla_cpp
