/*******************************************************
 * CARLA rosbag2 test — reads a bag directly via rosbag2_cpp, feeds the
 * refactored VINS Estimator. No bag playback process needed.
 *
 * Ported from the original-API CarlaBagTest.cpp to the vins_fusion_ros2
 * refactored Estimator API:
 *   readParameters()/setParameter()/registerPub()  ->  VINSOptions + initialize()
 *   inputIMU(t,acc,gyr) / inputImage(t,l,r)         ->  IMUData / ImageData structs
 *   OUTPUT_FOLDER global                            ->  options->OUTPUT_FOLDER
 * Since there is no registerPub(), this publishes /vins_estimator/odometry
 * itself (world-frame body pose) so carla_plot.py still sees the VINS track.
 *
 * Expected topics:
 *   /carla/ego_vehicle/cam_front_left/image   (sensor_msgs/Image)
 *   /carla/ego_vehicle/cam_front_right/image  (sensor_msgs/Image)
 *   /carla/ego_vehicle/imu                    (sensor_msgs/Imu)
 *   /carla/ego_vehicle/gnss                   (sensor_msgs/NavSatFix)
 * IMU is read regardless of config; set imu:0 in config to run stereo-only.
 *
 * Usage:
 *   ros2 run vins_fusion_ros2 carla_bag_test <config_file> <bag_path>
 *******************************************************/

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <vins/estimator/estimator.h>
#include <vins/estimator/parameters.h>

using namespace std;
using namespace Eigen;

Estimator estimator;

static const string TOP_IMU   = "/carla/ego_vehicle/imu";
static const string TOP_LEFT  = "/carla/ego_vehicle/cam_front_left/image";
static const string TOP_RIGHT = "/carla/ego_vehicle/cam_front_right/image";
static const string TOP_GNSS  = "/carla/ego_vehicle/gnss";

struct ImuRec {
    double t;
    double ax, ay, az;
    double wx, wy, wz;
};

struct GnssRec {
    double t;
    sensor_msgs::msg::NavSatFix msg;
};

// — Deserialise a bag message into a ROS2 message type —————————————————————

template<typename T>
static T deserialise(const shared_ptr<rcutils_uint8_array_t>& raw)
{
    rclcpp::SerializedMessage ser(raw->buffer_length);
    auto& rcl = ser.get_rcl_serialized_message();
    memcpy(rcl.buffer, raw->buffer, raw->buffer_length);
    rcl.buffer_length = raw->buffer_length;

    T msg;
    rclcpp::Serialization<T>{}.deserialize_message(&ser, &msg);
    return msg;
}

// — Main ——————————————————————————————————————————————————————————————————

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    cv::theRNG().state = 42;
    auto n = rclcpp::Node::make_shared("vins_estimator");

    auto pubLeft      = n->create_publisher<sensor_msgs::msg::Image>("/leftImage",  1000);
    auto pubRight     = n->create_publisher<sensor_msgs::msg::Image>("/rightImage", 1000);
    auto pubInfoLeft  = n->create_publisher<sensor_msgs::msg::CameraInfo>("/leftCameraInfo",  10);
    auto pubInfoRight = n->create_publisher<sensor_msgs::msg::CameraInfo>("/rightCameraInfo", 10);
    auto pubGnss      = n->create_publisher<sensor_msgs::msg::NavSatFix>("/carla/ego_vehicle/gnss", 100);
    // No registerPub() in the refactored API — publish VINS odometry ourselves.
    auto pubOdom      = n->create_publisher<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 1000);

    // — Build camera_info from the CARLA stereo calibration ——————————————————
    // fx=fy=480, cx=480, cy=360, width=960, height=720, baseline=0.5m
    sensor_msgs::msg::CameraInfo ci_left, ci_right;
    for (auto* ci : {&ci_left, &ci_right}) {
        ci->width  = 960;
        ci->height = 720;
        ci->distortion_model = "plumb_bob";
        ci->d = {0.0, 0.0, 0.0, 0.0, 0.0};
        ci->k = {480.0,   0.0, 480.0,
                   0.0, 480.0, 360.0,
                   0.0,   0.0,   1.0};
        ci->r = {1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0};
    }
    ci_left.header.frame_id  = "camera";
    ci_left.p  = {480.0, 0.0, 480.0,    0.0,
                    0.0, 480.0, 360.0,  0.0,
                    0.0,   0.0,   1.0,  0.0};
    // Right camera: Tx = -fx * baseline = -480 * 0.5 = -240
    ci_right.header.frame_id = "right_camera";
    ci_right.p = {480.0, 0.0, 480.0, -240.0,
                    0.0, 480.0, 360.0,   0.0,
                    0.0,   0.0,   1.0,   0.0};

    auto clean = rclcpp::remove_ros_arguments(argc, argv);
    if (clean.size() != 3) {
        printf("usage: carla_bag_test <config_file> <bag_path>\n"
               "  bag_path — rosbag2 directory (contains *.db3 + metadata.yaml)\n");
        return 1;
    }

    string configFile = clean[1];
    string bagPath    = clean[2];
    while (bagPath.size() > 1 && bagPath.back() == '/') bagPath.pop_back();

    printf("config  : %s\n", configFile.c_str());
    printf("bag     : %s\n", bagPath.c_str());

    auto options = std::make_shared<VINSOptions>();
    options->readParameters(configFile);
    estimator.initialize(options);

    // — Open bag —————————————————————————————————————————————————————————
    rosbag2_cpp::Reader reader;
    {
        rosbag2_storage::StorageOptions opts;
        opts.uri        = bagPath;
        opts.storage_id = "";   // auto-detect (sqlite3)
        reader.open(opts);
    }

    rosbag2_storage::StorageFilter flt;
    flt.topics = {TOP_IMU, TOP_LEFT, TOP_RIGHT, TOP_GNSS};
    reader.set_filter(flt);

    // — Load all messages ————————————————————————————————————————————————
    vector<ImuRec>       imuData;
    vector<GnssRec>      gnssData;
    map<double, cv::Mat> leftMap;    // t → grayscale
    map<double, cv::Mat> rightMap;

    printf("reading bag ... (may take a moment for large bags)\n");
    size_t n_imu = 0, n_left = 0, n_right = 0, n_gnss = 0;

    while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        const string& topic = bag_msg->topic_name;

        if (topic == TOP_IMU) {
            auto imu = deserialise<sensor_msgs::msg::Imu>(bag_msg->serialized_data);
            double t = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9;
            imuData.push_back({t,
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z});
            n_imu++;

        } else if (topic == TOP_GNSS) {
            auto gnss = deserialise<sensor_msgs::msg::NavSatFix>(bag_msg->serialized_data);
            double t = gnss.header.stamp.sec + gnss.header.stamp.nanosec * 1e-9;
            gnssData.push_back({t, gnss});
            n_gnss++;

        } else if (topic == TOP_LEFT || topic == TOP_RIGHT) {
            auto img_msg = deserialise<sensor_msgs::msg::Image>(bag_msg->serialized_data);
            double t = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;

            cv::Mat gray;
            try {
                auto cv_ptr = cv_bridge::toCvCopy(img_msg, "mono8");
                gray = cv_ptr->image.clone();
            } catch (const cv_bridge::Exception& e) {
                printf("warning: cv_bridge error at t=%.3f: %s\n", t, e.what());
                continue;
            }

            if (topic == TOP_LEFT) { leftMap[t]  = gray; n_left++;  }
            else                   { rightMap[t] = gray; n_right++; }
        }
    }

    printf("loaded  : imu=%zu  left=%zu  right=%zu  gnss=%zu\n",
           n_imu, n_left, n_right, n_gnss);

    if (leftMap.empty()) {
        printf("error: no left camera frames found — check topic names in bag\n");
        return 1;
    }

    sort(imuData.begin(), imuData.end(),
         [](const ImuRec& a, const ImuRec& b){ return a.t < b.t; });

    // — Output file ——————————————————————————————————————————————————————
    FILE* outFile = fopen((options->OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
    if (!outFile)
        printf("warning: output path missing: %s -- poses won't be saved\n",
               options->OUTPUT_FOLDER.c_str());

    size_t imuIdx  = 0;
    size_t gnssIdx = 0;
    size_t frameIdx = 0;

    for (auto& [tImg, imLeft] : leftMap) {
        if (!rclcpp::ok()) break;

        // Match right frame by exact timestamp; fall back to nearest within 50 ms
        cv::Mat imRight;
        auto itR = rightMap.find(tImg);
        if (itR != rightMap.end()) {
            imRight = itR->second;
        } else {
            auto itLow = rightMap.lower_bound(tImg);
            cv::Mat* best = nullptr;
            double best_dt = 0.05;

            if (itLow != rightMap.end()) {
                double dt = itLow->first - tImg;
                if (dt < best_dt) { best_dt = dt; best = &itLow->second; }
            }
            if (itLow != rightMap.begin()) {
                --itLow;
                double dt = tImg - itLow->first;
                if (dt < best_dt) { best = &itLow->second; }
            }
            if (!best) {
                printf("warning: no right frame for t=%.6f, skipping\n", tImg);
                continue;
            }
            imRight = *best;
        }

        // Republish GNSS messages up to this image timestamp (async for RTAB-Map)
        while (gnssIdx < gnssData.size() && gnssData[gnssIdx].t <= tImg) {
            pubGnss->publish(gnssData[gnssIdx].msg);
            gnssIdx++;
        }

        // Feed IMU records up to this image timestamp.
        // If imu:0 in config, estimator ignores these calls — no overhead.
        while (imuIdx < imuData.size() && imuData[imuIdx].t <= tImg) {
            const auto& r = imuData[imuIdx];
            IMUData imu;
            imu.timestamp = r.t;
            imu.linear_acceleration = Vector3d(r.ax, r.ay, r.az);
            imu.angular_velocity    = Vector3d(r.wx, r.wy, r.wz);
            estimator.inputIMU(imu);
            imuIdx++;
        }

        // Publish images + camera_info (for RViz and RTAB-Map)
        auto stamp = rclcpp::Time(static_cast<uint64_t>(tImg * 1e9));
        auto lMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imLeft).toImageMsg();
        auto rMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imRight).toImageMsg();
        lMsg->header.frame_id = "camera";
        rMsg->header.frame_id = "right_camera";
        lMsg->header.stamp = rMsg->header.stamp = stamp;
        ci_left.header.stamp = ci_right.header.stamp = stamp;
        pubLeft->publish(*lMsg);
        pubRight->publish(*rMsg);
        pubInfoLeft->publish(ci_left);
        pubInfoRight->publish(ci_right);

        ImageData img;
        img.timestamp = tImg;
        img.image0 = imLeft;
        img.image1 = imRight;
        estimator.inputImage(img);

        Matrix<double, 4, 4> pose;
        estimator.getPoseInWorldFrame(pose);
        if (outFile)
            fprintf(outFile,
                    "%f %f %f %f %f %f %f %f %f %f %f %f\n",
                    pose(0,0), pose(0,1), pose(0,2), pose(0,3),
                    pose(1,0), pose(1,1), pose(1,2), pose(1,3),
                    pose(2,0), pose(2,1), pose(2,2), pose(2,3));

        // Publish VINS odometry (world-frame body pose) for carla_plot.py.
        {
            Matrix3d R = pose.block<3,3>(0,0);
            Quaterniond q(R);
            nav_msgs::msg::Odometry od;
            od.header.stamp = stamp;
            od.header.frame_id = "world";
            od.child_frame_id  = "body";
            od.pose.pose.position.x = pose(0,3);
            od.pose.pose.position.y = pose(1,3);
            od.pose.pose.position.z = pose(2,3);
            od.pose.pose.orientation.x = q.x();
            od.pose.pose.orientation.y = q.y();
            od.pose.pose.orientation.z = q.z();
            od.pose.pose.orientation.w = q.w();
            pubOdom->publish(od);
        }

        if (frameIdx % 50 == 0)
            printf("frame %zu/%zu  imu_fed=%zu\n", frameIdx, leftMap.size(), imuIdx);
        frameIdx++;
    }

    if (outFile) fclose(outFile);
    printf("done.\n");
    rclcpp::shutdown();   // let VINS background threads finish before node is destroyed
    return 0;
}
