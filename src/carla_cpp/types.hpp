/*******************************************************
 * carla_cpp: native C++ LibCarla -> VINS-Fusion driver
 *
 * Shared plain-data types that form the boundary between the three
 * worlds in this program and must therefore depend on NEITHER CARLA
 * headers NOR rclcpp NOR the VINS estimator headers:
 *   - CarlaWorld   (LibCarla + OpenCV)  produces these
 *   - VinsBridge   (estimator + OpenCV) consumes images/imu
 *   - RosOutput    (rclcpp)             consumes images/gnss/clock
 *
 * Keeping these structs dependency-free is what lets each translation
 * unit include only one heavy toolchain, avoiding header clashes.
 *******************************************************/
#pragma once

#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

namespace carla_cpp {

// One IMU sample, already converted from CARLA (left-handed) to the
// ROS/VINS right-handed convention. Timestamp is CARLA simulation time
// (elapsed_seconds), the same clock used for images.
struct ImuSample {
  double t = 0.0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();  // linear acceleration
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();  // angular velocity
};

// Everything CARLA produced for a single synchronous tick (frame).
// Every tick carries exactly one IMU sample (sensor_tick == fixed_delta);
// every Nth tick additionally carries a stereo pair + a GNSS fix.
struct FrameBundle {
  uint64_t frame = 0;   // CARLA frame id (SensorData::GetFrame())
  double t = 0.0;       // sim time of this tick (from the IMU sample)

  std::vector<ImuSample> imu;

  bool hasStereo = false;
  cv::Mat left;         // CV_8UC4 BGRA (as delivered by CARLA RGB camera)
  cv::Mat right;        // CV_8UC4 BGRA
  double tImg = 0.0;

  bool hasGnss = false;
  double lat = 0.0, lon = 0.0, alt = 0.0;
  double tGnss = 0.0;
};

// Manual driving command produced by KeyboardControl. Kept free of the
// carla::rpc::VehicleControl type so keyboard_control.* needs no CARLA
// headers; main maps it onto cc::Vehicle::Control.
struct ControlCmd {
  float throttle = 0.0f;
  float steer = 0.0f;
  float brake = 0.0f;
  bool reverse = false;
  bool hand_brake = false;
};

// Ego ground-truth odometry in ROS/ENU (CARLA y/yaw negated), published on
// /carla/ego_vehicle/odometry to mirror the Python carla_ros_bridge pseudo-odom.
// Orientation is yaw-only (qx=qy=0), matching the GT convention used elsewhere.
struct EgoOdom {
  double t = 0.0;
  double x = 0.0, y = 0.0, z = 0.0;       // position
  double qz = 0.0, qw = 1.0;              // yaw quaternion
  double vx = 0.0, vy = 0.0, vz = 0.0;    // linear velocity
  double wz = 0.0;                        // yaw rate [rad/s]
};

}  // namespace carla_cpp
