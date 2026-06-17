/*******************************************************
 * carla_cpp: CARLA -> VINS/ROS unit conversions.
 *
 * These exactly mirror the Python carla_ros_bridge so the C++ env feeds
 * VINS the same data the proven pipeline used:
 *   - IMU  : carla_ros_bridge/src/carla_ros_bridge/imu.py:70-79
 *   - Image: carla_ros_bridge/src/carla_ros_bridge/camera.py:217-234 ('bgra8')
 *
 * Header-only and OpenCV/Eigen only (no CARLA, no ROS) so it can be used
 * from the LibCarla callbacks and from VinsBridge alike.
 *******************************************************/
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

namespace carla_cpp {

// CARLA uses a left-handed frame (X forward, Y right, Z up); ROS/VINS uses
// right-handed (X forward, Y left, Z up). Replicates imu.py exactly:
//   accel: ( x, -y,  z)      gyro: (-x,  y, -z)
inline Eigen::Vector3d carlaAccelToVins(float x, float y, float z) {
  return Eigen::Vector3d(x, -y, z);
}
inline Eigen::Vector3d carlaGyroToVins(float x, float y, float z) {
  return Eigen::Vector3d(-x, y, -z);
}

// CARLA RGB cameras deliver 32-bit BGRA (carla::sensor::data::Color is
// {b,g,r,a}); VINS' feature tracker wants single-channel 8-bit. The input
// `bgra` typically aliases the CARLA buffer, so the output is a fresh
// allocation that stays valid after the sensor callback returns.
inline cv::Mat bgraToGray(const cv::Mat &bgra) {
  cv::Mat gray;
  cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
  return gray;
}

// Pinhole intrinsics matching camera.py:_build_camera_info():
//   fx = width / (2 * tan(fov_deg * pi / 360)), fy = fx,
//   cx = width/2, cy = height/2.
struct Intrinsics { double fx, fy, cx, cy; };
inline Intrinsics intrinsicsFromFov(int width, int height, double fov_deg) {
  const double fx = width / (2.0 * std::tan(fov_deg * M_PI / 360.0));
  return Intrinsics{fx, fx, width / 2.0, height / 2.0};
}

}  // namespace carla_cpp
