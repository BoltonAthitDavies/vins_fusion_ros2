/*******************************************************
 * carla_cpp: RosOutput
 *
 * The low-rate (<=10 Hz) ROS output edge. The CARLA->VINS ingestion path is
 * ROS-free; this republishes exactly the subset of carla_ros_bridge topics
 * that terminal-3 rtabmap + global_fusion consume, so the existing mapping
 * pipeline keeps working unchanged:
 *   /clock                                          (sim time; consumers use_sim_time)
 *   /carla/ego_vehicle/cam_front_{left,right}/image + /camera_info  (bgra8)
 *   /carla/ego_vehicle/gnss                         (NavSatFix)
 *   static TF ego_vehicle -> ego_vehicle/cam_front_{left,right}
 *
 * VINS odometry itself is published by the estimator's own registerPub()
 * path, not here.
 *******************************************************/
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "types.hpp"

namespace carla_cpp {

class RosOutput {
 public:
  explicit RosOutput(rclcpp::Node::SharedPtr node);

  void publishClock(double sim_time);
  void publish(const FrameBundle &b);  // stereo + camera_info + gnss
  void publishOdometry(const EgoOdom &o);    // /carla/ego_vehicle/odometry (GT)
  void publishWheelOdom(const EgoOdom &o);   // /carla/ego_vehicle/wheel_odometry
  void publishNoiseOdom(const EgoOdom &o);   // /carla/ego_vehicle/noise_odometry

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_, pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_,
      pub_right_info_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_noise_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

  sensor_msgs::msg::CameraInfo left_info_, right_info_;

  static sensor_msgs::msg::CameraInfo makeCameraInfo(const std::string &frame,
                                                     double baseline = 0.0);
};

}  // namespace carla_cpp
