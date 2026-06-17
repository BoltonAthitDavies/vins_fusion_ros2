/*******************************************************
 * carla_cpp: VinsPublisher
 *
 * The main project's Estimator does NOT self-publish (upstream's registerPub()
 * does not exist here). This polls Estimator::getVisualInertialOdom() on a
 * background thread and publishes the functional outputs downstream
 * (global_fusion / rtabmap) consume:
 *   /vins_estimator/odometry  (nav_msgs/Odometry)
 *   /vins_estimator/path      (nav_msgs/Path)
 * and appends a TUM trajectory line per fresh pose.
 *******************************************************/
#pragma once

#include <atomic>
#include <cstdio>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vins/estimator/estimator.h>

namespace carla_cpp {

class VinsPublisher {
 public:
  VinsPublisher(rclcpp::Node::SharedPtr node, Estimator &estimator,
                const std::string &traj_path,
                std::string world_frame = "world",
                std::string body_frame = "body");
  ~VinsPublisher();
  void stop();

 private:
  void run();

  rclcpp::Node::SharedPtr node_;
  Estimator &est_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  nav_msgs::msg::Path path_;
  std::string world_frame_, body_frame_;
  std::FILE *traj_ = nullptr;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace carla_cpp
