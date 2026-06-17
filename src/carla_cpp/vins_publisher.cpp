#include "vins_publisher.hpp"

#include <chrono>

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace carla_cpp {

static builtin_interfaces::msg::Time toStamp(double t) {
  builtin_interfaces::msg::Time s;
  s.sec = static_cast<int32_t>(t);
  s.nanosec = static_cast<uint32_t>((t - s.sec) * 1e9);
  return s;
}

VinsPublisher::VinsPublisher(rclcpp::Node::SharedPtr node, Estimator &estimator,
                             const std::string &traj_path,
                             std::string world_frame, std::string body_frame)
    : node_(node),
      est_(estimator),
      world_frame_(std::move(world_frame)),
      body_frame_(std::move(body_frame)) {
  pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/vins_estimator/odometry", rclcpp::QoS(rclcpp::KeepLast(1000)));
  pub_path_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/vins_estimator/path", rclcpp::QoS(rclcpp::KeepLast(10)));
  path_.header.frame_id = world_frame_;
  if (!traj_path.empty()) {
    traj_ = std::fopen(traj_path.c_str(), "w");
    if (traj_ == nullptr)
      printf("[carla_cpp] WARN: cannot open trajectory file: %s\n",
             traj_path.c_str());
    else
      printf("[carla_cpp] writing TUM trajectory to %s\n", traj_path.c_str());
  }
  thread_ = std::thread(&VinsPublisher::run, this);
}

VinsPublisher::~VinsPublisher() {
  stop();
  if (traj_ != nullptr) std::fclose(traj_);
}

void VinsPublisher::stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void VinsPublisher::run() {
  double prev = -1.0;
  OdomData od;
  while (running_.load()) {
    // getVisualInertialOdom() returns false until the estimator has
    // initialized (NON_LINEAR), and true once per fresh VIO pose.
    if (est_.getVisualInertialOdom(od) && od.timestamp != prev) {
      prev = od.timestamp;
      auto stamp = toStamp(od.timestamp);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = world_frame_;
      odom.child_frame_id = body_frame_;
      odom.pose.pose.position.x = od.position.x();
      odom.pose.pose.position.y = od.position.y();
      odom.pose.pose.position.z = od.position.z();
      odom.pose.pose.orientation.x = od.orientation.x();
      odom.pose.pose.orientation.y = od.orientation.y();
      odom.pose.pose.orientation.z = od.orientation.z();
      odom.pose.pose.orientation.w = od.orientation.w();
      odom.twist.twist.linear.x = od.velocity.x();
      odom.twist.twist.linear.y = od.velocity.y();
      odom.twist.twist.linear.z = od.velocity.z();
      pub_odom_->publish(odom);

      geometry_msgs::msg::PoseStamped ps;
      ps.header = odom.header;
      ps.pose = odom.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(ps);
      pub_path_->publish(path_);

      if (traj_ != nullptr) {
        // TUM: timestamp tx ty tz qx qy qz qw
        std::fprintf(traj_, "%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
                     od.timestamp, od.position.x(), od.position.y(),
                     od.position.z(), od.orientation.x(), od.orientation.y(),
                     od.orientation.z(), od.orientation.w());
        std::fflush(traj_);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

}  // namespace carla_cpp
