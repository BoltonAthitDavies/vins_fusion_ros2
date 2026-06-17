#include "ros_output.hpp"

#include <array>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace carla_cpp {

namespace {
constexpr char kLeftFrame[] = "ego_vehicle/cam_front_left";
constexpr char kRightFrame[] = "ego_vehicle/cam_front_right";
constexpr char kGnssFrame[] = "ego_vehicle/gnss";
constexpr char kBaseFrame[] = "ego_vehicle";

// Sim time (CARLA elapsed_seconds) -> ROS stamp, matching the way the
// estimator stamps its odometry so all topics share one timeline.
builtin_interfaces::msg::Time toStamp(double t) {
  builtin_interfaces::msg::Time s;
  s.sec = static_cast<int32_t>(t);
  s.nanosec = static_cast<uint32_t>((t - s.sec) * 1e9);
  return s;
}

// Optical-frame transform: parent ego_vehicle -> camera optical frame.
// Rotation maps CARLA camera axes to the ROS optical frame
// (x-right, y-down, z-forward), i.e. mat2quat([[0,0,1],[-1,0,0],[0,-1,0]])
// = (w,x,y,z) = (0.5,-0.5,0.5,-0.5), matching carla_ros_bridge camera.py.
// Translation is the spawn offset converted CARLA->ROS (y negated).
geometry_msgs::msg::TransformStamped camTf(const std::string &child,
                                           double y_ros) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = kBaseFrame;
  tf.child_frame_id = child;
  tf.transform.translation.x = 1.5;
  tf.transform.translation.y = y_ros;
  tf.transform.translation.z = 1.5;
  tf.transform.rotation.w = 0.5;
  tf.transform.rotation.x = -0.5;
  tf.transform.rotation.y = 0.5;
  tf.transform.rotation.z = -0.5;
  return tf;
}
}  // namespace

sensor_msgs::msg::CameraInfo RosOutput::makeCameraInfo(
    const std::string &frame, double baseline) {
  sensor_msgs::msg::CameraInfo ci;
  ci.header.frame_id = frame;
  ci.width = 960;
  ci.height = 720;
  ci.distortion_model = "plumb_bob";
  ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  // fov 90 deg, 960x720 -> fx=fy=480, cx=480, cy=360 (see camera.py).
  const double fx = 480.0, fy = 480.0, cx = 480.0, cy = 360.0;
  ci.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  // P[3] = -fx * baseline encodes the stereo baseline (0 for the left/reference
  // camera, -fx*B for the right). rtabmap reads the baseline from here -- it
  // will NOT derive it from TF -- so without this stereo produces no depth.
  ci.p = {fx, 0.0, cx, -fx * baseline, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return ci;
}

RosOutput::RosOutput(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // reliable, matches bridge
  pub_clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
  pub_left_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/carla/ego_vehicle/cam_front_left/image", qos);
  pub_right_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/carla/ego_vehicle/cam_front_right/image", qos);
  pub_left_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/carla/ego_vehicle/cam_front_left/camera_info", qos);
  pub_right_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/carla/ego_vehicle/cam_front_right/camera_info", qos);
  pub_gnss_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/carla/ego_vehicle/gnss", qos);
  pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/odometry", qos);
  pub_wheel_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/wheel_odometry", qos);
  pub_noise_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/noise_odometry", qos);

  left_info_ = makeCameraInfo(kLeftFrame);            // reference: baseline 0
  right_info_ = makeCameraInfo(kRightFrame, 0.5);     // 0.5 m stereo baseline -> P[3]=-240

  // Static cam extrinsics (latched). rtabmap derives the stereo baseline
  // (0.5 m) from the two optical frames in TF.
  static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  auto stamp = node_->now();
  auto l = camTf(kLeftFrame, -0.25);
  auto r = camTf(kRightFrame, 0.25);
  l.header.stamp = stamp;
  r.header.stamp = stamp;
  static_tf_->sendTransform({l, r});
}

void RosOutput::publishClock(double sim_time) {
  rosgraph_msgs::msg::Clock msg;
  msg.clock = toStamp(sim_time);
  pub_clock_->publish(msg);
}

void RosOutput::publishOdometry(const EgoOdom &o) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = toStamp(o.t);
  msg.header.frame_id = "map";
  msg.child_frame_id = "ego_vehicle";
  msg.pose.pose.position.x = o.x;
  msg.pose.pose.position.y = o.y;
  msg.pose.pose.position.z = o.z;
  msg.pose.pose.orientation.z = o.qz;
  msg.pose.pose.orientation.w = o.qw;
  msg.twist.twist.linear.x = o.vx;
  msg.twist.twist.linear.y = o.vy;
  msg.twist.twist.linear.z = o.vz;
  msg.twist.twist.angular.z = o.wz;
  pub_odom_->publish(msg);
}

void RosOutput::publishWheelOdom(const EgoOdom &o) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = toStamp(o.t);
  msg.header.frame_id = "odom";   // REP-105: wheel odometry is a relative frame
  msg.child_frame_id = "ego_vehicle";
  msg.pose.pose.position.x = o.x;
  msg.pose.pose.position.y = o.y;
  msg.pose.pose.position.z = o.z;
  msg.pose.pose.orientation.z = o.qz;
  msg.pose.pose.orientation.w = o.qw;
  msg.twist.twist.linear.x = o.vx;
  msg.twist.twist.linear.y = o.vy;
  msg.twist.twist.angular.z = o.wz;
  pub_wheel_->publish(msg);
}

void RosOutput::publishNoiseOdom(const EgoOdom &o) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = toStamp(o.t);
  msg.header.frame_id = "map";   // GT + noise -> same world frame as /odometry
  msg.child_frame_id = "ego_vehicle";
  msg.pose.pose.position.x = o.x;
  msg.pose.pose.position.y = o.y;
  msg.pose.pose.position.z = o.z;
  msg.pose.pose.orientation.z = o.qz;
  msg.pose.pose.orientation.w = o.qw;
  msg.twist.twist.linear.x = o.vx;
  msg.twist.twist.linear.y = o.vy;
  msg.twist.twist.angular.z = o.wz;
  pub_noise_->publish(msg);
}

void RosOutput::publish(const FrameBundle &b) {
  if (b.hasStereo) {
    const auto stamp = toStamp(b.tImg);
    std_msgs::msg::Header lh, rh;
    lh.stamp = stamp; lh.frame_id = kLeftFrame;
    rh.stamp = stamp; rh.frame_id = kRightFrame;

    auto lmsg = cv_bridge::CvImage(lh, "bgra8", b.left).toImageMsg();
    auto rmsg = cv_bridge::CvImage(rh, "bgra8", b.right).toImageMsg();
    pub_left_->publish(*lmsg);
    pub_right_->publish(*rmsg);

    left_info_.header.stamp = stamp;
    right_info_.header.stamp = stamp;
    pub_left_info_->publish(left_info_);
    pub_right_info_->publish(right_info_);
  }

  if (b.hasGnss) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = toStamp(b.tGnss);
    fix.header.frame_id = kGnssFrame;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = b.lat;
    fix.longitude = b.lon;
    fix.altitude = b.alt;
    // Noise-free GNSS: floor the variance at (0.01 m)^2 like the bridge.
    const double var = 0.0001;
    fix.position_covariance = {var, 0, 0, 0, var, 0, 0, 0, var};
    fix.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    pub_gnss_->publish(fix);
  }
}

}  // namespace carla_cpp
