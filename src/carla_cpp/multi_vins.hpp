/*******************************************************
 * carla_cpp: MultiVins
 *
 * Runs several VINS variants off ONE live CARLA stream, in one process:
 *   mono+imu       -> /vins_mono/odometry
 *   stereo         -> /vins_stereo/odometry        (+ GPS: /vins_stereo_gps/odometry)
 *   stereo+imu     -> /vins_stereo_imu/odometry    (+ GPS: /vins_stereo_imu_gps/odometry)
 *
 * Each variant owns its own Estimator + feeder; the two GPS variants also own
 * an in-process global_fusion GlobalOptimization fed the variant's VIO odom +
 * the live GNSS. A single sim tick feeds all of them, so they see identical
 * data. The synchronous CARLA world paces to the slowest estimator, so there
 * is no real-time deadline.
 *******************************************************/
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vins/estimator/estimator.h>
#include <vins/estimator/parameters.h>

#include "globalOpt.h"
#include "types.hpp"
#include "vins_bridge.hpp"

namespace carla_cpp {

class MultiVins {
 public:
  // cfg_dir holds carla_native_mono.yaml / carla_native_stereo.yaml / carla_native.yaml.
  // `enabled` selects which ESTIMATORS to build ({"mono","stereo","stereo_imu"});
  // empty = all three (back-compat). Running fewer cuts live CPU load so the
  // real-time path keeps up. stereo also yields stereo_gps; stereo_imu yields
  // stereo_imu_gps (so pick "stereo" for /vins_stereo[_gps], "stereo_imu" for
  // /vins_stereo_imu[_gps]).
  MultiVins(rclcpp::Node::SharedPtr node, const std::string &cfg_dir,
            const std::set<std::string> &enabled = {});
  ~MultiVins();

  void pushImu(const ImuSample &s);
  void pushStereo(const cv::Mat &left, const cv::Mat &right, double t);
  void pushGnss(double t, double lat, double lon, double alt);
  void stop();

 private:
  struct Variant {
    std::string name;
    std::shared_ptr<VINSOptions> opts;
    std::shared_ptr<Estimator> est;
    std::unique_ptr<VinsBridge> bridge;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    bool gps = false;
    std::shared_ptr<GlobalOptimization> gopt;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gps;
    std::thread out;
  };

  void addVariant(const std::string &name, const std::string &cfg, bool use_imu,
                  bool mono, bool gps, const std::string &topic,
                  const std::string &gps_topic);
  void outLoop(Variant *v);

  rclcpp::Node::SharedPtr node_;
  std::vector<std::unique_ptr<Variant>> variants_;

  std::mutex gnss_m_;
  double g_t_ = -1.0, g_lat_ = 0.0, g_lon_ = 0.0, g_alt_ = 0.0;

  std::atomic<bool> running_{true};
};

}  // namespace carla_cpp
