/*******************************************************
 * carla_cpp: carla_vins_node — main
 *
 * A single native C++ process that replaces the Python carla_ros_bridge and
 * the vins_node: it ticks CARLA in synchronous mode via LibCarla, feeds
 * stereo + IMU straight into the VINS-Fusion Estimator (no ROS in the hot
 * path), and republishes only the low-rate topics terminal-3 rtabmap +
 * global_fusion need.
 *
 * Usage:
 *   carla_vins_node <config.yaml> [host=localhost] [port=2000]
 *                   [--spawn x,y,z,roll,pitch,yaw] [--autopilot]
 *******************************************************/
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>

#include <vins/estimator/estimator.h>
#include <vins/estimator/parameters.h>

#include "carla_world.hpp"
#include "frame_collector.hpp"
#include "keyboard_control.hpp"
#include "ros_output.hpp"
#include "types.hpp"
#include "vins_bridge.hpp"
#include "vins_publisher.hpp"

using namespace std::chrono_literals;

// VINS estimator (same global pattern as KITTIOdomTest / rosNodeTest).
Estimator estimator;

namespace {

carla_cpp::CarlaWorld::SpawnPose parseSpawn(const std::string &s) {
  // "x,y,z,roll,pitch,yaw"
  carla_cpp::CarlaWorld::SpawnPose p{100.f, 10.f, 1.f, 0.f, 0.f, -90.f};
  std::vector<float> v;
  std::string tok;
  for (char c : s + ",") {
    if (c == ',') {
      if (!tok.empty()) { v.push_back(std::stof(tok)); tok.clear(); }
    } else {
      tok += c;
    }
  }
  if (v.size() >= 6) { p.x = v[0]; p.y = v[1]; p.z = v[2]; p.roll = v[3]; p.pitch = v[4]; p.yaw = v[5]; }
  return p;
}


}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  cv::theRNG().state = 42;  // seed RANSAC for deterministic feature geometry
  auto plain = rclcpp::remove_ros_arguments(argc, argv);  // strip --ros-args

  if (plain.size() < 2) {
    printf("usage: carla_vins_node <config.yaml> [host] [port] "
           "[--town Town10HD] [--spawn x,y,z,roll,pitch,yaw] [--autopilot]\n");
    return 1;
  }

  std::string config_file = plain[1];
  std::string host = "localhost";
  uint16_t port = 2000;
  std::string spawn_str = "100.0,10.0,1.0,0.0,0.0,-90.0";
  std::string town = "Town10HD";   // matches the old bridge pipeline; "" keeps current map
  bool start_autopilot = false;

  // Parse remaining positional + flag args.
  int positional = 0;
  for (size_t i = 2; i < plain.size(); ++i) {
    const std::string &a = plain[i];
    if (a == "--autopilot") {
      start_autopilot = true;
    } else if (a == "--spawn" && i + 1 < plain.size()) {
      spawn_str = plain[++i];
    } else if (a == "--town" && i + 1 < plain.size()) {
      town = plain[++i];
    } else if (positional == 0) {
      host = a; positional++;
    } else if (positional == 1) {
      port = static_cast<uint16_t>(std::stoi(a)); positional++;
    }
  }

  // ---- VINS estimator setup (refactored API: options + initialize) ----
  auto options = std::make_shared<VINSOptions>();
  options->readParameters(config_file);
  estimator.initialize(options);

  auto node = rclcpp::Node::make_shared("vins_estimator");
  carla_cpp::RosOutput ros(node);

  // The refactored Estimator does not self-publish; VinsPublisher polls
  // getVisualInertialOdom() and publishes /vins_estimator/odometry + /path.
  std::string traj = "/tmp/carla_vio_tum.txt";
  carla_cpp::VinsPublisher pub(node, estimator, traj);
  carla_cpp::VinsBridge vins(estimator);
  carla_cpp::FrameCollector collector;

  carla_cpp::KeyboardControl kb;

  try {
    carla_cpp::CarlaWorld world(host, port, collector);
    world.setup(0.005, parseSpawn(spawn_str), town);

    kb.start();
    if (start_autopilot || !kb.interactive()) {
      world.setAutopilot(true);
      printf("[carla_cpp] autopilot ENABLED%s\n",
             kb.interactive() ? " (press P for manual)" : " (no TTY)");
    }

    // ---- synchronous tick loop ----
    while (rclcpp::ok() && !kb.quitRequested()) {
      kb.poll();
      if (kb.consumeCoverageToggle()) {
        world.setCoverageMode(kb.coverage());
      }
      if (kb.consumeAutopilotToggle()) {
        // Enabling plain autopilot supersedes any active coverage route.
        if (kb.autopilot()) world.setCoverageMode(false);
        world.setAutopilot(kb.autopilot());
        printf("[carla_cpp] autopilot %s\n", kb.autopilot() ? "ON" : "OFF");
      }
      if (kb.manualDriving()) world.applyControl(kb.command());

      uint64_t frame = world.tick();
      auto b = collector.take(frame, 50ms);

      if (!b.imu.empty()) {
        ros.publishClock(b.t);
        carla_cpp::EgoOdom eo;
        if (world.getEgoOdom(eo, b.t)) ros.publishOdometry(eo);  // GT odometry
      }
      for (const auto &s : b.imu) vins.pushImu(s);
      if (b.hasStereo) vins.pushStereo(b.left, b.right, b.tImg);
      ros.publish(b);
    }

    printf("[carla_cpp] shutting down...\n");
    kb.stop();
    vins.stop();   // stop feeding first
    pub.stop();    // then stop reading estimator output
    world.shutdown();
  } catch (const std::exception &e) {
    kb.stop();
    printf("[carla_cpp] FATAL: %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::shutdown();
  return 0;
}
