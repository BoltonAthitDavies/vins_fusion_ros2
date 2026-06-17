/*******************************************************
 * carla_cpp: CarlaWorld
 *
 * Thin facade over LibCarla. It is the ONLY place CARLA headers are
 * included (via pImpl) so the clang-built CARLA toolchain never mixes with
 * rclcpp/estimator headers in other translation units. Responsibilities:
 *   - connect, enable synchronous mode at fixed_delta_seconds
 *   - spawn ego (tesla.model3) + stereo cameras + IMU + GNSS matching
 *     carla_spawn_objects/config/objects.json
 *   - register Listen callbacks that convert + deposit into FrameCollector
 *   - drive the sim via tick(); apply manual control / autopilot
 *******************************************************/
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "frame_collector.hpp"
#include "types.hpp"

namespace carla_cpp {

class CarlaWorld {
 public:
  struct SpawnPose {
    float x = 0.f, y = 0.f, z = 0.f;      // CARLA world location (meters)
    float roll = 0.f, pitch = 0.f, yaw = 0.f;  // degrees
  };

  CarlaWorld(const std::string &host, uint16_t port, FrameCollector &collector);
  ~CarlaWorld();

  // Connect, optionally load `town` (e.g. "Town10HD") if the server is on a
  // different map, apply synchronous settings, spawn ego + sensors, Listen.
  // An empty `town` keeps whatever map is currently loaded.
  void setup(double fixed_delta_seconds, const SpawnPose &ego,
             const std::string &town = "");

  // Advance the simulation one step; returns the produced frame id.
  uint64_t tick(int timeout_ms = 10000);

  void applyControl(const ControlCmd &c);
  void setAutopilot(bool enabled);

  // Ego state in ROS/ENU (x, y, yaw[rad]) + scalar speed [m/s], for the T-mode
  // MPC (matches the trajectory_cmd frame: CARLA y and yaw negated). Returns
  // false if the ego does not exist.
  bool getEgoState(double &x, double &y, double &yaw, double &speed) const;

  // Full ego ground-truth odometry in ROS/ENU, for /carla/ego_vehicle/odometry.
  bool getEgoOdom(EgoOdom &o, double t) const;

  // Inputs for synthesised wheel odometry: signed forward speed [m/s]
  // (speedometer-equivalent) and yaw rate [rad/s] (ENU). CARLA has no wheel
  // encoders, so these come from the body velocity + angular velocity.
  bool getWheelInputs(double &forward_speed, double &yaw_rate) const;

  // "C" right-loop coverage mode (mirrors carla_manual_control): on -> set a
  // Traffic-Manager route + ignore lights/signs + normalize light timing, then
  // hand the ego to autopilot; off -> disable autopilot.
  void setCoverageMode(bool enabled);

  // Stop sensors, restore asynchronous mode, destroy actors.
  void shutdown();

 private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};

}  // namespace carla_cpp
