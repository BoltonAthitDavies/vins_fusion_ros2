/*******************************************************
 * carla_cpp: TrajectoryMpc  ('T' / opentopic mode)
 *
 * C++ port of the sampling MPC in
 * folder_for_reference/carla_manual_control.py (_compute_trajectory_control and
 * helpers). Drives the ego toward a single Pose2D target (/trajectory_cmd) by
 * brute-forcing steer x accel candidates, rolling out a kinematic bicycle model
 * over a horizon, and picking the lowest-cost pair. State (x,y,yaw) is the
 * ROS/ENU convention (same as the trajectory target); steering_sign maps the
 * ENU steer back to the CARLA vehicle command.
 *******************************************************/
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include "types.hpp"

namespace carla_cpp {

struct TrajTarget {
  double x = 0.0, y = 0.0, theta = 0.0;
  bool valid = false;
};

class TrajectoryMpc {
 public:
  // (x,y,yaw[rad],speed[m/s]) in ENU; dt = sim fixed step. Returns the control.
  ControlCmd compute(double x, double y, double yaw, double speed,
                     const TrajTarget &target, double dt) {
    if (!target.valid) return stopControl();
    const double dx = target.x - x, dy = target.y - y;
    const double distance = std::hypot(dx, dy);
    if (distance < goal_tol_) { prev_steer_ = 0.0; output_steer_ = 0.0; return stopControl(); }

    const double target_speed = targetSpeedForDistance(distance);
    double steer = 0.0, accel = 0.0;
    solvePoseMpc(x, y, yaw, std::max(0.0, speed), target, target_speed, steer, accel);
    prev_steer_ = steer;
    return toVehicleControl(steer, accel, std::max(0.0, speed), target_speed, dt);
  }

  // Cruise speed the MPC drives at (clamped to max_speed_). Raise it to match a
  // faster reference trajectory (e.g. a GT recorded under autopilot ~5.5 m/s).
  void setTargetSpeed(double v) { target_speed_ = clampd(v, 0.0, max_speed_); }
  double targetSpeed() const { return target_speed_; }
  void setMaxSpeed(double v) { max_speed_ = std::max(0.1, v); }
  // Prediction horizon in steps (× dt_ = seconds). Keep it ~ lookahead/target_speed
  // so the rollout reaches the target without overshooting it (overshoot makes the
  // optimizer pick a slow speed to "sit" on a close target).
  void setHorizon(int n) { horizon_ = std::max(1, n); }
  int horizon() const { return horizon_; }

 private:
  // --- parameters (defaults mirror the reference) ---
  int horizon_ = 100; double dt_ = 0.10;
  double target_speed_ = 3.5, max_speed_ = 8.0, max_accel_ = 2.0, max_decel_ = 4.0;
  double wheelbase_ = 2.875, max_steer_angle_ = 0.60, steering_sign_ = -1.0;
  double goal_tol_ = 0.75, max_throttle_ = 0.45, max_norm_steer_ = 0.65;
  double steer_rate_limit_ = 1.8, steer_filter_alpha_ = 0.35, steer_smooth_w_ = 1.2;
  // --- state ---
  double prev_steer_ = 0.0, output_steer_ = 0.0;

  static double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
  static double normAngle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  double targetSpeedForDistance(double distance) {
    double ts = clampd(target_speed_, 0.0, max_speed_);
    double braking_distance = std::max(0.0, distance - goal_tol_);
    double braking_speed = std::sqrt(std::max(0.0, 2.0 * max_decel_ * braking_distance));
    return std::min(ts, braking_speed);
  }

  std::vector<double> steerCandidates() {
    double center = clampd(prev_steer_, -max_norm_steer_, max_norm_steer_);
    const double offs[] = {-0.30, -0.18, -0.09, 0.0, 0.09, 0.18, 0.30};
    std::set<double> s;
    for (double o : offs) s.insert(clampd(center + o, -max_norm_steer_, max_norm_steer_));
    return std::vector<double>(s.begin(), s.end());
  }

  std::vector<double> accelCandidates(double cur_speed, double target_speed) {
    double desired = clampd((target_speed - cur_speed) / std::max(0.5, horizon_ * dt_),
                            -max_decel_, max_accel_);
    std::set<double> s = {-max_decel_, -0.5 * max_decel_, desired, 0.0,
                          0.5 * max_accel_, max_accel_};
    return std::vector<double>(s.begin(), s.end());
  }

  void solvePoseMpc(double sx, double sy, double syaw, double sspeed,
                    const TrajTarget &t, double target_speed, double &best_steer, double &best_accel) {
    double best_cost = std::numeric_limits<double>::infinity();
    best_steer = 0.0; best_accel = 0.0;
    for (double steer : steerCandidates())
      for (double accel : accelCandidates(sspeed, target_speed)) {
        double c = rolloutCost(sx, sy, syaw, sspeed, t, target_speed, steer, accel);
        if (c < best_cost) { best_cost = c; best_steer = steer; best_accel = accel; }
      }
  }

  double rolloutCost(double x, double y, double yaw, double speed,
                     const TrajTarget &t, double target_speed, double steer, double accel) {
    double steer_angle = steer * max_steer_angle_;
    double cost = 0.12 * steer * steer;
    cost += steer_smooth_w_ * (steer - prev_steer_) * (steer - prev_steer_);
    double an = accel / std::max(1.0e-6, max_accel_ + max_decel_);
    cost += 0.04 * an * an;
    double terminal_dsq = 0.0;
    for (int step = 1; step <= horizon_; ++step) {
      speed = clampd(speed + accel * dt_, 0.0, max_speed_);
      x += speed * std::cos(yaw) * dt_;
      y += speed * std::sin(yaw) * dt_;
      yaw = normAngle(yaw + speed / wheelbase_ * std::tan(steer_angle) * dt_);
      double dx = t.x - x, dy = t.y - y, dsq = dx * dx + dy * dy;
      double line_yaw = std::atan2(dy, dx);
      double line_he = normAngle(line_yaw - yaw);
      double tgt_he = normAngle(t.theta - yaw);
      double speed_err = speed - target_speed;
      double pw = 1.0 + 0.08 * step;
      cost += pw * 1.8 * dsq;
      cost += 1.2 * line_he * line_he;
      cost += 0.25 * tgt_he * tgt_he;
      cost += 0.45 * speed_err * speed_err;
      terminal_dsq = dsq;
    }
    cost += 3.0 * terminal_dsq;
    return cost;
  }

  ControlCmd toVehicleControl(double steer, double accel, double cur_speed, double target_speed, double dt) {
    ControlCmd c;
    double raw = clampd(steering_sign_ * steer, -max_norm_steer_, max_norm_steer_);
    c.steer = static_cast<float>(smoothSteer(raw, dt));
    c.reverse = false; c.hand_brake = false;
    if (target_speed < 0.1) { c.throttle = 0.0f; c.brake = 1.0f; return c; }
    if (accel >= 0.0) {
      double se = std::max(0.0, target_speed - cur_speed);
      c.throttle = static_cast<float>(clampd(0.10 + 0.20 * accel + 0.04 * se, 0.0, max_throttle_));
      c.brake = 0.0f;
    } else {
      c.throttle = 0.0f;
      c.brake = static_cast<float>(clampd(-accel / std::max(1.0e-6, max_decel_), 0.0, 1.0));
    }
    return c;
  }

  double smoothSteer(double raw, double dt) {
    if (dt <= 0.0) dt = dt_;
    double max_delta = std::max(0.0, steer_rate_limit_) * std::max(1.0e-3, dt);
    double limited = output_steer_ + clampd(raw - output_steer_, -max_delta, max_delta);
    double a = clampd(steer_filter_alpha_, 0.0, 1.0);
    output_steer_ = clampd((1.0 - a) * output_steer_ + a * limited, -1.0, 1.0);
    return output_steer_;
  }

  ControlCmd stopControl() {
    ControlCmd c; c.throttle = 0.0f; c.brake = 1.0f; c.steer = 0.0f;
    c.hand_brake = false; c.reverse = false; return c;
  }
};

}  // namespace carla_cpp
