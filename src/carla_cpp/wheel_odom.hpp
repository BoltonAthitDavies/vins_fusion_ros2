/*******************************************************
 * carla_cpp: WheelOdom
 *
 * Synthesised wheel odometry. CARLA exposes no wheel-encoder ticks, so this
 * dead-reckons from the vehicle's forward speed (speedometer-equivalent) and
 * yaw rate (gyro), starting at the origin — exactly what a real wheel-encoder
 * odometry gives: a RELATIVE estimate available from t=0 with no initialization.
 *
 *   x   += v · cos(yaw) · dt
 *   y   += v · sin(yaw) · dt
 *   yaw += w · dt
 *
 * noise_level = 0 -> clean (CARLA speed is exact, so the result is ~drift-free,
 *                   fine purely to bootstrap motion before VINS converges).
 * noise_level > 0 -> realistic: a once-per-run scale error + bias (wheel-radius /
 *                   slip miscalibration) plus per-step jitter, so it DRIFTS like
 *                   a real wheel odometry that nothing bounds (use GPS to bound).
 *******************************************************/
#pragma once

#include <algorithm>
#include <cmath>
#include <random>

#include "types.hpp"

namespace carla_cpp {

class WheelOdom {
 public:
  explicit WheelOdom(double noise_level = 0.0)
      : noise_(std::max(0.0, noise_level)), rng_(42) {
    std::normal_distribution<double> g(0.0, 1.0);
    // Once-per-run systematic errors (constant -> integrates into steady drift).
    scale_v_ = 1.0 + noise_ * 0.02 * g(rng_);   // ~2 % wheel-radius / slip scale
    bias_v_  = noise_ * 0.05 * g(rng_);          // m/s offset
    scale_w_ = 1.0 + noise_ * 0.02 * g(rng_);
    bias_w_  = noise_ * 0.003 * g(rng_);         // rad/s offset
  }

  // Integrate one step from (signed forward speed [m/s], yaw rate [rad/s]) over
  // dt; fill o (own frame, starts at origin). t = sim time for the stamp.
  void step(double fwd_speed, double yaw_rate, double dt, double t, EgoOdom &o) {
    double v = fwd_speed * scale_v_ + bias_v_ + noise_ * 0.02 * nd_(rng_);
    double w = yaw_rate  * scale_w_ + bias_w_ + noise_ * 0.005 * nd_(rng_);
    x_ += v * std::cos(yaw_) * dt;
    y_ += v * std::sin(yaw_) * dt;
    yaw_ += w * dt;
    if (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    else if (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    o.t = t; o.x = x_; o.y = y_; o.z = 0.0;
    o.qz = std::sin(yaw_ / 2.0);
    o.qw = std::cos(yaw_ / 2.0);
    o.vx = v * std::cos(yaw_);
    o.vy = v * std::sin(yaw_);
    o.vz = 0.0;
    o.wz = w;
  }

 private:
  double noise_;
  std::mt19937 rng_;
  std::normal_distribution<double> nd_{0.0, 1.0};
  double scale_v_ = 1.0, bias_v_ = 0.0, scale_w_ = 1.0, bias_w_ = 0.0;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
};

}  // namespace carla_cpp
