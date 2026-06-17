/*******************************************************
 * carla_cpp: NoiseOdom
 *
 * A cheap "good-enough at t=0" pose sensor: ground-truth pose (in ROS/ENU)
 * plus BOUNDED Gaussian noise. Unlike wheel_odom (which dead-reckons and
 * drifts unboundedly), this stays near the true pose — it models a noisy
 * global sensor (e.g. noisy GPS + compass) and is the right thing for a SHORT
 * bootstrap window: available from the first tick, no initialization, and it
 * won't have drifted meaningfully in ~2 s.
 *
 * noise_level = 0 -> passthrough (exact GT).
 * noise_level > 0 -> ~(0.10 m) position, (0.5 deg) heading, (0.05 m/s) speed
 *                    jitter per unit level, plus a small fixed bias.
 *******************************************************/
#pragma once

#include <algorithm>
#include <cmath>
#include <random>

#include "types.hpp"

namespace carla_cpp {

class NoiseOdom {
 public:
  explicit NoiseOdom(double noise_level = 0.0)
      : noise_(std::max(0.0, noise_level)), rng_(42) {
    std::normal_distribution<double> g(0.0, 1.0);
    bias_x_   = noise_ * 0.05 * g(rng_);   // m, fixed offset
    bias_y_   = noise_ * 0.05 * g(rng_);
    bias_yaw_ = noise_ * 0.005 * g(rng_);  // rad
  }

  // Add noise to a ground-truth odom `gt`, writing the noisy copy to `o`.
  void apply(const EgoOdom &gt, EgoOdom &o) {
    o = gt;
    o.x += bias_x_ + noise_ * 0.10 * nd_(rng_);
    o.y += bias_y_ + noise_ * 0.10 * nd_(rng_);
    double yaw = std::atan2(2.0 * gt.qw * gt.qz, 1.0 - 2.0 * gt.qz * gt.qz);
    yaw += bias_yaw_ + noise_ * 0.0087 * nd_(rng_);  // ~0.5 deg / level
    o.qz = std::sin(yaw / 2.0);
    o.qw = std::cos(yaw / 2.0);
    o.vx += noise_ * 0.05 * nd_(rng_);
    o.vy += noise_ * 0.05 * nd_(rng_);
  }

 private:
  double noise_;
  std::mt19937 rng_;
  std::normal_distribution<double> nd_{0.0, 1.0};
  double bias_x_ = 0.0, bias_y_ = 0.0, bias_yaw_ = 0.0;
};

}  // namespace carla_cpp
