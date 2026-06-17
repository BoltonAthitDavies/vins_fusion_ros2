/*******************************************************
 * carla_cpp: per-tick sensor collation.
 *
 * CARLA's Sensor::Listen callbacks fire on RPC worker threads. In
 * synchronous mode World::Tick(frame) returns only after the server has
 * produced and dispatched that frame's sensor data, but the user-side
 * callbacks may still be completing. FrameCollector stages deposits keyed
 * by CARLA frame id and lets the main loop pull a completed bundle with a
 * short condition-variable wait as a safety net.
 *
 * OpenCV/Eigen only — no CARLA, no ROS, no estimator headers.
 *******************************************************/
#pragma once

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <utility>

#include "types.hpp"

namespace carla_cpp {

class FrameCollector {
 public:
  // Deposit calls — invoked from CARLA RPC callback threads.
  void addImu(uint64_t frame, double t, const Eigen::Vector3d &acc,
              const Eigen::Vector3d &gyr) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto &b = slot(frame);
    b.t = t;
    b.imu.push_back(ImuSample{t, acc, gyr});
    cv_.notify_all();
  }

  // `left` selects which camera; both share the tick's frame id/timestamp.
  void addImage(uint64_t frame, double t, bool left, cv::Mat bgra) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto &b = slot(frame);
    if (left) b.left = std::move(bgra);
    else      b.right = std::move(bgra);
    b.tImg = t;
    cv_.notify_all();
  }

  void addGnss(uint64_t frame, double t, double lat, double lon, double alt) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto &b = slot(frame);
    b.hasGnss = true;
    b.lat = lat; b.lon = lon; b.alt = alt; b.tGnss = t;
    cv_.notify_all();
  }

  // Called by the main loop right after World::Tick(frame). Waits up to
  // `timeout` for this frame's IMU sample (which fires every tick, so its
  // arrival means the frame's callbacks are landing); if exactly one image
  // of a stereo pair has arrived, waits a little longer for its partner so
  // we never feed VINS half a pair. Returns the bundle and drops the slot
  // plus any stale older slots.
  FrameBundle take(uint64_t frame, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(mtx_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    // Cameras run slower than the tick (e.g. 10 Hz vs 80 Hz), and their callbacks
    // may land just after take() runs. A stereo pair is "due" once ~a camera
    // period has elapsed since the last delivered pair; on those frames we wait
    // for BOTH images (else the slot gets erased before the late frame lands and
    // stereo is never assembled). On non-due frames we return immediately so the
    // loop stays fast.
    const bool camera_due = (frame - last_paired_) >= kCameraPeriodTicks;
    auto ready = [&] {
      auto it = pending_.find(frame);
      if (it == pending_.end() || it->second.imu.empty()) return false;
      const auto &b = it->second;
      const bool both = !b.left.empty() && !b.right.empty();
      if (both) return true;
      const bool oneSide = (!b.left.empty()) != (!b.right.empty());
      if (oneSide) return false;     // half a pair -> always wait for its partner
      return !camera_due;            // both absent -> wait only if a pair is due
    };
    cv_.wait_until(lk, deadline, ready);

    FrameBundle out;
    auto it = pending_.find(frame);
    if (it != pending_.end()) {
      out = std::move(it->second);
      out.frame = frame;
      out.hasStereo = !out.left.empty() && !out.right.empty();
      if (!out.imu.empty() && out.t == 0.0) out.t = out.imu.back().t;
    }
    if (out.hasStereo) last_paired_ = frame;
    // Erase this and any older (already-consumed or dropped) frames.
    pending_.erase(pending_.begin(), pending_.upper_bound(frame));
    return out;
  }

 private:
  // ~camera period in ticks (world_rate / camera_rate); wait a hair early.
  // 200 Hz world / 20 Hz camera = 10 ticks/frame -> 9. (Was 7 for 80/10 Hz.)
  // MUST be updated if the world tick or camera sensor_tick changes.
  static constexpr uint64_t kCameraPeriodTicks = 9;
  uint64_t last_paired_ = 0;
  FrameBundle &slot(uint64_t frame) {
    auto &b = pending_[frame];
    b.frame = frame;
    return b;
  }

  std::mutex mtx_;
  std::condition_variable cv_;
  std::map<uint64_t, FrameBundle> pending_;
};

}  // namespace carla_cpp
