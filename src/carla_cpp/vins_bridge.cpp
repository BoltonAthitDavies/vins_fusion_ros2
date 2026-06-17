#include "vins_bridge.hpp"

#include <eigen3/Eigen/Geometry>

#include "conversions.hpp"

namespace carla_cpp {

VinsBridge::VinsBridge(Estimator &estimator, bool use_imu, bool mono)
    : est_(estimator), use_imu_(use_imu), mono_(mono) {
  thread_ = std::thread(&VinsBridge::run, this);
}

VinsBridge::~VinsBridge() { stop(); }

void VinsBridge::stop() {
  bool was = running_.exchange(false);
  if (!was) return;
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void VinsBridge::pushImu(const ImuSample &s) {
  if (!use_imu_) return;  // camera-only variant: ignore IMU
  Item it;
  it.isImage = false;
  it.t = s.t;
  it.acc = s.acc;
  it.gyr = s.gyr;
  {
    std::lock_guard<std::mutex> lk(m_);
    q_.push_back(std::move(it));
  }
  cv_.notify_one();
}

void VinsBridge::pushStereo(const cv::Mat &left, const cv::Mat &right,
                            double t) {
  Item it;
  it.isImage = true;
  it.t = t;
  // Convert to gray here (off the CARLA RPC threads, on the producer/main
  // thread) so the estimator gets the single-channel image it expects.
  it.left = bgraToGray(left);
  if (!mono_) it.right = bgraToGray(right);  // mono variant: image0 only
  {
    std::lock_guard<std::mutex> lk(m_);
    q_.push_back(std::move(it));
  }
  cv_.notify_one();
}

void VinsBridge::run() {
  while (true) {
    Item it;
    {
      std::unique_lock<std::mutex> lk(m_);
      cv_.wait(lk, [&] { return !q_.empty() || !running_.load(); });
      if (!running_.load() && q_.empty()) return;
      it = std::move(q_.front());
      q_.pop_front();
    }
    if (it.isImage) {
      // Refactored API: feed a single ImageData (timestamp + stereo pair).
      ImageData img;
      img.timestamp = it.t;
      img.image0 = it.left;
      img.image1 = it.right;
      est_.inputImage(img);  // drives the optimization (background thread)
    } else {
      IMUData d;
      d.timestamp = it.t;
      d.linear_acceleration = it.acc;
      d.angular_velocity = it.gyr;
      est_.inputIMU(d);
    }
  }
}

}  // namespace carla_cpp
