/*******************************************************
 * carla_cpp: VinsBridge
 *
 * Owns the VINS-Fusion Estimator and feeds it stereo + IMU directly
 * (no ROS in the path). The config sets multiple_thread:0, so both
 * Estimator::inputIMU and Estimator::inputImage drive processMeasurements()
 * inline on the calling thread and mutate shared estimator state without a
 * common lock. We therefore funnel ALL estimator calls onto ONE feeder
 * thread, draining an ordered queue so per-tick IMU is consumed before that
 * tick's stereo (satisfying the estimator's IMUAvailable() wait).
 *
 * Includes estimator.h (OpenCV/Ceres/Eigen/rclcpp) but NOT CARLA headers.
 *******************************************************/
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <thread>

#include "types.hpp"
#include <vins/estimator/estimator.h>

namespace carla_cpp {

class VinsBridge {
 public:
  // `estimator` must already be configured (VINSOptions::readParameters +
  // Estimator::initialize). Feeding-only; odometry/TUM output is handled by
  // VinsPublisher (the main project's Estimator does not self-publish).
  //   use_imu : drop IMU samples for camera-only variants (stereo without IMU).
  //   mono    : feed image0 only (single-camera variants).
  explicit VinsBridge(Estimator &estimator, bool use_imu = true, bool mono = false);
  ~VinsBridge();

  // Thread-safe producers (called from the main tick loop).
  void pushImu(const ImuSample &s);
  // `left`/`right` are CV_8UC4 BGRA; converted to gray before the estimator.
  void pushStereo(const cv::Mat &left, const cv::Mat &right, double t);

  void stop();

 private:
  struct Item {
    bool isImage = false;
    double t = 0.0;
    Eigen::Vector3d acc, gyr;   // IMU
    cv::Mat left, right;        // image (gray)
  };

  void run();                   // feeder thread body

  Estimator &est_;
  bool use_imu_;
  bool mono_;
  std::deque<Item> q_;
  std::mutex m_;
  std::condition_variable cv_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace carla_cpp
