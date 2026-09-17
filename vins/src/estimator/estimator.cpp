/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <vins/estimator/estimator.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sys/resource.h>
#include <unistd.h>

namespace {
double processCpuMs() {
  struct rusage usage {};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return -1.0;
  return usage.ru_utime.tv_sec * 1e3 + usage.ru_utime.tv_usec * 1e-3 +
         usage.ru_stime.tv_sec * 1e3 + usage.ru_stime.tv_usec * 1e-3;
}

long residentMemoryKb() {
  std::ifstream statm("/proc/self/statm");
  long total_pages = 0;
  long resident_pages = 0;
  if (!(statm >> total_pages >> resident_pages)) return -1;
  (void)total_pages;
  return resident_pages * sysconf(_SC_PAGESIZE) / 1024;
}
}  // namespace

Estimator::Estimator() {}

Estimator::~Estimator() {
  if (isRunning.load()) {
    isRunning.store(false);
    imuCondition.notify_all();
    featureCondition.notify_all();
    if (processThread.joinable()) {
      processThread.join();
    }
  }
  logEvent(last_processed_timestamp_, "shutdown", "graceful");
  writeRunSummary();
}

void Estimator::resetState() {
  ++total_resets_;
  {
    std::lock_guard<std::mutex> imu_lock(imu_mutex);
    clearBuffer(imuBuffer);
  }
  {
    std::lock_guard<std::mutex> feature_lock(featureBufferMutex);
    clearBuffer(featureBuffer);
    clearBuffer(featureEnqueueTimes);
    clearBuffer(featureFromExternalSource);
  }

  // NOTE: resetState() is only called from processNonLinearSolver's failure/gap-reset
  // paths, which already hold processingMutex (taken in processMeasurements around
  // processImage). Re-locking a std::mutex here would self-deadlock and hang the node on
  // every reset -- which is exactly what happened. Rely on the caller's lock instead.
  previousTimestamp = -1;
  currentTimestamp = 0;
  openExEstimation = 0;
  inputImageCount = 0;
  isFirstPoseInitialized = false;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    estimator_state[i].clear();
    std::vector<IMUData>().swap(deltaTimeImuBuffer[i]);
    pre_integrations[i] = nullptr;
  }

  for (int i = 0; i < options->getNumCameras(); i++) {
    cameraTranslation[i] = Vector3d::Zero();
    cameraRotation[i] = Matrix3d::Identity();
    updateCameraPose(i);
  }

  isFirstIMUReceived = false, backCount = 0;
  frontCount = 0;
  frameCount = 0;
  solver_flag = SolverState::INITIAL;
  initialTimestamp = 0;
  std::map<double, ImageFrame>().swap(all_image_frame);
  tmp_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  featureManager.clearState();
  failure_occur = 0;
  VINS_INFO << "reset state successfully";
}

void Estimator::initialize(std::shared_ptr<VINSOptions> options_) {
  std::lock_guard<std::mutex> lock(processingMutex);
  options = options_;
  initializeCamerasFromOptions();
  if (!options->OUTPUT_FOLDER.empty()) {
    performance_log_.open(options->OUTPUT_FOLDER + "/performance.csv",
                          std::ios::out | std::ios::trunc);
    events_log_.open(options->OUTPUT_FOLDER + "/events.csv",
                     std::ios::out | std::ios::trunc);
    frontend_log_.open(options->OUTPUT_FOLDER + "/frontend.csv",
                       std::ios::out | std::ios::trunc);
    backend_log_.open(options->OUTPUT_FOLDER + "/backend.csv",
                      std::ios::out | std::ios::trunc);
    state_log_.open(options->OUTPUT_FOLDER + "/estimator_state.csv",
                    std::ios::out | std::ios::trunc);
    if (performance_log_) {
      performance_log_
          << "timestamp_ns,solver_state,initialized_this_frame,keyframe,"
             "marginalization,processing_ms,imu_samples,feature_count,"
             "feature_backlog,images_received,images_enqueued,images_processed,"
             "imu_received,imu_used,poses_written,resets,imu_gap_resets,"
             "failure_resets,dynamic_points_dropped,process_cpu_ms,"
             "wall_elapsed_ms,resident_memory_kb\n";
      performance_log_.flush();
    } else {
      VINS_WARN << "Could not open performance.csv in " << options->OUTPUT_FOLDER;
    }
    if (events_log_) {
      events_log_ << "timestamp_ns,event,detail\n";
      events_log_.flush();
      logEvent(0.0, "startup", "estimator_initialized");
    } else {
      VINS_WARN << "Could not open events.csv in " << options->OUTPUT_FOLDER;
    }
    if (frontend_log_) {
      frontend_log_
          << "timestamp_ns,feature_tracking_ms,feature_count,stereo_feature_count,"
             "enqueued,feature_backlog,images_received,images_enqueued,"
             "dynamic_points_dropped\n";
      frontend_log_.flush();
    } else {
      VINS_WARN << "Could not open frontend.csv in " << options->OUTPUT_FOLDER;
    }
    if (backend_log_) {
      backend_log_
          << "timestamp_ns,solver_state,initialized_this_frame,keyframe,marginalization,"
             "external_feature_source,"
             "feature_queue_wait_ms,imu_wait_ms,imu_propagation_ms,visual_update_ms,"
             "initialization_ms,triangulation_ms,parameter_preparation_ms,"
             "problem_construction_ms,solver_ms,estimate_update_ms,marginalization_ms,"
             "outlier_rejection_ms,failure_detection_ms,slide_window_ms,state_output_ms,"
             "estimator_update_ms,pointcloud_ms,backend_total_ms,imu_samples,input_features,"
             "tracked_features,new_features,long_tracks,average_parallax_px,managed_features,"
             "optimized_features,outliers_removed,solver_calls,solver_iterations,"
             "solver_parameter_blocks,solver_residual_blocks,solver_residuals,"
             "solver_termination_type,solver_solution_usable,solver_initial_cost,"
             "solver_final_cost,feature_backlog,accel_bias_norm,gyro_bias_norm,velocity_norm\n";
      backend_log_.flush();
    } else {
      VINS_WARN << "Could not open backend.csv in " << options->OUTPUT_FOLDER;
    }
    if (state_log_) {
      state_log_
          << "timestamp_ns,solver_state,frame_index,px,py,pz,qw,qx,qy,qz,vx,vy,vz,"
             "accel_bias_x,accel_bias_y,accel_bias_z,gyro_bias_x,gyro_bias_y,gyro_bias_z,"
             "time_delay_s\n";
      state_log_.flush();
    } else {
      VINS_WARN << "Could not open estimator_state.csv in " << options->OUTPUT_FOLDER;
    }
  }
  isRunning.store(true);
  processThread = std::thread(&Estimator::processMeasurements, this);
}

void Estimator::initializeCamerasFromOptions() {
  for (int i = 0; i < options->getNumCameras(); i++) {
    cameraTranslation[i] = options->TIC[i];
    cameraRotation[i] = options->RIC[i];
    updateCameraPose(i);
    VINS_INFO << " exitrinsic cam[" << i << "]: \n"
              << cameraRotation[i] << "\n"
              << cameraTranslation[i].transpose();
  }
  featureManager.setOptions(options);
  featureTracker.setOptions(options);
  ProjectionTwoFrameOneCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionTwoFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionOneFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  gravity = options->imu.gravity();
  featureTracker.readIntrinsicParameter(options->camera_names);
}

void Estimator::inputImage(const ImageData &image) {
  ++total_images_received_;
  inputImageCount++;
  FeatureFrame featureFrame;
  TicToc featureTrackerTime;

  if (image.image1.empty()) {
    featureFrame = featureTracker.trackImage(image.timestamp, image.image0,
                                             cv::Mat(), image.mask);
  } else {
    featureFrame = featureTracker.trackImage(image.timestamp, image.image0,
                                             image.image1, image.mask);
  }
  const double feature_tracking_ms = featureTrackerTime.toc();
  if (options->shouldShowTrack()) {
    track_image.image0 = featureTracker.getTrackImage();
    track_image.timestamp = image.timestamp;
    safe_track_image.set(track_image);
  }

  bool enqueued = false;
  std::size_t feature_backlog = 0;
  {
    std::lock_guard<std::mutex> lock(featureBufferMutex);
    if (inputImageCount % options->imageSkip() == 0 || featureBuffer.empty()) {
      featureBuffer.push(make_pair(image.timestamp, featureFrame));
      featureEnqueueTimes.push(std::chrono::steady_clock::now());
      featureFromExternalSource.push(false);
      ++total_images_enqueued_;
      ++total_image_frames_enqueued_;
      enqueued = true;
    }
    feature_backlog = featureBuffer.size();
  }
  if (enqueued) {
    featureCondition.notify_one();
  }

  std::size_t stereo_feature_count = 0;
  for (const auto &feature : featureFrame) {
    if (feature.second.size() > 1) ++stereo_feature_count;
  }
  ++frontend_rows_;
  {
    std::lock_guard<std::mutex> lock(frontend_log_mutex_);
    if (frontend_log_) {
      frontend_log_.setf(std::ios::fixed, std::ios::floatfield);
      frontend_log_.precision(0);
      frontend_log_ << image.timestamp * 1e9 << ',';
      frontend_log_.precision(3);
      frontend_log_ << feature_tracking_ms << ',' << featureFrame.size() << ','
                    << stereo_feature_count << ',' << (enqueued ? 1 : 0) << ','
                    << feature_backlog << ',' << total_images_received_.load() << ','
                    << total_image_frames_enqueued_.load() << ','
                    << featureTracker.dynamic_dropped << '\n';
      frontend_log_.flush();
    }
  }
}

void Estimator::inputIMU(const IMUData &imu) {
  ++total_imu_received_;
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    imuBuffer.push(imu);
  }
  imuCondition.notify_all();

  if (solver_flag == SolverState::NON_LINEAR) {
    fastPredictIMU(imu);
  }
}

void Estimator::inputFeature(double timestamp,
                             const FeatureFrame &featureFrame) {
  ++total_external_feature_frames_;
  {
    std::lock_guard<std::mutex> lock(featureBufferMutex);
    featureBuffer.push(make_pair(timestamp, featureFrame));
    featureEnqueueTimes.push(std::chrono::steady_clock::now());
    featureFromExternalSource.push(true);
    ++total_images_enqueued_;
  }
  featureCondition.notify_one();
}

bool Estimator::getIMUInterval(double startTime, double endTime,
                               vector<IMUData> &data) {
  std::lock_guard<std::mutex> lock(imu_mutex);
  if (imuBuffer.empty()) {
    VINS_ERROR << "No IMU data received";
    return false;
  }

  if (endTime <= imuBuffer.back().timestamp) {
    while (imuBuffer.front().timestamp <= startTime) {
      imuBuffer.pop();
    }

    while (imuBuffer.front().timestamp < endTime) {
      data.push_back(imuBuffer.front());
      imuBuffer.pop();
    }
    if (!imuBuffer.empty()) {
      data.push_back(imuBuffer.front());
    }
    return true;
  }

  VINS_WARN << "Waiting for IMU data";
  return false;
}

bool Estimator::IMUAvailable(double t) {
  return !imuBuffer.empty() && t <= imuBuffer.back().timestamp;
}

void Estimator::processMeasurements() {
  while (isRunning.load()) {
    TimestampedFeatureFrame feature;
    std::chrono::steady_clock::time_point feature_enqueued_at;
    bool external_feature_source = false;
    vector<IMUData> imu_datas;
    {
      std::unique_lock<std::mutex> lock(featureBufferMutex);
      featureCondition.wait(
          lock, [this] { return !featureBuffer.empty() || !isRunning.load(); });
      if (!isRunning.load() || featureBuffer.empty()) break;
      feature = featureBuffer.front();
      featureBuffer.pop();
      if (!featureEnqueueTimes.empty()) {
        feature_enqueued_at = featureEnqueueTimes.front();
        featureEnqueueTimes.pop();
      } else {
        // Defensive fallback for old/direct producers. Every in-tree producer
        // pushes both queues while holding featureBufferMutex.
        feature_enqueued_at = std::chrono::steady_clock::now();
      }
      if (!featureFromExternalSource.empty()) {
        external_feature_source = featureFromExternalSource.front();
        featureFromExternalSource.pop();
      }
      lock.unlock();
    }
    const auto backend_start = std::chrono::steady_clock::now();
    frame_metrics_ = FrameMetrics{};
    frame_metrics_.feature_queue_wait_ms =
        std::chrono::duration<double, std::milli>(backend_start - feature_enqueued_at).count();
    currentTimestamp = feature.first + options->time_delay;
    if (options->hasImu()) {
      const auto imu_wait_start = std::chrono::steady_clock::now();
      std::unique_lock<std::mutex> lock(imu_mutex);

      imuCondition.wait(lock, [this] {
        return IMUAvailable(currentTimestamp) || !isRunning.load();
      });
      if (!isRunning.load()) {
        break;
      }
      frame_metrics_.imu_wait_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - imu_wait_start).count();
    }

    const auto imu_propagation_start = std::chrono::steady_clock::now();
    if (options->hasImu()) {
      getIMUInterval(previousTimestamp, currentTimestamp, imu_datas);
      total_imu_used_ += imu_datas.size();
      if (!isFirstPoseInitialized) initFirstIMUPose(imu_datas);
      for (size_t i = 0; i < imu_datas.size(); i++) {
        double dt;
        if (i == 0)
          dt = imu_datas[i].timestamp - previousTimestamp;
        else if (i == imu_datas.size() - 1)
          dt = currentTimestamp - imu_datas[i - 1].timestamp;
        else
          dt = imu_datas[i].timestamp - imu_datas[i - 1].timestamp;
        processIMU(imu_datas[i], dt);
      }
    }
    frame_metrics_.imu_propagation_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - imu_propagation_start).count();
    TicToc t_proc;
    const SolverState solver_before = solver_flag;
    {
      std::lock_guard<std::mutex> lock(processingMutex);
      const auto estimator_update_start = std::chrono::steady_clock::now();
      processImage(feature.second, feature.first);
      frame_metrics_.estimator_update_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - estimator_update_start).count();
      const auto pointcloud_start = std::chrono::steady_clock::now();
      printStatistics(currentTimestamp);
      collectPointCloudAll(feature.first);
      frame_metrics_.pointcloud_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - pointcloud_start).count();
      previousTimestamp = currentTimestamp;
    }
    const double processing_ms = t_proc.toc();
    frame_metrics_.backend_total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - backend_start).count();
    ++total_images_processed_;
    if (first_processed_timestamp_ < 0.0) {
      first_processed_timestamp_ = feature.first;
    }
    last_processed_timestamp_ = feature.first;
    const bool initialized_this_frame =
        solver_before != SolverState::NON_LINEAR &&
        solver_flag == SolverState::NON_LINEAR;
    if (initialized_this_frame) {
      ++initialization_events_;
      if (first_initialized_timestamp_ < 0.0) {
        first_initialized_timestamp_ = feature.first;
      }
      logEvent(feature.first, "initialization_complete", "solver_non_linear");
    }
    const bool keyframe = solver_flag == SolverState::NON_LINEAR &&
        marginalization_flag == MarginalizationType::MARGIN_OLD;
    if (keyframe) {
      ++total_keyframes_;
    }
    // Real-time profiling: per-frame wall-clock processing cost and the feature backlog.
    // If proc_ms stays under the inter-keyframe period and backlog stays small, the node
    // keeps up; a growing backlog means it cannot process this stream in real time.
    size_t backlog;
    {
      std::lock_guard<std::mutex> fl(featureBufferMutex);
      backlog = featureBuffer.size();
    }
    const int active_state_index = std::min(frameCount, WINDOW_SIZE);
    const auto &active_state = estimator_state[active_state_index];
    const std::size_t managed_features = featureManager.feature.size();
    const int optimized_features = featureManager.getFeatureCount();

    ++backend_rows_;
    if (backend_log_) {
      backend_log_.setf(std::ios::fixed, std::ios::floatfield);
      backend_log_.precision(0);
      backend_log_ << feature.first * 1e9 << ','
          << (solver_flag == SolverState::NON_LINEAR ? 1 : 0) << ','
          << (initialized_this_frame ? 1 : 0) << ',' << (keyframe ? 1 : 0) << ','
          << (marginalization_flag == MarginalizationType::MARGIN_OLD ? 0 : 1) << ','
          << (external_feature_source ? 1 : 0) << ',';
      backend_log_.precision(3);
      backend_log_
          << frame_metrics_.feature_queue_wait_ms << ',' << frame_metrics_.imu_wait_ms << ','
          << frame_metrics_.imu_propagation_ms << ',' << frame_metrics_.visual_update_ms << ','
          << frame_metrics_.initialization_ms << ',' << frame_metrics_.triangulation_ms << ','
          << frame_metrics_.parameter_preparation_ms << ','
          << frame_metrics_.problem_construction_ms << ',' << frame_metrics_.solver_ms << ','
          << frame_metrics_.estimate_update_ms << ',' << frame_metrics_.marginalization_ms << ','
          << frame_metrics_.outlier_rejection_ms << ','
          << frame_metrics_.failure_detection_ms << ',' << frame_metrics_.slide_window_ms << ','
          << frame_metrics_.state_output_ms << ',' << frame_metrics_.estimator_update_ms << ','
          << frame_metrics_.pointcloud_ms << ',' << frame_metrics_.backend_total_ms << ','
          << imu_datas.size() << ',' << feature.second.size() << ','
          << featureManager.last_track_num << ',' << featureManager.new_feature_num << ','
          << featureManager.long_track_num << ',' << featureManager.last_average_parallax << ','
          << managed_features << ',' << optimized_features << ','
          << frame_metrics_.outliers_removed << ',' << frame_metrics_.solver_calls << ','
          << frame_metrics_.solver_iterations << ',' << frame_metrics_.solver_parameter_blocks << ','
          << frame_metrics_.solver_residual_blocks << ',' << frame_metrics_.solver_residuals << ','
          << frame_metrics_.solver_termination_type << ','
          << (frame_metrics_.solver_solution_usable ? 1 : 0) << ','
          << frame_metrics_.solver_initial_cost << ',' << frame_metrics_.solver_final_cost << ','
          << backlog << ',' << active_state.accel_bias.norm() << ','
          << active_state.gyro_bias.norm() << ',' << active_state.velocity.norm() << '\n';
      backend_log_.flush();
    }
    if (state_log_) {
      const Quaterniond q(active_state.rotation);
      state_log_.setf(std::ios::fixed, std::ios::floatfield);
      state_log_.precision(0);
      state_log_ << feature.first * 1e9 << ','
                 << (solver_flag == SolverState::NON_LINEAR ? 1 : 0) << ','
                 << active_state_index << ',';
      state_log_.precision(9);
      state_log_ << active_state.position.x() << ',' << active_state.position.y() << ','
                 << active_state.position.z() << ',' << q.w() << ',' << q.x() << ','
                 << q.y() << ',' << q.z() << ',' << active_state.velocity.x() << ','
                 << active_state.velocity.y() << ',' << active_state.velocity.z() << ','
                 << active_state.accel_bias.x() << ',' << active_state.accel_bias.y() << ','
                 << active_state.accel_bias.z() << ',' << active_state.gyro_bias.x() << ','
                 << active_state.gyro_bias.y() << ',' << active_state.gyro_bias.z() << ','
                 << options->time_delay << '\n';
      state_log_.flush();
    }
    VINS_INFO << "PERF t=" << currentTimestamp << " proc_ms=" << processing_ms
              << " backlog=" << backlog << " recv=" << inputImageCount;
    if (performance_log_) {
      performance_log_.setf(std::ios::fixed, std::ios::floatfield);
      performance_log_.precision(0);
      performance_log_ << feature.first * 1e9 << ','
          << (solver_flag == SolverState::NON_LINEAR ? 1 : 0) << ','
          << (initialized_this_frame ? 1 : 0) << ',' << (keyframe ? 1 : 0) << ','
          << (marginalization_flag == MarginalizationType::MARGIN_OLD ? 0 : 1) << ',';
      performance_log_.precision(3);
      performance_log_ << processing_ms << ',' << imu_datas.size() << ','
          << feature.second.size() << ',' << backlog << ','
          << total_images_received_.load() << ',' << total_images_enqueued_.load() << ','
          << total_images_processed_.load() << ',' << total_imu_received_.load() << ','
          << total_imu_used_.load() << ',' << total_poses_written_.load() << ','
          << total_resets_.load() << ',' << imu_gap_resets_.load() << ','
          << failure_resets_.load() << ',' << featureTracker.dynamic_dropped << ','
          << processCpuMs() << ','
          << std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - process_start_).count()
          << ',' << residentMemoryKb() << '\n';
      performance_log_.flush();
    }
  }
}

void Estimator::logEvent(Timestamp timestamp, const std::string &event,
                         const std::string &detail) {
  if (!events_log_) return;
  events_log_.setf(std::ios::fixed, std::ios::floatfield);
  events_log_.precision(0);
  events_log_ << timestamp * 1e9 << ',' << event << ',' << detail << '\n';
  events_log_.flush();
}

void Estimator::writeRunSummary() {
  if (!options || options->OUTPUT_FOLDER.empty()) return;
  std::ofstream out(options->OUTPUT_FOLDER + "/run_summary.csv",
                    std::ios::out | std::ios::trunc);
  if (!out) return;
  const double initialization_time =
      first_processed_timestamp_ >= 0.0 && first_initialized_timestamp_ >= 0.0
          ? first_initialized_timestamp_ - first_processed_timestamp_
          : -1.0;
  out << "key,value\n"
      << "shutdown_status,graceful\n"
      << "images_received," << total_images_received_.load() << '\n'
      << "images_enqueued," << total_images_enqueued_.load() << '\n'
      << "image_frames_enqueued," << total_image_frames_enqueued_.load() << '\n'
      << "external_feature_frames_received,"
      << total_external_feature_frames_.load() << '\n'
      << "images_processed," << total_images_processed_.load() << '\n'
      << "frontend_rows," << frontend_rows_.load() << '\n'
      << "backend_rows," << backend_rows_.load() << '\n'
      << "images_skipped_before_backend,"
      << (total_images_received_.load() >= total_image_frames_enqueued_.load()
              ? total_images_received_.load() - total_image_frames_enqueued_.load()
              : 0) << '\n'
      << "imu_messages_received," << total_imu_received_.load() << '\n'
      << "imu_samples_used," << total_imu_used_.load() << '\n'
      << "poses_written," << total_poses_written_.load() << '\n'
      << "keyframes," << total_keyframes_.load() << '\n'
      << "initialization_events," << initialization_events_.load() << '\n'
      << "initialization_time_s," << std::fixed << std::setprecision(6)
      << initialization_time << '\n'
      << "resets," << total_resets_.load() << '\n'
      << "imu_gap_resets," << imu_gap_resets_.load() << '\n'
      << "failure_resets," << failure_resets_.load() << '\n'
      << "failure_detection_enabled,0\n"
      << "solver_calls," << total_solver_calls_.load() << '\n'
      << "outliers_removed," << total_outliers_removed_.load() << '\n'
      << "dynamic_points_dropped," << featureTracker.dynamic_dropped << '\n';
}
void Estimator::updateCameraPose(int index) {
  PoseData pose;
  pose.position = cameraTranslation[index];
  pose.orientation = Quaterniond(cameraRotation[index]);
  safe_camera_pose[index].set(pose);
}

void Estimator::collectPointCloudAll(Timestamp timestamp) {
  PointCloudData main_cloud;
  PointCloudData point_cloud;
  PointCloudData margin_cloud;
  main_cloud.timestamp = timestamp;
  point_cloud.timestamp = timestamp;
  margin_cloud.timestamp = timestamp;

  for (const auto &it_per_id : featureManager.feature) {
    int used_num = it_per_id.feature_per_frame.size();
    int start_frame = it_per_id.start_frame;

    // Only use well-tracked and solved features
    if (used_num < 2 || !it_per_id.isSolved()) continue;

    int imu_i = start_frame;
    const auto &first_obs = it_per_id.feature_per_frame[0];
    Eigen::Vector3d pts_i = first_obs.point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i =
        estimator_state[imu_i].rotation *
            (cameraRotation[0] * pts_i + cameraTranslation[0]) +
        estimator_state[imu_i].position;

    // 1. Main cloud
    if (start_frame < WINDOW_SIZE - 2 && start_frame <= WINDOW_SIZE * 3 / 4) {
      main_cloud.points.emplace_back(w_pts_i);
    }

    // 2. Margin cloud
    if (start_frame == 0 && used_num <= 2) {
      margin_cloud.points.emplace_back(w_pts_i);
    }

    // 3. Current frame cloud
    if (solver_flag == SolverState::NON_LINEAR &&
        marginalization_flag == MarginalizationType::MARGIN_OLD) {
      if (start_frame < WINDOW_SIZE - 2 &&
          start_frame + used_num - 1 >= WINDOW_SIZE - 2) {
        int imu_j = WINDOW_SIZE - 2 - start_frame;
        const auto &cur_obs = it_per_id.feature_per_frame[imu_j];
        point_cloud.points.emplace_back(w_pts_i);

        ChannelFloat p_2d;
        p_2d.values.push_back(cur_obs.point.x());
        p_2d.values.push_back(cur_obs.point.y());
        p_2d.values.push_back(cur_obs.uv.x());
        p_2d.values.push_back(cur_obs.uv.y());
        p_2d.values.push_back(it_per_id.feature_id);
        point_cloud.channels.push_back(p_2d);
      }
    }
  }
  if (!main_cloud.points.empty()) {
    safe_main_cloud.set(main_cloud);
  }
  if (!margin_cloud.points.empty()) {
    safe_margin_cloud.set(margin_cloud);
  }
  if (!point_cloud.points.empty()) {
    safe_point_cloud.set(point_cloud);
  }
  if (solver_flag == SolverState::NON_LINEAR &&
      marginalization_flag == MarginalizationType::MARGIN_OLD) {
    int i = WINDOW_SIZE - 2;
    PoseData pose;
    pose.timestamp = estimator_state[i].timestamp;
    pose.position = estimator_state[i].position;
    pose.orientation = Quaterniond(estimator_state[i].rotation);
    safe_keyframe_pose.set(pose);
  }
}

void Estimator::printStatistics(Timestamp timestamp) {
  if (solver_flag != SolverState::NON_LINEAR) return;
  VINS_DEBUG << "position: (" << estimator_state[WINDOW_SIZE].position.x()
             << "," << estimator_state[WINDOW_SIZE].position.y() << ","
             << estimator_state[WINDOW_SIZE].position.z() << ")";
}

void Estimator::initFirstIMUPose(const vector<IMUData> &data) {
  isFirstPoseInitialized = true;
  Vector3d averageAcceleration = Vector3d::Zero();

  for (const auto &imu : data) {
    averageAcceleration += imu.linear_acceleration;
  }

  averageAcceleration /= data.size();
  VINS_INFO << "Average acceleration: " << averageAcceleration.transpose();

  Matrix3d initialRotation = Utility::g2R(averageAcceleration);
  double yaw = Utility::R2ypr(initialRotation).x();
  initialRotation = Utility::ypr2R(Vector3d{-yaw, 0, 0}) * initialRotation;
  estimator_state[0].rotation = initialRotation;
  VINS_INFO << "init R0: \n" << estimator_state[0].rotation;
}

void Estimator::setFirstPose(const Eigen::Vector3d &position,
                             const Eigen::Matrix3d &rotation) {
  estimator_state[0].position = position;
  estimator_state[0].rotation = rotation;
}

void Estimator::processIMU(const IMUData &data, double deltaTime) {
  if (!isFirstIMUReceived) {
    isFirstIMUReceived = true;
    previousImuData = data;
  }

  // Guard against abnormally large IMU intervals (e.g. messages dropped while the node
  // falls behind real time). Integrating a multi-second dt explodes BOTH the
  // pre-integration covariance and the propagated state (updateStateWithIMU) into NaN,
  // which aborts the Ceres solver. Clamping here keeps the pre-integration and the state
  // update consistent and finite so the estimator recovers instead of crashing.
  const double max_imu_dt = IntegrationBase::MAX_IMU_DT;
  if (deltaTime > max_imu_dt) {
    VINS_WARN << "Large IMU interval " << deltaTime
              << "s (likely dropped messages); clamping to " << max_imu_dt;
    deltaTime = max_imu_dt;
    // If we are already tracking, a gap this large means the window's visual and inertial
    // constraints are no longer mutually consistent. Flag a re-initialization rather than
    // limp on with a clamped factor that would NaN the solver next frame.
    if (solver_flag == SolverState::NON_LINEAR) imu_gap_detected = true;
  }

  if (!pre_integrations[frameCount]) {
    pre_integrations[frameCount] = std::make_shared<IntegrationBase>(
        previousImuData, estimator_state[frameCount].accel_bias,
        estimator_state[frameCount].gyro_bias, options->imu);
  }
  if (frameCount != 0) {
    IMUData delta_imu = data;
    delta_imu.timestamp = deltaTime;

    pre_integrations[frameCount]->push_back(delta_imu);
    tmp_pre_integration->push_back(delta_imu);
    deltaTimeImuBuffer[frameCount].push_back(delta_imu);

    updateStateWithIMU(data, deltaTime);
  }
  previousImuData = data;
}

void Estimator::updateStateWithIMU(const IMUData &data, double deltaTime) {
  int j = frameCount;
  Vector3d un_acc_0 =
      estimator_state[j].rotation * (previousImuData.linear_acceleration -
                                     estimator_state[j].accel_bias) -
      gravity;
  Vector3d un_gyr =
      0.5 * (previousImuData.angular_velocity + data.angular_velocity) -
      estimator_state[j].gyro_bias;
  estimator_state[j].rotation *=
      Utility::deltaQ(un_gyr * deltaTime).toRotationMatrix();
  Vector3d un_acc_1 =
      estimator_state[j].rotation *
          (data.linear_acceleration - estimator_state[j].accel_bias) -
      gravity;
  Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  estimator_state[j].position += deltaTime * estimator_state[j].velocity +
                                 0.5 * deltaTime * deltaTime * un_acc;
  estimator_state[j].velocity += deltaTime * un_acc;
}

void Estimator::processImage(const FeatureFrame &features,
                             Timestamp timestamp) {
  const auto visual_update_start = std::chrono::steady_clock::now();
  setMarginalizationFlag(features);
  insertImageFrame(features, timestamp);
  handleExtrinsicInitialization();
  frame_metrics_.visual_update_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - visual_update_start).count();
  if (!isNonLinearSolver()) {
    const auto initialization_start = std::chrono::steady_clock::now();
    processInitialization(timestamp);
    frame_metrics_.initialization_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - initialization_start).count();
  } else {
    processNonLinearSolver(timestamp);
  }
}

void Estimator::setMarginalizationFlag(const FeatureFrame &features) {
  if (featureManager.addFeatureCheckParallax(frameCount, features,
                                             options->time_delay)) {
    marginalization_flag = MarginalizationType::MARGIN_OLD;
  } else {
    marginalization_flag = MarginalizationType::MARGIN_SECOND_NEW;
  }
}
void Estimator::insertImageFrame(const FeatureFrame &features,
                                 Timestamp timestamp) {
  estimator_state[frameCount].timestamp = timestamp;
  ImageFrame imageframe(features, timestamp);
  imageframe.pre_integration = std::move(tmp_pre_integration);
  all_image_frame.insert(make_pair(timestamp, imageframe));
  tmp_pre_integration = std::make_shared<IntegrationBase>(
      previousImuData, estimator_state[frameCount].accel_bias,
      estimator_state[frameCount].gyro_bias, options->imu);
}

void Estimator::handleExtrinsicInitialization() {
  if (!options->isInitializingExtrinsic() || frameCount == 0) {
    return;
  }
  vector<pair<Vector3d, Vector3d>> corres =
      featureManager.getCorresponding(frameCount - 1, frameCount);
  Matrix3d calib_ric;
  if (initial_ex_rotation.CalibrationExRotation(
          corres, pre_integrations[frameCount]->delta_q, calib_ric)) {
    cameraRotation[0] = calib_ric;
    options->RIC[0] = calib_ric;
    options->extrinsic_estimation_mode = ExtrinsicEstimationMode::APPROXIMATE;
  }
}

void Estimator::processInitialization(Timestamp timestamp) {
  if (options->isMonoWithImu()) {
    processMonoWithImuInitialization(timestamp);
  }
  if (options->isStereoWithImu()) {
    processStereoWithImuInitialization();
  }
  if (options->isStereoWithoutImu()) {
    processStereoWithoutImuInitialization();
  }

  if (frameCount < WINDOW_SIZE) {
    frameCount++;
    estimator_state[frameCount] = estimator_state[frameCount - 1];
  }
}
void Estimator::processNonLinearSolver(Timestamp timestamp) {
  // A large IMU gap (dropped messages) was detected while tracking: re-initialize cleanly
  // instead of running the solver on an inconsistent window (which evaluates to NaN).
  if (imu_gap_detected) {
    VINS_WARN << "Re-initializing after IMU gap (dropped messages).";
    imu_gap_detected = false;
    failure_occur = 1;
    ++imu_gap_resets_;
    logEvent(timestamp, "reset", "imu_gap");
    resetState();
    initializeCamerasFromOptions();
    return;
  }
  const auto triangulation_start = std::chrono::steady_clock::now();
  if (!options->hasImu()) {
    featureManager.initFramePoseByPnP(frameCount, estimator_state,
                                      cameraTranslation, cameraRotation);
  }
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);
  frame_metrics_.triangulation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - triangulation_start).count();

  optimize();
  set<int> removeIndex;
  const auto outlier_start = std::chrono::steady_clock::now();
  outliersRejection(removeIndex);
  featureManager.removeOutlier(removeIndex);
  frame_metrics_.outlier_rejection_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - outlier_start).count();
  frame_metrics_.outliers_removed += removeIndex.size();
  total_outliers_removed_.fetch_add(removeIndex.size());
  const auto failure_detection_start = std::chrono::steady_clock::now();
  const bool failure_detected = failureDetection();
  frame_metrics_.failure_detection_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - failure_detection_start).count();
  if (failure_detected) {
    failure_occur = 1;
    ++failure_resets_;
    logEvent(timestamp, "reset", "failure_detection");
    resetState();
    initializeCamerasFromOptions();
    return;
  }

  slideWindow();
  const auto state_output_start = std::chrono::steady_clock::now();
  featureManager.removeFailures();
  // prepare output of VINS
  {
    key_poses.timestamp = timestamp;
    key_poses.poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++)
      key_poses.poses.push_back(estimator_state[i].position);

    safe_key_poses.set(key_poses);
  }

  last_state = estimator_state[WINDOW_SIZE];
  last_state0 = estimator_state[0];
  updateLatestStates();

  vio_odom.timestamp = timestamp;
  vio_odom.position = estimator_state[WINDOW_SIZE].position;
  vio_odom.orientation = Quaterniond(estimator_state[WINDOW_SIZE].rotation);
  vio_odom.velocity = estimator_state[WINDOW_SIZE].velocity;
  safe_vio_odom.set(vio_odom);

  // Append the latest VIO state to the result CSV. This port only truncates the file at
  // startup (parameters.h) and otherwise never wrote it, so vio.csv was always empty.
  // Format matches VINS-Fusion: t_ns, px,py,pz, qw,qx,qy,qz, vx,vy,vz.
  if (!options->VINS_RESULT_PATH.empty()) {
    std::ofstream foutC(options->VINS_RESULT_PATH, std::ios::app);
    if (foutC.is_open()) {
      foutC.setf(std::ios::fixed, std::ios::floatfield);
      foutC.precision(0);
      foutC << timestamp * 1e9 << ",";
      foutC.precision(5);
      foutC << vio_odom.position.x() << "," << vio_odom.position.y() << ","
            << vio_odom.position.z() << "," << vio_odom.orientation.w() << ","
            << vio_odom.orientation.x() << "," << vio_odom.orientation.y() << ","
            << vio_odom.orientation.z() << "," << vio_odom.velocity.x() << ","
            << vio_odom.velocity.y() << "," << vio_odom.velocity.z() << std::endl;
      ++total_poses_written_;
    }
  }
  frame_metrics_.state_output_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - state_output_start).count();
}

void Estimator::processMonoWithImuInitialization(Timestamp timestamp) {
  if (frameCount != WINDOW_SIZE) return;

  bool result = false;
  if (!options->isInitializingExtrinsic() &&
      (timestamp - initialTimestamp) > 0.1) {
    result = initialStructure();
    initialTimestamp = timestamp;
  }
  if (result) {
    optimize();
    updateLatestStates();
    solver_flag = SolverState::NON_LINEAR;
    slideWindow();
    VINS_INFO << "Initialization complete. Switching to NON_LINEAR mode.";
  } else {
    slideWindow();
  }
}

void Estimator::processStereoWithImuInitialization() {
  const auto triangulation_start = std::chrono::steady_clock::now();
  featureManager.initFramePoseByPnP(frameCount, estimator_state,
                                    cameraTranslation, cameraRotation);
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);
  frame_metrics_.triangulation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - triangulation_start).count();

  if (frameCount != WINDOW_SIZE) return;

  map<double, ImageFrame>::iterator frame_it;
  int i = 0;
  for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end();
       frame_it++) {
    frame_it->second.R = estimator_state[i].rotation;
    frame_it->second.T = estimator_state[i].position;
    i++;
  }
  solveGyroscopeBias(all_image_frame, estimator_state);
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    pre_integrations[i]->repropagate(Vector3d::Zero(),
                                     estimator_state[i].gyro_bias);
  }
  optimize();
  updateLatestStates();
  solver_flag = SolverState::NON_LINEAR;
  slideWindow();
  VINS_INFO << "Initialization complete. Switching to NON_LINEAR mode.";
}

void Estimator::processStereoWithoutImuInitialization() {
  const auto triangulation_start = std::chrono::steady_clock::now();
  featureManager.initFramePoseByPnP(frameCount, estimator_state,
                                    cameraTranslation, cameraRotation);
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);
  frame_metrics_.triangulation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - triangulation_start).count();
  optimize();

  if (frameCount != WINDOW_SIZE) return;

  optimize();
  updateLatestStates();
  solver_flag = SolverState::NON_LINEAR;
  slideWindow();
  VINS_INFO << "Initialization complete. Switching to NON_LINEAR mode.";
}

bool Estimator::isNonLinearSolver() const {
  return solver_flag == SolverState::NON_LINEAR;
}

bool Estimator::isNewMarginalization() const {
  return marginalization_flag == MarginalizationType::MARGIN_SECOND_NEW;
}

bool Estimator::checkIMUExcitation() {
  Vector3d sumG = Vector3d::Zero();
  auto it = std::next(all_image_frame.begin());
  for (; it != all_image_frame.end(); ++it) {
    double dt = it->second.pre_integration->sum_dt;
    sumG += it->second.pre_integration->delta_v / dt;
  }

  Vector3d avgG = sumG / (all_image_frame.size() - 1);
  double variance = 0;
  for (it = std::next(all_image_frame.begin()); it != all_image_frame.end();
       ++it) {
    double dt = it->second.pre_integration->sum_dt;
    Vector3d tmpG = it->second.pre_integration->delta_v / dt;
    variance += (tmpG - avgG).squaredNorm();
  }

  variance = std::sqrt(variance / (all_image_frame.size() - 1));
  return variance >= 0.25;
}

std::vector<SFMFeature> Estimator::buildSFMFeatures() {
  std::vector<SFMFeature> sfm_features;

  for (auto &feature : featureManager.feature) {
    SFMFeature sf;
    sf.state = false;
    sf.id = feature.feature_id;
    int frame_id = feature.start_frame - 1;
    for (auto &f : feature.feature_per_frame) {
      ++frame_id;
      sf.observation.emplace_back(frame_id, Vector2d(f.point.x(), f.point.y()));
    }
    sfm_features.emplace_back(std::move(sf));
  }

  return sfm_features;
}

bool Estimator::solvePoseWithPnP(const std::vector<Quaterniond> &rotations,
                                 const std::vector<Vector3d> &positions,
                                 const std::map<int, Vector3d> &sfmPoints) {
  auto frame_it = all_image_frame.begin();
  int i = 0;

  while (frame_it != all_image_frame.end()) {
    if (frame_it->first == estimator_state[i].timestamp) {
      frame_it->second.is_key_frame = true;
      frame_it->second.R =
          rotations[i].toRotationMatrix() * options->RIC[0].transpose();
      frame_it->second.T = positions[i];
      ++i;
      ++frame_it;
      continue;
    }

    if (frame_it->first > estimator_state[i].timestamp) {
      ++i;
    }
    cv::Mat tmp_r;
    cv::Mat r, rvec, tvec, D;
    Matrix3d R_inital = (rotations[i].inverse()).toRotationMatrix();
    Vector3d P_inital = -R_inital * positions[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, tvec);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts3;
    vector<cv::Point2f> pts2;

    for (const auto &pt : frame_it->second.points) {
      auto it = sfmPoints.find(pt.first);
      if (it == sfmPoints.end()) continue;

      for (const auto &obs : pt.second) {
        Vector3d p3d = it->second;
        Vector2d p2d = obs.second.head<2>();
        pts3.emplace_back(p3d.x(), p3d.y(), p3d.z());
        pts2.emplace_back(p2d.x(), p2d.y());
      }
    }

    if (pts3.size() < 6) {
      VINS_WARN << "Not enough points for solvePnP: " << pts3.size();
      return false;
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    if (!cv::solvePnP(pts3, pts2, K, D, rvec, tvec, true)) {
      VINS_WARN << "Failed to solvePnP";
      return false;
    }

    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    MatrixXd T_pnp;
    cv::cv2eigen(tvec, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R = R_pnp * options->RIC[0].transpose();
    frame_it->second.T = T_pnp;
    ++frame_it;
  }

  return true;
}

bool Estimator::initialStructure() {
  if (!checkIMUExcitation()) {
    VINS_WARN << "IMU excitation insufficient";
  }

  auto sfm_features = buildSFMFeatures();

  Matrix3d relativeRotation;
  Vector3d relativeTranslation;
  int referenceFrame;
  if (!computeRelativePose(relativeRotation, relativeTranslation,
                           referenceFrame)) {
    VINS_WARN << "Not enough features or parallax; Move device around!!!";
    return false;
  }

  std::vector<Quaterniond> rotations(WINDOW_SIZE + 1);
  std::vector<Vector3d> positions(WINDOW_SIZE + 1);
  std::map<int, Vector3d> sfmPoints;
  GlobalSFM sfm;
  if (!sfm.construct(frameCount + 1, rotations, positions, referenceFrame,
                     relativeRotation, relativeTranslation, sfm_features,
                     sfmPoints)) {
    marginalization_flag = MarginalizationType::MARGIN_OLD;
    VINS_WARN << "Failed to construect sfm!!!";
    return false;
  }

  if (!solvePoseWithPnP(rotations, positions, sfmPoints)) {
    return false;
  }

  return visualInitialAlign();
}

bool Estimator::visualInitialAlign() {
  TicToc t_g;
  VectorXd x;
  // solve scale
  bool result = VisualIMUAlignment(all_image_frame, estimator_state, gravity, x,
                                   *options);
  if (!result) {
    VINS_DEBUG << "misalign visual structure with IMU";
    return false;
  }

  // change state
  for (int i = 0; i <= frameCount; i++) {
    Matrix3d Ri = all_image_frame[estimator_state[i].timestamp].R;
    Vector3d Pi = all_image_frame[estimator_state[i].timestamp].T;
    estimator_state[i].position = Pi;
    estimator_state[i].rotation = Ri;
    all_image_frame[estimator_state[i].timestamp].is_key_frame = true;
  }

  double s = (x.tail<1>())(0);
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    pre_integrations[i]->repropagate(Vector3d::Zero(),
                                     estimator_state[i].gyro_bias);
  }
  for (int i = frameCount; i >= 0; i--)
    estimator_state[i].position =
        s * estimator_state[i].position -
        estimator_state[i].rotation * options->TIC[0] -
        (s * estimator_state[0].position -
         estimator_state[0].rotation * options->TIC[0]);
  int kv = -1;
  map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end();
       frame_i++) {
    if (frame_i->second.is_key_frame) {
      kv++;
      estimator_state[kv].velocity = frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  Matrix3d R0 = Utility::g2R(gravity);
  double yaw = Utility::R2ypr(R0 * estimator_state[0].rotation).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  gravity = R0 * gravity;
  Matrix3d rot_diff = R0;
  for (int i = 0; i <= frameCount; i++) {
    estimator_state[i].position = rot_diff * estimator_state[i].position;
    estimator_state[i].rotation = rot_diff * estimator_state[i].rotation;
    estimator_state[i].velocity = rot_diff * estimator_state[i].velocity;
  }
  featureManager.clearDepth();
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);

  return true;
}

bool Estimator::computeRelativePose(Matrix3d &relative_R, Vector3d &relative_T,
                                    int &referenceFrame) {
  for (int i = 0; i < WINDOW_SIZE; i++) {
    auto correspondences = featureManager.getCorresponding(i, WINDOW_SIZE);
    if (correspondences.size() > 20) {
      double totalParallax = 0;
      for (const auto &corr : correspondences) {
        Vector2d pts0(corr.first(0), corr.first(1));
        Vector2d pts1(corr.second(0), corr.second(1));
        totalParallax += (pts0 - pts1).norm();
      }

      double avgParallax = totalParallax / correspondences.size();
      if (avgParallax * 460 > 30 &&
          m_estimator.solveRelativeRT(correspondences, relative_R,
                                      relative_T)) {
        referenceFrame = i;
        return true;
      }
    }
  }
  return false;
}

void Estimator::prepareParameters() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    estimator_state[i].toPoseArray(poseArray[i]);
    estimator_state[i].toSpeedBiasArray(speedBiasArray[i]);
  }

  for (int i = 0; i < options->getNumCameras(); i++) {
    para_Ex_Pose[i][0] = cameraTranslation[i].x();
    para_Ex_Pose[i][1] = cameraTranslation[i].y();
    para_Ex_Pose[i][2] = cameraTranslation[i].z();
    Quaterniond q{cameraRotation[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  VectorXd dep = featureManager.getDepthVector();
  for (int i = 0; i < featureManager.getFeatureCount(); i++)
    para_Feature[i][0] = dep(i);

  para_Td[0][0] = options->time_delay;
}

void Estimator::updateEstimates() {
  Vector3d origin_R0 = Utility::R2ypr(estimator_state[0].rotation);
  Vector3d origin_P0 = estimator_state[0].position;

  if (failure_occur) {
    origin_R0 = Utility::R2ypr(last_state0.rotation);
    origin_P0 = last_state0.position;
    failure_occur = 0;
  }

  if (options->hasImu()) {
    const auto &pose0Vector = Utility::toPositionVector(poseArray[0]);
    const auto &pose0Matrix =
        Utility::toQuaternion(poseArray[0]).toRotationMatrix();

    Vector3d origin_R00 = Utility::R2ypr(pose0Matrix);
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
        abs(abs(origin_R00.y()) - 90) < 1.0) {
      rot_diff = estimator_state[0].rotation * pose0Matrix.transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {
      estimator_state[i].rotation =
          rot_diff * Utility::toQuaternion(poseArray[i]).toRotationMatrix();

      estimator_state[i].position =
          rot_diff * (Utility::toPositionVector(poseArray[i]) - pose0Vector) +
          origin_P0;
      estimator_state[i].velocity =
          rot_diff * Utility::toPositionVector(speedBiasArray[i]);
      estimator_state[i].accel_bias =
          Utility::toPositionVector(speedBiasArray[i] + 3);
      estimator_state[i].gyro_bias =
          Utility::toPositionVector(speedBiasArray[i] + 6);
    }
  } else {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
      estimator_state[i].rotation =
          Utility::toQuaternion(poseArray[i]).toRotationMatrix();
      estimator_state[i].position = Utility::toPositionVector(poseArray[i]);
    }
  }

  if (options->hasImu()) {
    for (int i = 0; i < options->getNumCameras(); i++) {
      cameraTranslation[i] =
          Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
      cameraRotation[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                                      para_Ex_Pose[i][4], para_Ex_Pose[i][5])
                              .normalized()
                              .toRotationMatrix();
      updateCameraPose(i);
    }
  }

  VectorXd dep = featureManager.getDepthVector();
  for (int i = 0; i < featureManager.getFeatureCount(); i++)
    dep(i) = para_Feature[i][0];
  featureManager.setDepth(dep);

  if (options->hasImu()) options->time_delay = para_Td[0][0];
}

bool Estimator::failureDetection() {
  return false;
  if (featureManager.last_track_num < 2) {
  }
  if (estimator_state[WINDOW_SIZE].accel_bias.norm() > 2.5) {
    return true;
  }
  if (estimator_state[WINDOW_SIZE].gyro_bias.norm() > 1.0) {
    return true;
  }
  Vector3d tmp_P = estimator_state[WINDOW_SIZE].position;
  if ((tmp_P - last_state.position).norm() > 5) {
    // return true;
  }
  if (abs(tmp_P.z() - last_state.position.z()) > 1) {
    // return true;
  }
  Matrix3d tmp_R = estimator_state[WINDOW_SIZE].rotation;
  Matrix3d delta_R = tmp_R.transpose() * last_state.rotation;
  Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50) {
    // return true;
  }
  return false;
}

void Estimator::AddPoseParameterBlocks(ceres::Problem &problem) {
  for (int i = 0; i < frameCount + 1; i++) {
    auto *local_param = new PoseLocalParameterization();
    problem.AddParameterBlock(poseArray[i], SIZE_POSE, local_param);
    if (options->hasImu()) {
      problem.AddParameterBlock(speedBiasArray[i], SIZE_SPEEDBIAS);
    }
  }
  if (!options->hasImu()) {
    problem.SetParameterBlockConstant(poseArray[0]);
  }
}
void Estimator::AddExtrinsicParameterBlocks(ceres::Problem &problem) {
  for (int i = 0; i < options->getNumCameras(); i++) {
    auto *local_param = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_param);
    if (((options->isExtrinsicEstimationApproximate() ||
          options->isInitializingExtrinsic()) &&
         frameCount == WINDOW_SIZE &&
         estimator_state[0].velocity.norm() > 0.2) ||
        openExEstimation) {
      openExEstimation = 1;
    } else {
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
  }
}
void Estimator::AddTimeDelayParameterBlock(ceres::Problem &problem) {
  problem.AddParameterBlock(para_Td[0], 1);
  if (!options->shouldEstimateTD() || estimator_state[0].velocity.norm() < 0.2)
    problem.SetParameterBlockConstant(para_Td[0]);
}
void Estimator::AddMarginalizationFactor(ceres::Problem &problem) {
  if (last_marginalization_info && last_marginalization_info->valid) {
    auto *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, nullptr,
                             last_marginalization_parameter_blocks);
  }
}
void Estimator::AddIMUFactors(ceres::Problem &problem) {
  if (!options->hasImu()) return;

  for (int i = 0; i < frameCount; i++) {
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0) continue;
    auto *imu_factor = new IMUFactor(pre_integrations[j]);
    problem.AddResidualBlock(imu_factor, nullptr, poseArray[i],
                             speedBiasArray[i], poseArray[j],
                             speedBiasArray[j]);
  }
}
void Estimator::AddFeatureFactors(ceres::Problem &problem) {
  int feature_index = -1;
  for (auto &it : featureManager.feature) {
    if (it.feature_per_frame.size() < 4) continue;
    ++feature_index;

    int imu_i = it.start_frame, imu_j = imu_i - 1;
    Vector3d pts_i = it.feature_per_frame[0].point;

    for (auto &f : it.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = f.point;
        auto *factor = new ProjectionTwoFrameOneCamFactor(
            pts_i, pts_j, it.feature_per_frame[0].velocity, f.velocity,
            it.feature_per_frame[0].cur_td, f.cur_td);
        problem.AddResidualBlock(factor, new ceres::HuberLoss(1.0),
                                 poseArray[imu_i], poseArray[imu_j],
                                 para_Ex_Pose[0], para_Feature[feature_index],
                                 para_Td[0]);
      }

      if (options->isUsingStereo() && f.is_stereo) {
        Vector3d pts_j_right = f.pointRight;
        if (imu_i != imu_j) {
          auto *factor = new ProjectionTwoFrameTwoCamFactor(
              pts_i, pts_j_right, it.feature_per_frame[0].velocity,
              f.velocityRight, it.feature_per_frame[0].cur_td, f.cur_td);
          problem.AddResidualBlock(factor, new ceres::HuberLoss(1.0),
                                   poseArray[imu_i], poseArray[imu_j],
                                   para_Ex_Pose[0], para_Ex_Pose[1],
                                   para_Feature[feature_index], para_Td[0]);
        } else {
          auto *factor = new ProjectionOneFrameTwoCamFactor(
              pts_i, pts_j_right, it.feature_per_frame[0].velocity,
              f.velocityRight, it.feature_per_frame[0].cur_td, f.cur_td);
          problem.AddResidualBlock(factor, new ceres::HuberLoss(1.0),
                                   para_Ex_Pose[0], para_Ex_Pose[1],
                                   para_Feature[feature_index], para_Td[0]);
        }
      }
    }
  }
}
void Estimator::solveOptimization() {
  const auto problem_construction_start = std::chrono::steady_clock::now();
  ceres::Problem problem;

  AddPoseParameterBlocks(problem);
  AddExtrinsicParameterBlocks(problem);
  AddTimeDelayParameterBlock(problem);
  AddMarginalizationFactor(problem);
  AddIMUFactors(problem);
  AddFeatureFactors(problem);
  frame_metrics_.problem_construction_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - problem_construction_start).count();
  frame_metrics_.solver_parameter_blocks = std::max(
      frame_metrics_.solver_parameter_blocks, problem.NumParameterBlocks());
  frame_metrics_.solver_residual_blocks = std::max(
      frame_metrics_.solver_residual_blocks, problem.NumResidualBlocks());
  frame_metrics_.solver_residuals = std::max(
      frame_metrics_.solver_residuals, problem.NumResiduals());

  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type =
      options->USE_GPU_CERES ? ceres::DENSE_QR : ceres::DENSE_SCHUR;
  ceres_options.trust_region_strategy_type = ceres::DOGLEG;
  ceres_options.max_num_iterations = options->max_num_iterations();
  ceres_options.max_solver_time_in_seconds =
      (!isNewMarginalization()) ? options->max_solver_time() * 0.8
                                : options->max_solver_time();

  ceres::Solver::Summary summary;
  const auto solver_start = std::chrono::steady_clock::now();
  ceres::Solve(ceres_options, &problem, &summary);
  frame_metrics_.solver_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solver_start).count();
  if (frame_metrics_.solver_calls == 0) {
    frame_metrics_.solver_initial_cost = summary.initial_cost;
  }
  ++frame_metrics_.solver_calls;
  ++total_solver_calls_;
  frame_metrics_.solver_iterations += static_cast<int>(summary.iterations.size());
  frame_metrics_.solver_termination_type = static_cast<int>(summary.termination_type);
  frame_metrics_.solver_solution_usable = summary.IsSolutionUsable();
  frame_metrics_.solver_final_cost = summary.final_cost;
}

void Estimator::processOldMarginalization() {
  auto marginalization_info = std::make_shared<MarginalizationInfo>();
  prepareParameters();

  if (last_marginalization_info && last_marginalization_info->valid) {
    vector<int> drop_set;
    for (int i = 0;
         i < static_cast<int>(last_marginalization_parameter_blocks.size());
         i++) {
      if (last_marginalization_parameter_blocks[i] == poseArray[0] ||
          last_marginalization_parameter_blocks[i] == speedBiasArray[0]) {
        drop_set.push_back(i);
      }
    }

    auto marginalization_factor =
        std::make_shared<MarginalizationFactor>(last_marginalization_info);
    auto residual_block_info = std::make_shared<ResidualBlockInfo>(
        marginalization_factor, nullptr, last_marginalization_parameter_blocks,
        drop_set);
    marginalization_info->addResidualBlockInfo(residual_block_info);
  }

  // IMU
  if (options->hasImu() && pre_integrations[1]->sum_dt < 10.0) {
    marginalization_info->addResidualBlockInfo(createIMUResidualBlock());
  }

  // Features
  addFeatureResidualBlocks(marginalization_info);

  marginalization_info->preMarginalize();
  marginalization_info->marginalize();

  auto addr_shift = createAddrShift(true);
  auto parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
  last_marginalization_info = std::move(marginalization_info);
  last_marginalization_parameter_blocks = parameter_blocks;
}
void Estimator::processNewMarginalization() {
  if (!last_marginalization_info) return;
  if (std::count(last_marginalization_parameter_blocks.begin(),
                 last_marginalization_parameter_blocks.end(),
                 poseArray[WINDOW_SIZE - 1])) {
    auto marginalization_info = std::make_shared<MarginalizationInfo>();
    prepareParameters();

    if (last_marginalization_info->valid) {
      vector<int> drop_set;
      for (int i = 0;
           i < static_cast<int>(last_marginalization_parameter_blocks.size());
           i++) {
        if (last_marginalization_parameter_blocks[i] ==
            poseArray[WINDOW_SIZE - 1]) {
          drop_set.push_back(i);
        }
      }

      auto marginalization_factor =
          std::make_shared<MarginalizationFactor>(last_marginalization_info);
      auto residual_block_info = std::make_shared<ResidualBlockInfo>(
          marginalization_factor, nullptr,
          last_marginalization_parameter_blocks, drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    marginalization_info->preMarginalize();
    marginalization_info->marginalize();

    auto addr_shift = createAddrShift(false);
    auto parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift);
    last_marginalization_info = std::move(marginalization_info);
    last_marginalization_parameter_blocks = parameter_blocks;
  }
}
std::shared_ptr<ResidualBlockInfo> Estimator::createIMUResidualBlock() {
  auto imu_factor = std::make_shared<IMUFactor>(pre_integrations[1]);
  return std::make_shared<ResidualBlockInfo>(
      imu_factor, nullptr,
      std::vector<double *>{poseArray[0], speedBiasArray[0], poseArray[1],
                            speedBiasArray[1]},
      std::vector<int>{0, 1});
}
void Estimator::addFeatureResidualBlocks(
    std::shared_ptr<MarginalizationInfo> &marg_info) {
  int feature_index = -1;

  for (auto &it_per_id : featureManager.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) continue;

    ++feature_index;

    int imu_i = it_per_id.start_frame;
    if (imu_i != 0) continue;

    int imu_j = imu_i - 1;
    const Vector3d &pts_i = it_per_id.feature_per_frame[0].point;

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      const Vector3d &pts_j = it_per_frame.point;

      if (imu_i != imu_j) {
        auto f_td = std::make_shared<ProjectionTwoFrameOneCamFactor>(
            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
            it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
            it_per_frame.cur_td);
        auto loss = std::make_shared<ceres::HuberLoss>(1.0);
        marg_info->addResidualBlockInfo(std::make_shared<ResidualBlockInfo>(
            f_td, loss,
            vector<double *>{poseArray[imu_i], poseArray[imu_j],
                             para_Ex_Pose[0], para_Feature[feature_index],
                             para_Td[0]},
            vector<int>{0, 3}));
      }

      if (options->isUsingStereo() && it_per_frame.is_stereo) {
        const Vector3d &pts_j_right = it_per_frame.pointRight;
        auto loss = std::make_shared<ceres::HuberLoss>(1.0);

        if (imu_i != imu_j) {
          auto f = std::make_shared<ProjectionTwoFrameTwoCamFactor>(
              pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td);
          marg_info->addResidualBlockInfo(std::make_shared<ResidualBlockInfo>(
              f, loss,
              vector<double *>{poseArray[imu_i], poseArray[imu_j],
                               para_Ex_Pose[0], para_Ex_Pose[1],
                               para_Feature[feature_index], para_Td[0]},
              vector<int>{0, 4}));
        } else {
          auto f = std::make_shared<ProjectionOneFrameTwoCamFactor>(
              pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td);
          marg_info->addResidualBlockInfo(std::make_shared<ResidualBlockInfo>(
              f, loss,
              vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1],
                               para_Feature[feature_index], para_Td[0]},
              vector<int>{2}));
        }
      }
    }
  }
}
std::unordered_map<long, double *> Estimator::createAddrShift(bool is_old) {
  std::unordered_map<long, double *> addr_shift;

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    if (!is_old && i == WINDOW_SIZE - 1) continue;

    if (i == WINDOW_SIZE && !is_old) {
      addr_shift[reinterpret_cast<long>(poseArray[i])] = poseArray[i - 1];
      if (options->hasImu())
        addr_shift[reinterpret_cast<long>(speedBiasArray[i])] =
            speedBiasArray[i - 1];
    } else {
      addr_shift[reinterpret_cast<long>(poseArray[i])] =
          poseArray[i - (is_old ? 1 : 0)];
      if (options->hasImu())
        addr_shift[reinterpret_cast<long>(speedBiasArray[i])] =
            speedBiasArray[i - (is_old ? 1 : 0)];
    }
  }

  for (int i = 0; i < options->getNumCameras(); i++)
    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

  addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

  return addr_shift;
}

void Estimator::optimize() {
  const auto parameter_preparation_start = std::chrono::steady_clock::now();
  prepareParameters();
  frame_metrics_.parameter_preparation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - parameter_preparation_start).count();
  solveOptimization();
  const auto estimate_update_start = std::chrono::steady_clock::now();
  updateEstimates();
  frame_metrics_.estimate_update_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - estimate_update_start).count();
  if (frameCount < WINDOW_SIZE) return;
  const auto marginalization_start = std::chrono::steady_clock::now();
  if (isNewMarginalization()) {
    processNewMarginalization();
  } else {
    processOldMarginalization();
  }
  frame_metrics_.marginalization_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - marginalization_start).count();
}

void Estimator::slideWindow() {
  const auto slide_window_start = std::chrono::steady_clock::now();
  if (!isNewMarginalization()) {
    double t_0 = estimator_state[0].timestamp;
    back_state = estimator_state[0];
    slideWindowOld();
  } else {
    slideWindowNew();
  }
  frame_metrics_.slide_window_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - slide_window_start).count();
}

void Estimator::slideWindowNew() {
  if (frameCount != WINDOW_SIZE) {
    return;
  }
  estimator_state[frameCount - 1] = estimator_state[frameCount];
  if (options->hasImu()) {
    for (unsigned int i = 0; i < deltaTimeImuBuffer[frameCount].size(); i++) {
      const auto &imu = deltaTimeImuBuffer[frameCount][i];
      pre_integrations[frameCount - 1]->push_back(imu);
      deltaTimeImuBuffer[frameCount - 1].push_back(imu);
    }
    pre_integrations[WINDOW_SIZE] = nullptr;
    pre_integrations[WINDOW_SIZE] = std::make_shared<IntegrationBase>(
        previousImuData, estimator_state[WINDOW_SIZE].accel_bias,
        estimator_state[WINDOW_SIZE].gyro_bias, options->imu);

    deltaTimeImuBuffer[WINDOW_SIZE].clear();
  }
  frontCount++;
  featureManager.removeFront(frameCount);
}

void Estimator::slideWindowOld() {
  if (frameCount != WINDOW_SIZE) {
    return;
  }
  for (int i = 0; i < WINDOW_SIZE; i++) {
    estimator_state[i].timestamp = estimator_state[i + 1].timestamp;
    estimator_state[i].swap(estimator_state[i + 1]);
    if (options->hasImu()) {
      std::swap(pre_integrations[i], pre_integrations[i + 1]);
      deltaTimeImuBuffer[i].swap(deltaTimeImuBuffer[i + 1]);
    }
  }
  estimator_state[WINDOW_SIZE] = estimator_state[WINDOW_SIZE - 1];
  if (options->hasImu()) {
    pre_integrations[WINDOW_SIZE] = nullptr;
    pre_integrations[WINDOW_SIZE] = std::make_shared<IntegrationBase>(
        previousImuData, estimator_state[WINDOW_SIZE].accel_bias,
        estimator_state[WINDOW_SIZE].gyro_bias, options->imu);
    deltaTimeImuBuffer[WINDOW_SIZE].clear();
  }
  auto it_0 = all_image_frame.find(estimator_state[0].timestamp);
  all_image_frame.erase(all_image_frame.begin(), it_0);

  backCount++;
  if (isNonLinearSolver()) {
    Matrix3d R0, R1;
    Vector3d P0, P1;
    R0 = back_state.rotation * cameraRotation[0];
    R1 = estimator_state[0].rotation * cameraRotation[0];
    P0 = back_state.position + back_state.rotation * cameraTranslation[0];
    P1 = estimator_state[0].position +
         estimator_state[0].rotation * cameraTranslation[0];
    featureManager.removeBackShiftDepth(R0, P0, R1, P1);
  } else {
    featureManager.removeBack();
  }
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = estimator_state[frameCount].rotation;
  T.block<3, 1>(0, 3) = estimator_state[frameCount].position;
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = estimator_state[index].rotation;
  T.block<3, 1>(0, 3) = estimator_state[index].position;
}

bool Estimator::getIntegratedImuOdom(OdomData &data) {
  if (!safe_imu_pre_odom.check()) {
    return false;
  }
  safe_imu_pre_odom.get(data);
  return true;
}
bool Estimator::getVisualInertialOdom(OdomData &data) {
  if (!safe_vio_odom.check()) {
    return false;
  }
  safe_vio_odom.get(data);
  return true;
}

bool Estimator::getKeyPoses(PoseSequenceData &poses) {
  if (!safe_key_poses.check()) {
    return false;
  }
  safe_key_poses.get(poses);
  if (poses.poses.size() < 1) {
    return false;
  }
  return true;
}

bool Estimator::getCameraPose(int index, PoseData &data) {
  safe_camera_pose[index].get(data, true);
  return true;
}

bool Estimator::getTrackImage(ImageData &image) {
  if (!safe_track_image.check()) {
    return false;
  }
  safe_track_image.get(image);
  return true;
}

bool Estimator::getMainCloud(PointCloudData &data) {
  if (!safe_main_cloud.check()) {
    return false;
  }
  safe_main_cloud.get(data);
  return true;
}
bool Estimator::getMarginCloud(PointCloudData &data) {
  if (!safe_margin_cloud.check()) {
    return false;
  }
  safe_margin_cloud.get(data);
  return true;
}
bool Estimator::getkeyframeCloud(PointCloudData &data) {
  if (!safe_point_cloud.check()) {
    return false;
  }
  safe_point_cloud.get(data);
  return true;
}

bool Estimator::getkeyframePose(PoseData &data) {
  if (!safe_keyframe_pose.check()) {
    return false;
  }
  safe_keyframe_pose.get(data);
  return true;
}

void Estimator::predictPtsInNextFrame() {
  if (frameCount < 2) return;
  Eigen::Matrix4d curT, prevT, nextT;
  getPoseInWorldFrame(curT);
  getPoseInWorldFrame(frameCount - 1, prevT);
  nextT = curT * (prevT.inverse() * curT);
  map<int, Eigen::Vector3d> predictPts;

  for (auto &it_per_id : featureManager.feature) {
    if (it_per_id.estimated_depth > 0) {
      int firstIndex = it_per_id.start_frame;
      int lastIndex =
          it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
      if ((int)it_per_id.feature_per_frame.size() >= 2 &&
          lastIndex == frameCount) {
        double depth = it_per_id.estimated_depth;
        Vector3d pts_j =
            cameraRotation[0] * (depth * it_per_id.feature_per_frame[0].point) +
            cameraTranslation[0];
        Vector3d pts_w = estimator_state[firstIndex].rotation * pts_j +
                         estimator_state[firstIndex].position;
        Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() *
                             (pts_w - nextT.block<3, 1>(0, 3));
        Vector3d pts_cam =
            cameraRotation[0].transpose() * (pts_local - cameraTranslation[0]);
        int ptsIndex = it_per_id.feature_id;
        predictPts[ptsIndex] = pts_cam;
      }
    }
  }
  featureTracker.setPrediction(predictPts);
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                                    Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                                    Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi,
                                    Vector3d &uvj) {
  Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
  Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
  Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
  double rx = residual.x();
  double ry = residual.y();
  return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex) {
  int feature_index = -1;
  for (auto &it_per_id : featureManager.feature) {
    double err = 0;
    int errCnt = 0;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) continue;
    feature_index++;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        double tmp_error = reprojectionError(
            estimator_state[imu_i].rotation, estimator_state[imu_i].position,
            cameraRotation[0], cameraTranslation[0],
            estimator_state[imu_j].rotation, estimator_state[imu_j].position,
            cameraRotation[0], cameraTranslation[0], depth, pts_i, pts_j);
        err += tmp_error;
        errCnt++;
      }
      // need to rewrite projecton factor.........
      if (options->isUsingStereo() && it_per_frame.is_stereo) {
        Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          double tmp_error = reprojectionError(
              estimator_state[imu_i].rotation, estimator_state[imu_i].position,
              cameraRotation[0], cameraTranslation[0],
              estimator_state[imu_j].rotation, estimator_state[imu_j].position,
              cameraRotation[1], cameraTranslation[1], depth, pts_i,
              pts_j_right);
          err += tmp_error;
          errCnt++;
        } else {
          double tmp_error = reprojectionError(
              estimator_state[imu_i].rotation, estimator_state[imu_i].position,
              cameraRotation[0], cameraTranslation[0],
              estimator_state[imu_j].rotation, estimator_state[imu_j].position,
              cameraRotation[1], cameraTranslation[1], depth, pts_i,
              pts_j_right);
          err += tmp_error;
          errCnt++;
        }
      }
    }
    double ave_err = err / errCnt;
    if (ave_err * FOCAL_LENGTH > 3) removeIndex.insert(it_per_id.feature_id);
  }
}

void Estimator::fastPredictIMU(const IMUData &data) {
  std::lock_guard<std::mutex> lock(propagateMutex);
  double dt = data.timestamp - latestImuData.timestamp;
  // Clamp (don't early-return): returning without updating latestImuData would leave it
  // stale so every subsequent sample also sees a huge dt, permanently stalling the fast
  // IMU odometry. A bounded dt keeps this propagation finite; latestImuData is updated
  // below regardless so the stream stays in sync.
  if (dt > IntegrationBase::MAX_IMU_DT) {
    VINS_WARN << "Large IMU interval " << dt
              << "s in fast propagation; clamping to " << IntegrationBase::MAX_IMU_DT;
    dt = IntegrationBase::MAX_IMU_DT;
  } else if (dt < 0.0) {
    dt = 0.0;
  }
  Eigen::Vector3d un_acc_0 =
      latest_Q * (latestImuData.linear_acceleration - latest_state.accel_bias) -
      gravity;
  Eigen::Vector3d un_gyr =
      0.5 * (latestImuData.angular_velocity + data.angular_velocity) -
      latest_state.gyro_bias;
  latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
  Eigen::Vector3d un_acc_1 =
      latest_Q * (data.linear_acceleration - latest_state.accel_bias) - gravity;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  latest_state.position = latest_state.position + dt * latest_state.velocity +
                          0.5 * dt * dt * un_acc;
  latest_state.velocity = latest_state.velocity + dt * un_acc;

  latestImuData = data;
  imu_odom.timestamp = data.timestamp;
  imu_odom.position = latest_state.position;
  imu_odom.velocity = latest_state.velocity;
  imu_odom.orientation = latest_Q;
  safe_imu_pre_odom.set(imu_odom);
}

void Estimator::updateLatestStates() {
  latest_state = estimator_state[frameCount];
  latest_Q = latest_state.rotation;
  latestImuData = previousImuData;
  latestImuData.timestamp =
      estimator_state[frameCount].timestamp + options->time_delay;

  queue<IMUData> tmp_imu;
  {
    std::lock_guard<std::mutex> imu_lock(imu_mutex);
    tmp_imu = imuBuffer;
  }
  while (!tmp_imu.empty()) {
    fastPredictIMU(tmp_imu.front());
    tmp_imu.pop();
  }
}
