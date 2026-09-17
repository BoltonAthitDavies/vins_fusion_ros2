/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <vins/common/sensor_data_type.h>
#include <vins/estimator/feature_manager.h>
#include <vins/estimator/parameters.h>
#include <vins/factor/imu_factor.h>
#include <vins/factor/marginalization_factor.h>
#include <vins/factor/pose_local_parameterization.h>
#include <vins/factor/projectionOneFrameTwoCamFactor.h>
#include <vins/factor/projectionTwoFrameOneCamFactor.h>
#include <vins/factor/projectionTwoFrameTwoCamFactor.h>
#include <vins/featureTracker/feature_tracker.h>
#include <vins/initial/initial_alignment.h>
#include <vins/initial/initial_ex_rotation.h>
#include <vins/initial/initial_sfm.h>
#include <vins/initial/solve_5pts.h>
#include <vins/logger/logger.h>
#include <vins/utility/tic_toc.h>
#include <vins/utility/utility.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <limits>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <thread>

using namespace std;

class Estimator {
 public:
  Estimator();
  ~Estimator();
  //
  void initialize(std::shared_ptr<VINSOptions> options_);
  // interface
  void inputIMU(const IMUData &imu);
  void inputFeature(Timestamp timestamp, const FeatureFrame &featureFrame);
  void inputImage(const ImageData &image);

  /// Tracked points dropped so far because they landed on a dynamic region.
  /// featureTracker is private, and this is the only thing the node needs from
  /// it -- proof that the masks are actually removing features, not just
  /// arriving. Zero after a run with filter:=true means the mask never bit.
  std::size_t dynamicDroppedPoints() const {
    return featureTracker.dynamic_dropped;
  }
  //
  void processMeasurements();
  //
  void resetState();
  void setFirstPose(const Eigen::Vector3d &position,
                    const Eigen::Matrix3d &rotation);

  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);

  bool getIntegratedImuOdom(OdomData &data);
  bool getVisualInertialOdom(OdomData &data);
  bool getKeyPoses(PoseSequenceData &poses);
  bool getTrackImage(ImageData &image);

  bool getCameraPose(int index, PoseData &data);

  bool getMainCloud(PointCloudData &data);
  bool getMarginCloud(PointCloudData &data);
  bool getkeyframeCloud(PointCloudData &data);
  bool getkeyframePose(PoseData &data);

 private:
  struct FrameMetrics {
    double feature_queue_wait_ms = 0.0;
    double imu_wait_ms = 0.0;
    double imu_propagation_ms = 0.0;
    double visual_update_ms = 0.0;
    double initialization_ms = 0.0;
    double triangulation_ms = 0.0;
    double parameter_preparation_ms = 0.0;
    double problem_construction_ms = 0.0;
    double solver_ms = 0.0;
    double estimate_update_ms = 0.0;
    double marginalization_ms = 0.0;
    double outlier_rejection_ms = 0.0;
    double failure_detection_ms = 0.0;
    double slide_window_ms = 0.0;
    double state_output_ms = 0.0;
    double estimator_update_ms = 0.0;
    double pointcloud_ms = 0.0;
    double backend_total_ms = 0.0;
    int solver_calls = 0;
    int solver_iterations = 0;
    int solver_parameter_blocks = 0;
    int solver_residual_blocks = 0;
    int solver_residuals = 0;
    int solver_termination_type = -1;
    bool solver_solution_usable = false;
    double solver_initial_cost = std::numeric_limits<double>::quiet_NaN();
    double solver_final_cost = std::numeric_limits<double>::quiet_NaN();
    std::size_t outliers_removed = 0;
  };

  void initializeCamerasFromOptions();
  void processIMU(const IMUData &data, double deltaTime);
  void updateStateWithIMU(const IMUData &data, double deltaTime);
  void processImage(const FeatureFrame &features, Timestamp timestamp);
  void fastPredictIMU(const IMUData &data);

  bool isNonLinearSolver() const;
  bool isNewMarginalization() const;
  void setMarginalizationFlag(const FeatureFrame &features);
  void insertImageFrame(const FeatureFrame &features, Timestamp timestamp);
  void handleExtrinsicInitialization();
  void processInitialization(Timestamp timestamp);
  void processNonLinearSolver(Timestamp timestamp);
  void processMonoWithImuInitialization(Timestamp timestamp);
  void processStereoWithImuInitialization();
  void processStereoWithoutImuInitialization();

  // internal
  bool initialStructure();
  bool checkIMUExcitation();
  std::vector<SFMFeature> buildSFMFeatures();
  bool solvePoseWithPnP(const std::vector<Quaterniond> &rotations,
                        const std::vector<Vector3d> &positions,
                        const std::map<int, Vector3d> &sfmPoints);
  bool visualInitialAlign();
  bool computeRelativePose(Matrix3d &relative_R, Vector3d &relative_T,
                           int &referenceFrame);
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  void optimize();
  void prepareParameters();
  void updateEstimates();
  bool failureDetection();
  bool getIMUInterval(double t0, double t1, vector<IMUData> &data);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                           Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                           Matrix3d &ricj, Vector3d &ticj, double depth,
                           Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates();
  bool IMUAvailable(double t);
  void initFirstIMUPose(const vector<IMUData> &data);

 private:
  void AddPoseParameterBlocks(ceres::Problem &problem);
  void AddExtrinsicParameterBlocks(ceres::Problem &problem);
  void AddTimeDelayParameterBlock(ceres::Problem &problem);
  void AddMarginalizationFactor(ceres::Problem &problem);
  void AddIMUFactors(ceres::Problem &problem);
  void AddFeatureFactors(ceres::Problem &problem);
  void solveOptimization();

  void processOldMarginalization();
  void processNewMarginalization();
  std::shared_ptr<ResidualBlockInfo> createIMUResidualBlock();
  void addFeatureResidualBlocks(
      std::shared_ptr<MarginalizationInfo> &marg_info);
  std::unordered_map<long, double *> createAddrShift(bool is_old);

  void updateCameraPose(int index);
  void collectPointCloudAll(Timestamp timestamp);
  void printStatistics(Timestamp timestamp);
  void logEvent(Timestamp timestamp, const std::string &event,
                const std::string &detail = "");
  void writeRunSummary();
  template <typename Container>
  void clearBuffer(Container &container) {
    container.clear();
  }
  template <typename T>
  void clearBuffer(std::queue<T> &queue) {
    std::queue<T> empty;
    std::swap(queue, empty);
  }

  std::thread processThread;
  std::mutex processingMutex;
  std::mutex featureBufferMutex;
  std::condition_variable featureCondition;
  std::mutex propagateMutex;
  std::mutex imu_mutex;
  std::condition_variable imuCondition;
  //
  std::queue<IMUData> imuBuffer;
  queue<TimestampedFeatureFrame> featureBuffer;
  queue<std::chrono::steady_clock::time_point> featureEnqueueTimes;
  queue<bool> featureFromExternalSource;

  Timestamp previousTimestamp = -1;
  Timestamp currentTimestamp = 0;
  Timestamp initialTimestamp = 0;

  bool openExEstimation;
  SolverState solver_flag;
  MarginalizationType marginalization_flag;
  Vector3d gravity;
  Matrix3d cameraRotation[2];
  Vector3d cameraTranslation[2];
  StateData estimator_state[WINDOW_SIZE + 1];
  StateData back_state, last_state, last_state0;
  StateData latest_state;
  Eigen::Quaterniond latest_Q;

  IntegrationBase::Ptr pre_integrations[(WINDOW_SIZE + 1)];
  IMUData previousImuData;
  std::vector<IMUData> deltaTimeImuBuffer[(WINDOW_SIZE + 1)];

  // 计数变量
  int inputImageCount = 0;
  int frameCount = 0;
  int backCount = 0;
  int frontCount = 0;

  // Cumulative scientific run accounting. These deliberately do not reset when
  // the estimator reinitializes, so failed/recovered portions remain visible.
  std::atomic<std::size_t> total_images_received_{0};
  std::atomic<std::size_t> total_images_enqueued_{0};
  std::atomic<std::size_t> total_image_frames_enqueued_{0};
  std::atomic<std::size_t> total_external_feature_frames_{0};
  std::atomic<std::size_t> total_images_processed_{0};
  std::atomic<std::size_t> total_imu_received_{0};
  std::atomic<std::size_t> total_imu_used_{0};
  std::atomic<std::size_t> total_poses_written_{0};
  std::atomic<std::size_t> total_keyframes_{0};
  std::atomic<std::size_t> total_resets_{0};
  std::atomic<std::size_t> imu_gap_resets_{0};
  std::atomic<std::size_t> failure_resets_{0};
  std::atomic<std::size_t> initialization_events_{0};
  std::atomic<std::size_t> frontend_rows_{0};
  std::atomic<std::size_t> backend_rows_{0};
  std::atomic<std::size_t> total_solver_calls_{0};
  std::atomic<std::size_t> total_outliers_removed_{0};
  Timestamp first_processed_timestamp_{-1.0};
  Timestamp first_initialized_timestamp_{-1.0};
  Timestamp last_processed_timestamp_{-1.0};
  std::ofstream performance_log_;
  std::ofstream events_log_;
  std::ofstream frontend_log_;
  std::ofstream backend_log_;
  std::ofstream state_log_;
  std::mutex frontend_log_mutex_;
  FrameMetrics frame_metrics_;
  const std::chrono::steady_clock::time_point process_start_{
      std::chrono::steady_clock::now()};

  FeatureManager featureManager;
  FeatureTracker featureTracker;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool isFirstIMUReceived;
  bool failure_occur;
  // Set when an abnormally large IMU interval (dropped messages) is seen while already
  // tracking. Triggers a clean re-initialization before the next solve, instead of
  // feeding a geometrically-inconsistent IMU factor into Ceres (which evaluates to NaN).
  bool imu_gap_detected = false;

  //
  SafeClass<PointCloudData> safe_main_cloud;
  SafeClass<PointCloudData> safe_point_cloud;
  SafeClass<PointCloudData> safe_margin_cloud;
  SafeClass<PoseData> safe_camera_pose[2];
  SafeClass<PoseData> safe_keyframe_pose;
  SafeClass<PoseSequenceData> safe_key_poses;
  PoseSequenceData key_poses;
  ImageData track_image;
  SafeClass<ImageData> safe_track_image;
  SafeClass<OdomData> safe_imu_pre_odom;
  OdomData imu_odom;
  SafeClass<OdomData> safe_vio_odom;
  OdomData vio_odom;

  double poseArray[WINDOW_SIZE + 1][SIZE_POSE];
  double speedBiasArray[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  MarginalizationInfo::Ptr last_marginalization_info;
  vector<double *> last_marginalization_parameter_blocks;

  map<double, ImageFrame> all_image_frame;
  IntegrationBase::Ptr tmp_pre_integration;
  IMUData latestImuData;
  bool isFirstPoseInitialized = false;
  std::atomic<bool> isRunning{false};
  std::shared_ptr<VINSOptions> options;
};
