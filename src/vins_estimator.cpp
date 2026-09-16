#include <cv_bridge/cv_bridge.h>
#include <vins_fusion_ros2/vins_estimator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <stdexcept>

namespace {
std::string csvEscape(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
  std::string escaped = "\"";
  for (const char c : value) {
    escaped += c;
    if (c == '"') escaped += '"';
  }
  return escaped + '"';
}
}  // namespace

VinsEstimator::VinsEstimator() : rclcpp::Node("vins_estimator") {
  options = std::make_shared<VINSOptions>();
  estimator_ = std::make_shared<Estimator>();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  initialize();
}

void VinsEstimator::reportFilter() {
  RCLCPP_INFO(this->get_logger(),
              "dynamic filter: %zu frames masked, %zu with no recent detection, "
              "%zu tracked points dropped onto dynamic regions, %zu masks "
              "discarded for exceeding max_mask_fraction",
              det_matched_, det_missed_, estimator_->dynamicDroppedPoints(),
              mask_rejected_);
  if (det_matched_ == 0) {
    RCLCPP_WARN(this->get_logger(),
                "filter is ON but NOT ONE frame has matched a detection. Is the "
                "detector node running, and is det_max_age (%.0f ms) long enough "
                "for its inference time?",
                det_max_age_ * 1e3);
  }
}

VinsEstimator::~VinsEstimator() {
  if (!output_path_.empty()) {
    std::ofstream out(output_path_ + "/filter_summary.csv");
    if (out) {
      out << "key,value\n"
          << "filter_enabled," << (filter_ ? 1 : 0) << '\n'
          << "detection_frames_matched," << det_matched_ << '\n'
          << "detection_frames_missed," << det_missed_ << '\n'
          << "masks_rejected," << mask_rejected_ << '\n'
          << "dynamic_points_dropped," << estimator_->dynamicDroppedPoints() << '\n';
    }
  }
  if (filter_) {
    RCLCPP_INFO(this->get_logger(),
                "dynamic filter: %zu frames masked, %zu with no recent detection, "
                "%zu tracked points dropped onto dynamic regions",
                det_matched_, det_missed_, estimator_->dynamicDroppedPoints());
    if (det_matched_ == 0) {
      RCLCPP_WARN(this->get_logger(),
                  "filter was ON but NOT ONE frame matched a detection. Is the "
                  "detector node running, and is det_max_age (%.0f ms) long enough "
                  "for its inference time?",
                  det_max_age_ * 1e3);
    }
    if (mask_rejected_ > 0) {
      RCLCPP_WARN(this->get_logger(),
                  "discarded %zu masks for exceeding max_mask_fraction",
                  mask_rejected_);
    }
  }
}

void VinsEstimator::initialize() {
  initializeParamters();
  initializeSubscribers();
  initializerPublishers();
}
void VinsEstimator::initializeParamters() {
  auto config_file = readParam<std::string>(this, "config_file");
  world_frame_id = readParam<std::string>(this, "world_frame_id", "world");
  body_frame_id = readParam<std::string>(this, "body_frame_id", "body");
  camera_frame_id = readParam<std::string>(this, "camera_frame_id", "camera");
  options->readParameters(config_file);

  // Optional ROS param "output_path" overrides the config's output_path, so each
  // node instance can write its own vio.csv without editing the yaml. The
  // directory and any missing parents are created automatically. Empty = keep
  // the config value.
  auto output_path = readParam<std::string>(this, "output_path", "");
  if (!output_path.empty()) {
    if (!rcpputils::fs::create_directories(rcpputils::fs::path(output_path))) {
      throw std::runtime_error("Could not create output_path directory: " +
                               output_path);
    }

    options->OUTPUT_FOLDER = output_path;
    options->VINS_RESULT_PATH = output_path + "/vio.csv";
    options->POSE_GRAPH_SAVE_PATH = output_path + "/pose_graph/";
    std::ofstream output_file(options->VINS_RESULT_PATH, std::ios::out);
    if (!output_file) {
      throw std::runtime_error("Could not create VINS result file: " +
                               options->VINS_RESULT_PATH);
    }
    RCLCPP_INFO(this->get_logger(), "output_path override -> %s",
                options->VINS_RESULT_PATH.c_str());
  }
  output_path_ = options->OUTPUT_FOLDER;

  // Optional ROS param "pose_graph_save_path" wins over both the config value and
  // the output_path-derived default above, so a run can point its pose graph
  // somewhere other than its vio.csv. Empty = keep whatever was resolved already.
  auto pose_graph_save_path =
      readParam<std::string>(this, "pose_graph_save_path", "");
  if (!pose_graph_save_path.empty()) {
    options->POSE_GRAPH_SAVE_PATH = pose_graph_save_path;
    RCLCPP_INFO(this->get_logger(), "pose_graph_save_path override -> %s",
                options->POSE_GRAPH_SAVE_PATH.c_str());
  }

  // --- dynamic object detection (RY-SLAM) -----------------------------------
  // OFF BY DEFAULT. With filter=false no detection subscription is created and
  // ImageData::mask stays empty, so FeatureTracker::setMask() takes its original
  // all-255 path -- the stock tracker, bit for bit, and the recorded baseline
  // stays reproducible.
  filter_ = readParam<bool>(this, "filter", false);
  mask_dilate_px_ = readParam<int>(this, "mask_dilate_px", 8);
  det_max_age_ = readParam<double>(this, "det_max_age", 0.15);
  // Measured on dataset/dynamic_dataset: dilated boxes cover mean 43.5% of the
  // frame, p90 61%, max 72.8%. 0.8 sits above the observed maximum, so the valve
  // only catches a runaway detection instead of firing during normal operation.
  max_mask_fraction_ = readParam<double>(this, "max_mask_fraction", 0.8);

  const auto experiment_id = readParam<std::string>(this, "experiment_id", "");
  const auto dataset_path = readParam<std::string>(this, "dataset_path", "");
  const auto world_path = readParam<std::string>(this, "world_path", "");
  const auto replay_rate = readParam<double>(this, "replay_rate", 1.0);
  const auto run_command = readParam<std::string>(this, "run_command", "");
  const auto run_notes = readParam<std::string>(this, "run_notes", "");

  if (!output_path_.empty()) {
    std::ofstream metadata(output_path_ + "/run_metadata.csv");
    if (!metadata) {
      throw std::runtime_error("Could not create run metadata in: " + output_path_);
    }
    metadata << "key,value\n";
    const std::vector<std::pair<std::string, std::string>> rows = {
        {"pipeline", "vins_fusion_ros2"}, {"experiment_id", experiment_id},
        {"dataset_path", dataset_path}, {"world_path", world_path},
        {"run_command", run_command}, {"run_notes", run_notes},
        {"config_file", config_file}, {"output_path", output_path_},
        {"image0_topic", options->imageTopic()},
        {"image1_topic", options->image1Topic()}, {"imu_topic", options->imuTopic()},
        {"world_frame_id", world_frame_id}, {"body_frame_id", body_frame_id},
        {"camera_frame_id", camera_frame_id}, {"filter", filter_ ? "true" : "false"},
        {"replay_rate", std::to_string(replay_rate)},
        {"det_max_age_s", std::to_string(det_max_age_)},
        {"mask_dilate_px", std::to_string(mask_dilate_px_)},
        {"max_mask_fraction", std::to_string(max_mask_fraction_)},
        {"start_wall_time_unix_ns", std::to_string(
             std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count())}};
    for (const auto& row : rows) {
      metadata << csvEscape(row.first) << ',' << csvEscape(row.second) << '\n';
    }
  }

  // What the filter ACTUALLY did this run, one row per frame, written next to
  // vio.csv. Distinct from scoring the detector offline with script/yolo_eval.py:
  // this records the masks the tracker really saw, including frames where no
  // detection was recent enough and masks rejected for covering too much.
  if (filter_ && !options->OUTPUT_FOLDER.empty()) {
    const std::string p = options->OUTPUT_FOLDER + "/yolo_mask.csv";
    mask_log_.open(p);
    if (mask_log_) {
      mask_log_ << "timestamp_ns,n_boxes,mask_coverage,matched,det_age_ms,rejected\n";
      RCLCPP_INFO(this->get_logger(), "mask log -> %s", p.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "could not open %s", p.c_str());
    }
  }

  estimator_->initialize(options);
}
void VinsEstimator::initializeSubscribers() {
  imu_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  feature_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt_imu;
  sub_opt_imu.callback_group = imu_callback_group_;

  rclcpp::SubscriptionOptions sub_opt_image;
  sub_opt_image.callback_group = image_callback_group_;

  rclcpp::SubscriptionOptions sub_opt_feature;
  sub_opt_feature.callback_group = feature_callback_group_;

  if (options->hasImu()) {
    // Deep IMU queue (~25s at 80Hz): during the init burst or any processing hiccup the
    // estimator can briefly fall behind, and dropping IMU here would create a large gap
    // between processed frames that corrupts pre-integration. Reliable by default.
    auto imu = this->create_subscription<sensor_msgs::msg::Imu>(
        options->imuTopic(), rclcpp::QoS(rclcpp::KeepLast(2000)),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          auto imu_msg = fromMsg(*msg);
          estimator_->inputIMU(imu_msg);
        },
        sub_opt_imu);
    subs_.push_back(imu);
  }

  if (options->isUsingStereo()) {
    // Reliable + deep image QoS instead of the best-effort sensor-data profile. The
    // default best-effort/depth-5 profile silently drops stereo frames when the node
    // falls behind real time (worst during initialization), which produces multi-second
    // gaps between processed frames -> NaN in IMU pre-integration. Buffering them keeps
    // the frame cadence intact. Bags here are published RELIABLE, so this matches.
    rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
    image_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    image_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    image_qos.depth = 100;

    sub_img0_filter_ = std::make_shared<Subscriber<Image>>(
        this, options->imageTopic(), image_qos, sub_opt_image);

    sub_img1_filter_ = std::make_shared<Subscriber<Image>>(
        this, options->image1Topic(), image_qos, sub_opt_image);

    sync_img_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(100), *sub_img0_filter_, *sub_img1_filter_);

    sync_img_->registerCallback(std::bind(&VinsEstimator::stereoCallback, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  } else {
    auto sub_img0 = this->create_subscription<sensor_msgs::msg::Image>(
        options->imageTopic(), rclcpp::QoS(rclcpp::KeepLast(100)),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          ImageData image;
          image.image0 = fromMsg(*msg);
          image.timestamp = fromMsg(msg->header.stamp);
          image.mask = maskFor(image.timestamp, image.image0.size());
          logMask(msg->header.stamp);
          estimator_->inputImage(image);
        },
        sub_opt_image);
    subs_.push_back(sub_img0);
  }

  if (filter_) {
    dets_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opt_dets;
    sub_opt_dets.callback_group = dets_callback_group_;
    auto dets_topic =
        readParam<std::string>(this, "dynamic_dets_topic", "/orbslam3/dynamic_dets");
    dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        dets_topic, rclcpp::SensorDataQoS(),
        std::bind(&VinsEstimator::onDetections, this, std::placeholders::_1),
        sub_opt_dets);
    RCLCPP_INFO(this->get_logger(),
                "filter: ON -- masking dynamic objects from %s (dilate %d px, "
                "max age %.0f ms, max coverage %.0f%%)",
                dets_topic.c_str(), mask_dilate_px_, det_max_age_ * 1e3,
                max_mask_fraction_ * 100.0);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "filter: off (stock VINS feature tracking)");
  }

  auto sub_feature = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(100)),
      [this](const sensor_msgs::msg::PointCloud::SharedPtr msg) {
        estimator_->inputFeature(fromMsg(msg->header.stamp), fromMsg(*msg));
      },
      sub_opt_feature);
  subs_.push_back(sub_feature);

  publish_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&VinsEstimator::timeCallback, this));

  if (filter_) {
    filter_report_timer_ =
        this->create_wall_timer(std::chrono::seconds(10),
                                std::bind(&VinsEstimator::reportFilter, this));
  }
}
void VinsEstimator::initializerPublishers() {
  pub_latest_odometry =
      this->create_publisher<nav_msgs::msg::Odometry>("imu_propagate", 1);
  pub_path = this->create_publisher<nav_msgs::msg::Path>("path", 1);
  pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
  pub_image_track =
      this->create_publisher<sensor_msgs::msg::Image>("image_track", 1);
  pub_point_cloud =
      this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 1);
  pub_margin_cloud =
      this->create_publisher<sensor_msgs::msg::PointCloud>("margin_cloud", 1);
  pub_keyframe_point =
      this->create_publisher<sensor_msgs::msg::PointCloud>("keyframe_point", 1);
  pub_keyframe_pose =
      this->create_publisher<nav_msgs::msg::Odometry>("keyframe_pose", 1);
}

void VinsEstimator::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img0,
    const sensor_msgs::msg::Image::ConstSharedPtr& img1) {
  ImageData image;
  image.image0 = fromMsg(*img0);
  image.image1 = fromMsg(*img1);
  image.timestamp = fromMsg(img0->header.stamp);
  image.mask = maskFor(image.timestamp, image.image0.size());
  logMask(img0->header.stamp);
  estimator_->inputImage(image);
}

void VinsEstimator::onDetections(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg) {
  std::vector<cv::Rect2f> boxes;
  boxes.reserve(msg->detections.size());
  for (const auto& det : msg->detections) {
    const float w = static_cast<float>(det.bbox.size_x);
    const float h = static_cast<float>(det.bbox.size_y);
    if (w <= 0.0f || h <= 0.0f) continue;
    boxes.emplace_back(static_cast<float>(det.bbox.center.position.x) - 0.5f * w,
                       static_cast<float>(det.bbox.center.position.y) - 0.5f * h,
                       w, h);
  }
  std::lock_guard<std::mutex> lk(dets_mu_);
  dets_.emplace_back(fromMsg(msg->header.stamp), std::move(boxes));
  // A couple of seconds of history at camera rate; anything older than
  // det_max_age is unusable anyway, so this only bounds memory.
  while (dets_.size() > 60) dets_.pop_front();
}

cv::Mat VinsEstimator::maskFor(double t, const cv::Size& size) {
  if (!filter_) return cv::Mat();

  std::vector<cv::Rect2f> boxes;
  {
    std::lock_guard<std::mutex> lk(dets_mu_);
    double best = det_max_age_;
    const std::vector<cv::Rect2f>* pick = nullptr;
    for (const auto& entry : dets_) {
      const double age = std::fabs(entry.first - t);
      if (age <= best) {
        best = age;
        pick = &entry.second;
      }
    }
    if (!pick) {
      ++det_missed_;
      last_ = LogRow{0, 0.0, false, -1.0, false};
      return cv::Mat();
    }
    boxes = *pick;
    last_age_ = best;
  }
  ++det_matched_;

  // A real "nothing dynamic here" answer. An empty Mat is both correct and
  // cheaper than an all-255 one: setMask() then takes its original path.
  if (boxes.empty()) {
    last_ = LogRow{0, 0.0, true, last_age_, false};
    return cv::Mat();
  }

  cv::Mat mask(size, CV_8UC1, cv::Scalar(255));
  const int d = std::max(0, mask_dilate_px_);
  double dynamic_area = 0.0;
  for (const auto& b : boxes) {
    cv::Rect r(static_cast<int>(std::floor(b.x)) - d,
               static_cast<int>(std::floor(b.y)) - d,
               static_cast<int>(std::ceil(b.width)) + 2 * d,
               static_cast<int>(std::ceil(b.height)) + 2 * d);
    r &= cv::Rect(0, 0, size.width, size.height);
    if (r.area() <= 0) continue;
    // Count before painting so overlapping boxes are not double-counted.
    dynamic_area += cv::countNonZero(mask(r));
    mask(r).setTo(0);
  }

  const double fraction = dynamic_area / static_cast<double>(size.area());
  if (fraction > max_mask_fraction_) {
    ++mask_rejected_;
    last_ = LogRow{boxes.size(), fraction, true, last_age_, true};
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "dynamic mask would cover %.0f%% of the frame (limit "
                         "%.0f%%) -- ignoring it for this frame rather than "
                         "starving the tracker",
                         fraction * 100.0, max_mask_fraction_ * 100.0);
    return cv::Mat();
  }
  last_ = LogRow{boxes.size(), fraction, true, last_age_, false};
  return mask;
}

void VinsEstimator::logMask(const builtin_interfaces::msg::Time& stamp) {
  if (!mask_log_) return;
  const int64_t ns = static_cast<int64_t>(stamp.sec) * 1000000000LL +
                     static_cast<int64_t>(stamp.nanosec);
  mask_log_ << ns << ',' << last_.n_boxes << ','
            << std::fixed << std::setprecision(4) << last_.coverage << ','
            << (last_.matched ? 1 : 0) << ',' << std::setprecision(1)
            << (last_.age_s < 0 ? -1.0 : last_.age_s * 1e3) << ','
            << (last_.rejected ? 1 : 0) << '\n';
  // Flush every row: this node's executor does not reliably return from spin()
  // on SIGINT, so the destructor cannot be trusted to close the stream and the
  // final line would be left truncated.
  mask_log_.flush();
}

void VinsEstimator::timeCallback() {
  publishImuData();
  publishOdometry();
  publishKeyFrameData();
  publishImage();
  publishPointCloud();
}

void VinsEstimator::publishPointCloud() {
  PointCloudData cloud;
  if (estimator_->getMainCloud(cloud)) {
    auto msg = toMsg(cloud);
    msg.header.frame_id = world_frame_id;
    pub_point_cloud->publish(msg);
  }

  if (estimator_->getMarginCloud(cloud)) {
    auto msg = toMsg(cloud);
    msg.header.frame_id = world_frame_id;
    pub_margin_cloud->publish(msg);
  }

  if (estimator_->getkeyframeCloud(cloud)) {
    auto msg = toMsg(cloud);
    msg.header.frame_id = world_frame_id;
    pub_keyframe_point->publish(msg);
  }
}

void VinsEstimator::publishImage() {
  ImageData image;
  if (estimator_->getTrackImage(image)) {
    auto img = toMsg(image.image0);
    img.header.frame_id = world_frame_id;
    img.header.stamp = toMsg(image.timestamp);
    pub_image_track->publish(img);
  }
}
void VinsEstimator::publishImuData() {
  OdomData imu_odom;
  if (estimator_->getIntegratedImuOdom(imu_odom)) {
    nav_msgs::msg::Odometry odometry = toMsg(imu_odom);
    odometry.header.frame_id = world_frame_id;
    odometry.child_frame_id = body_frame_id;
    pub_latest_odometry->publish(odometry);
  }
}
void VinsEstimator::publishOdometry() {
  OdomData vio_odom;
  if (estimator_->getVisualInertialOdom(vio_odom)) {
    nav_msgs::msg::Odometry odometry = toMsg(vio_odom);
    odometry.header.frame_id = world_frame_id;
    odometry.child_frame_id = body_frame_id;

    pub_odometry->publish(odometry);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.header.frame_id = world_frame_id;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.header.frame_id = world_frame_id;
    path.poses.push_back(pose_stamped);
    pub_path->publish(path);

    // world-->body
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odometry.header.stamp;
    tf_msg.header.frame_id = world_frame_id;
    tf_msg.child_frame_id = body_frame_id;
    tf_msg.transform.translation.x = odometry.pose.pose.position.x;
    tf_msg.transform.translation.y = odometry.pose.pose.position.y;
    tf_msg.transform.translation.z = odometry.pose.pose.position.z;
    tf_msg.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
    PoseData camera_pose;
    estimator_->getCameraPose(0, camera_pose);

    // body-->camera
    tf_msg.header.frame_id = body_frame_id;
    tf_msg.child_frame_id = camera_frame_id;
    tf_msg.transform.translation.x = camera_pose.position.x();
    tf_msg.transform.translation.y = camera_pose.position.y();
    tf_msg.transform.translation.z = camera_pose.position.z();
    tf_msg.transform.rotation.x = camera_pose.orientation.x();
    tf_msg.transform.rotation.y = camera_pose.orientation.y();
    tf_msg.transform.rotation.z = camera_pose.orientation.z();
    tf_msg.transform.rotation.w = camera_pose.orientation.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void VinsEstimator::publishKeyFrameData() {
  PoseData pose;
  if (estimator_->getkeyframePose(pose)) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = toMsg(pose.timestamp);
    odometry.header.frame_id = world_frame_id;
    odometry.child_frame_id = body_frame_id;
    odometry.pose.pose.position.x = pose.position.x();
    odometry.pose.pose.position.y = pose.position.y();
    odometry.pose.pose.position.z = pose.position.z();
    odometry.pose.pose.orientation.x = pose.orientation.x();
    odometry.pose.pose.orientation.y = pose.orientation.y();
    odometry.pose.pose.orientation.z = pose.orientation.z();
    odometry.pose.pose.orientation.w = pose.orientation.w();
    pub_keyframe_pose->publish(odometry);
  }
}
