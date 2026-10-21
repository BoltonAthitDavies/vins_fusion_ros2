#ifndef VINS_ESTIMATOR_H
#define VINS_ESTIMATOR_H
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <vins/estimator/estimator.h>
#include <vins_fusion_ros2/visualization.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <deque>
#include <map>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <utility>
#include <vector>

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using namespace message_filters;

class VinsEstimator : public rclcpp::Node {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;
  /// The sim and real bags carry ONLY /camN/image_raw/compressed. Decoding the
  /// JPEG here, as orbslam3_ros2 already does, removes the pair of
  /// `image_transport republish` processes that were otherwise required -- and
  /// with them a class of faults that cost real runs: raw 1280x720 rgb8 is
  /// 2.7 MB per message over best-effort DDS and is dropped nondeterministically
  /// (919 vs 813 frames in two identical runs), two republisher pairs on one
  /// topic deliver every frame twice, and a pair left over from an earlier
  /// session silently feeds later runs.
  using CompressedSyncPolicy =
      message_filters::sync_policies::ApproximateTime<CompressedImage,
                                                      CompressedImage>;

  VinsEstimator();
  ~VinsEstimator();

  void initialize();
  void initializeParamters();
  void initializeSubscribers();
  void initializerPublishers();

 private:
  void timeCallback();
  void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img0,
                      const sensor_msgs::msg::Image::ConstSharedPtr& img1);
  void stereoCompressedCallback(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img0,
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img1);
  /// Shared tail of both stereo callbacks: mask, log, hand to the estimator.
  void submitStereo(const cv::Mat& im0, const cv::Mat& im1, double t,
                    const builtin_interfaces::msg::Time& stamp);
  /// JPEG -> grayscale. Returns an empty Mat and warns on a decode failure.
  cv::Mat decodeCompressed(
      const sensor_msgs::msg::CompressedImage& msg, const char* which);

  /// Buffers YOLO detections by their SOURCE IMAGE stamp. Deliberately NOT part
  /// of the stereo message_filters sync: a 3-way ApproximateTime would drop
  /// stereo pairs whenever inference falls behind the camera, and losing frames
  /// costs VINS far more than an unfiltered frame does.
  void onDetections(
      const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg);
  /// CV_8UC1 mask for a frame: 255 = keep, 0 = dynamic. Empty when filtering is
  /// off, no detection is recent enough, or the mask would blank too much.
  cv::Mat maskFor(double t, const cv::Size& size);
  static int64_t stampNs(const builtin_interfaces::msg::Time& t);
  /// Motion gate: drain finished censuses from the estimator into verdicts.
  void applyPendingVotes();
  static double iou(const cv::Rect2f& a, const cv::Rect2f& b);
  /// One row per frame into yolo_mask.csv, next to vio.csv. Timestamp written as
  /// integer nanoseconds exactly as the trajectory is, so the two files join on
  /// column 0 with no tolerance fudging.
  void logMask(const builtin_interfaces::msg::Time& stamp);
  /// Periodic filter-health report. On a WALL timer, and not left to the
  /// destructor: this node's executor does not reliably return from spin() on
  /// SIGINT, so a shutdown-only report is never seen on a bag replay.
  void reportFilter();

  void publishPointCloud();
  void publishImuData();
  void publishOdometry();
  void publishImage();
  void publishKeyFrameData();

 private:
  std::shared_ptr<Estimator> estimator_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::shared_ptr<Subscriber<Image>> sub_img0_filter_;
  std::shared_ptr<Subscriber<Image>> sub_img1_filter_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_img_;
  std::shared_ptr<Subscriber<CompressedImage>> sub_cimg0_filter_;
  std::shared_ptr<Subscriber<CompressedImage>> sub_cimg1_filter_;
  std::shared_ptr<message_filters::Synchronizer<CompressedSyncPolicy>> sync_cimg_;
  /// "raw" (default) or "compressed"; appends /compressed to the configured
  /// topics when compressed.
  std::string image_transport_{"raw"};
  std::size_t decode_failures_ = 0;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feature_callback_group_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr filter_report_timer_;
  /// One-shot: errors if nothing publishes the detection topic (live mode).
  rclcpp::TimerBase::SharedPtr dets_watchdog_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  //
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_track;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry,
      pub_latest_odometry, pub_keyframe_pose;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud,
      pub_margin_cloud, pub_keyframe_point;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;
  rclcpp::CallbackGroup::SharedPtr dets_callback_group_;
  std::mutex dets_mu_;
  std::deque<std::pair<double, std::vector<cv::Rect2f>>> dets_;
  bool filter_ = false;
  int mask_dilate_px_ = 8;
  double det_max_age_ = 0.15;
  double max_mask_fraction_ = 0.8;
  /// What maskFor() decided about the frame currently being handed to the
  /// estimator. Recorded per frame so a run's trajectory can be explained by
  /// what was actually masked while producing it.
  struct LogRow {
    std::size_t n_boxes = 0;
    double coverage = 0.0;
    bool matched = false;
    double age_s = -1.0;
    bool rejected = false;
    std::size_t n_moving = 0, n_static = 0, n_unknown = 0;
  };
  LogRow last_;
  // Motion gate (same policy and parameters as the ORB-SLAM3 node). The census
  // comes back from the backend thread; prev_votes_ is the newest scored frame,
  // inflight_ holds boxes pushed but not yet scored, keyed by stamp.
  static constexpr int VERDICT_UNKNOWN = 0, VERDICT_STATIC = 1, VERDICT_MOVING = 2;
  struct VotedBox {
    cv::Rect2f box;
    int verdict = VERDICT_UNKNOWN;
    int since_vote = 0;  ///< frames since this chain was last actually scored
    int chain_age = 0;   ///< frames this box has been associated through
  };
  std::string filter_mode_{"class"};
  double motion_iou_ = 0.3;
  int motion_min_points_ = 15;
  double motion_inlier_ratio_ = 0.5;
  bool motion_unknown_mask_ = false;
  int motion_revote_frames_ = 15;
  int motion_min_box_age_ = 30;
  std::mutex votes_mu_;
  std::vector<VotedBox> prev_votes_;
  std::map<double, std::vector<VotedBox>> inflight_;
  std::vector<cv::Rect2f> vote_boxes_;
  std::ofstream votes_log_;
  std::size_t boxes_voted_moving_ = 0;
  std::size_t boxes_voted_static_ = 0;
  double last_age_ = -1.0;
  std::ofstream mask_log_;
  std::string output_path_;

  std::size_t det_matched_ = 0;
  std::size_t det_missed_ = 0;
  std::size_t mask_rejected_ = 0;

  //
  std::string world_frame_id;
  std::string body_frame_id;
  std::string camera_frame_id;

  nav_msgs::msg::Path path;
  std::shared_ptr<VINSOptions> options;
};

#endif  // VINS_ESTIMATOR_H
