/*
 * image_encoding_fixer -- relabel mislabelled single-channel Image messages as mono8.
 *
 * The WiL cameras publish CompressedImage with format "yuv422; jpeg compressed mono8":
 * the JPEG payload is single-channel, but image_transport's `republish` stamps the
 * *original* encoding ("yuv422", 2 bytes/pixel) onto the decoded 1-byte-per-pixel
 * image. Every cv_bridge consumer then rejects it with
 *     "Image is wrongly formed: step < width * byte_depth * num_channels
 *      or 1920 != 1920 * 1 * 2"
 * and never looks at the pixels, which are perfectly fine.
 *
 * vins_fusion_ros2_node tolerates this internally (see fromMsg in visualization.cpp),
 * but image_view, rqt and anything else using cv_bridge do not. This node fixes the
 * label on the wire so every consumer works.
 *
 * Usage:
 *   ros2 run vins_fusion_ros2 image_encoding_fixer --ros-args \
 *     -r in:=/cam0/image_raw -r out:=/cam0/image_mono
 *
 * The proper fix is upstream, in whatever publishes the CompressedImage: it should not
 * advertise yuv422 for a mono8 payload. This node is a shim for bags already recorded
 * and for rigs that cannot be reflashed.
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

// The row stride is ground truth: step == width means one byte per pixel, whatever the
// encoding field claims. Guard on data.size() too so a genuinely odd message is left
// alone rather than silently relabelled.
bool isSingleBytePerPixel(const sensor_msgs::msg::Image &m) {
  return m.width > 0 && m.height > 0 && m.step == m.width &&
         m.data.size() == static_cast<size_t>(m.step) * m.height;
}

class ImageEncodingFixer : public rclcpp::Node {
 public:
  ImageEncodingFixer() : rclcpp::Node("image_encoding_fixer") {
    // Match the deep reliable QoS vins_estimator uses for images, so this node does not
    // become the lossy link in the chain.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("out", qos);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "in", qos, [this](sensor_msgs::msg::Image::UniquePtr msg) {
          if (msg->encoding != "mono8" && isSingleBytePerPixel(*msg)) {
            if (!warned_) {
              RCLCPP_INFO(this->get_logger(),
                          "relabelling '%s' -> 'mono8' (step=%u, width=%u)",
                          msg->encoding.c_str(), msg->step, msg->width);
              warned_ = true;
            }
            msg->encoding = "mono8";
          }
          pub_->publish(std::move(msg));
        });
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  bool warned_ = false;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageEncodingFixer>());
  rclcpp::shutdown();
  return 0;
}
