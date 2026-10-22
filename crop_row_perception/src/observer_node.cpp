#include "crop_row_perception/observer_node.hpp"
#include "crop_row_perception/core.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <weeder_msgs/msg/crop_row_observation.hpp>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

namespace crop_row_perception
{
using Observation = weeder_msgs::msg::CropRowObservation;
using Image = sensor_msgs::msg::Image;
class ObserverNode::Impl
{
  struct Pending {
    Image::ConstSharedPtr image;
    uint64_t capture_ns, sequence;
  };
public:
  explicit Impl(ObserverNode & node) : node_(node) {
    if (node_.get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("use_sim_time is unsupported; use wall-paced replay");
    }
    const auto input = node_.declare_parameter<std::string>("image_topic", "/camera/image_raw");
    const auto output = node_.declare_parameter<std::string>(
      "observation_topic", "/crop_row/observation");
    expected_frame_ = node_.declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    vehicle_frame_ = node_.declare_parameter<std::string>("vehicle_frame", "base_link");
    calibration_id_ = node_.declare_parameter<std::string>("calibration_id", "");
    replay_ = node_.declare_parameter<bool>("replay", false);
    max_age_ms_ = node_.declare_parameter<double>("max_frame_age_ms", 250.0);
    const auto model = node_.declare_parameter<std::string>("model_path", "");
    const auto intrinsics = node_.declare_parameter<std::string>("intrinsic_path", "");
    const auto bev = node_.declare_parameter<std::string>("bev_path", "");
    const auto backend = node_.declare_parameter<std::string>("backend", "onnx");
    const auto width = node_.declare_parameter<int>("model_width", 640);
    const auto height = node_.declare_parameter<int>("model_height", 640);
    const auto detection = node_.declare_parameter<double>("detection_confidence", 0.25);
    const auto navigation = node_.declare_parameter<double>("navigation_confidence", 0.35);
    if (input.empty() || output.empty() || expected_frame_.empty() || vehicle_frame_.empty() ||
      calibration_id_.empty() || model.empty() || intrinsics.empty() || bev.empty() ||
      !std::isfinite(max_age_ms_) || max_age_ms_ <= 0 || max_age_ms_ > 5000 ||
      width <= 0 || width > 8192 || height <= 0 || height > 8192 ||
      !std::isfinite(detection) || detection < 0 || detection > 1 ||
      !std::isfinite(navigation) || navigation < 0 || navigation > 1 ||
      (backend != "onnx" && backend != "rknn"))
    {
      throw std::invalid_argument("invalid observer config; explicit model/calibration paths and ID required");
    }
    rk_row_nav_config_t config;
    rk_row_nav_config_init(&config);
    config.model_path = model.c_str();
    config.intrinsic_path = intrinsics.c_str();
    config.bev_path = bev.c_str();
    config.backend = backend == "onnx" ? RK_ROW_NAV_ONNX : RK_ROW_NAV_RKNN;
    config.model_width = static_cast<uint32_t>(width);
    config.model_height = static_cast<uint32_t>(height);
    config.detection_confidence = static_cast<float>(detection);
    config.minimum_navigation_confidence = static_cast<float>(navigation);
    config.max_frame_age_ms = max_age_ms_;
    sdk_ = std::make_unique<RowNavSdk>(config);
    session_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
      "-" + std::to_string(rk_row_nav_now_ns());
    // Volatile QoS: late subscribers must never inherit a cached VALID result.
    pub_ = node_.create_publisher<Observation>(output, rclcpp::QoS(1).reliable().durability_volatile());
    sub_ = node_.create_subscription<Image>(input, rclcpp::SensorDataQoS().keep_last(1),
      [this](Image::ConstSharedPtr image) {receive(std::move(image));});
    timer_ = node_.create_wall_timer(std::chrono::milliseconds(50), [this] {watchdog();});
    parameter_guard_ = node_.add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "Observer parameters are startup-only; update config and restart";
        return result;
      });
    worker_ = std::thread([this] {work();});
    RCLCPP_INFO(node_.get_logger(), "Observation-only: %s -> %s; SDK %s; replay=%d",
      input.c_str(), output.c_str(), rk_row_nav_version(), replay_);
  }
  ~Impl() {
    timer_->cancel();
    queue_.close();
    if (worker_.joinable()) {worker_.join();}
  }
private:
  Observation empty(uint8_t status, const std::string & detail) const {
    Observation msg;
    msg.header.frame_id = vehicle_frame_;
    msg.session_id = session_;
    msg.replay = replay_;
    msg.status = status;
    msg.detail = detail;
    msg.lateral_error_m = msg.heading_error_rad = msg.lookahead_m =
      std::numeric_limits<double>::quiet_NaN();
    msg.processing_ms = msg.frame_age_ms = std::numeric_limits<double>::quiet_NaN();
    msg.sdk_version = rk_row_nav_version();
    msg.calibration_id = calibration_id_;
    msg.dropped_frames = queue_.dropped();
    return msg;
  }
  void reject(uint64_t seq, const Image & image, const std::string & detail) {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    reject_sequence_ = seq;
    auto msg = empty(Observation::INVALID_INPUT, detail);
    msg.sequence = seq;
    msg.header.stamp = image.header.stamp;
    msg.has_capture_time = image.header.stamp.sec > 0;
    last_result_ = msg;
    last_result_capture_ns_ = 0;
    has_result_ = true;
    pub_->publish(msg);
  }
  void receive(Image::ConstSharedPtr image) {
    const uint64_t seq = ++sequence_;
    const int64_t stamp = static_cast<int64_t>(image->header.stamp.sec) * 1000000000LL +
      image->header.stamp.nanosec;
    const auto mapped = clock_.map(stamp, node_.now().nanoseconds(), rk_row_nav_now_ns(), max_age_ms_);
    if (!mapped || image->header.stamp.nanosec >= 1000000000u ||
      stamp <= last_stamp_ || image->header.frame_id != expected_frame_ ||
      (image->encoding != "bgr8" && image->encoding != "rgb8") ||
      !image->width || !image->height || image->width > 32768 || image->height > 32768 ||
      image->step < uint64_t(image->width) * 3 ||
      image->data.size() < uint64_t(image->step) * image->height)
    {
      reject(seq, *image, "bad encoding/frame/buffer or stale/future/non-increasing stamp/clock jump");
      return;
    }
    last_stamp_ = stamp;
    queue_.put(Pending{std::move(image), *mapped, seq});
  }
  void work() {
    while (auto pending = queue_.take()) {
      const auto & image = *pending->image;
      rk_row_nav_frame_t frame;
      rk_row_nav_frame_init(&frame);
      frame.data = image.data.data();
      frame.data_size_bytes = image.data.size();
      frame.width = image.width; frame.height = image.height; frame.stride_bytes = image.step;
      frame.pixel_format = image.encoding == "bgr8" ? RK_ROW_NAV_BGR888 : RK_ROW_NAV_RGB888;
      frame.capture_time_ns = pending->capture_ns; frame.frame_id = pending->sequence;
      rk_row_nav_result_t result;
      const auto code = sdk_->process(frame, result);
      auto msg = empty(Observation::SDK_ERROR, code == RK_ROW_NAV_OK ? "" : rk_row_nav_get_last_error());
      msg.sdk_return_code = code;
      msg.header.stamp = image.header.stamp;
      msg.has_capture_time = true;
      msg.sequence = pending->sequence;
      if (code == RK_ROW_NAV_OK) {
        msg.processing_ms = result.processing_ms;
        if (result.observation_status <= RK_ROW_NAV_OBSERVATION_STALE) {
          msg.status = static_cast<uint8_t>(result.observation_status);
        } else {
          msg.detail = "unknown SDK observation status";
        }
        if (usable(result)) {
          msg.lateral_error_m = result.lateral_error_m;
          msg.heading_error_rad = result.heading_error_rad;
          msg.lookahead_m = result.lookahead_m;
          msg.confidence = result.confidence;
        } else if (msg.status == Observation::VALID) {
          msg.status = Observation::SDK_ERROR;
          msg.detail = "SDK returned non-finite/out-of-range VALID result";
        }
        if (result.frame_id != frame.frame_id || result.capture_time_ns != frame.capture_time_ns) {
          msg.status = Observation::SDK_ERROR;
          msg.detail = "SDK result does not match input frame";
          msg.lateral_error_m = msg.heading_error_rad = msg.lookahead_m =
            std::numeric_limits<double>::quiet_NaN();
          msg.confidence = 0;
        }
      }
      std::lock_guard<std::mutex> lock(publish_mutex_);
      if (pending->sequence < reject_sequence_) {continue;}
      const auto now = rk_row_nav_now_ns();
      msg.frame_age_ms = static_cast<double>(now - pending->capture_ns) / 1e6;
      if (msg.frame_age_ms > max_age_ms_) {
        msg.status = Observation::STALE;
        msg.lateral_error_m = msg.heading_error_rad = msg.lookahead_m =
          std::numeric_limits<double>::quiet_NaN();
        msg.confidence = 0;
      }
      last_result_capture_ns_ = pending->capture_ns;
      last_result_ = msg;
      has_result_ = true;
      pub_->publish(msg);
    }
  }
  void watchdog() {
    std::lock_guard<std::mutex> lock(publish_mutex_);
    const auto now = rk_row_nav_now_ns();
    if (!has_result_) {
      pub_->publish(empty(Observation::NO_INPUT, "waiting for first processed image"));
    } else if (last_result_capture_ns_ &&
      static_cast<double>(now - last_result_capture_ns_) / 1e6 > max_age_ms_)
    {
      auto msg = empty(Observation::STALE, "no fresh processed observation");
      msg.sequence = last_result_.sequence;
      msg.header.stamp = last_result_.header.stamp;
      msg.has_capture_time = true;
      msg.frame_age_ms = static_cast<double>(now - last_result_capture_ns_) / 1e6;
      last_result_ = msg;
      pub_->publish(msg);
    } else if (last_result_.status != Observation::VALID) {
      auto msg = last_result_;
      if (last_result_capture_ns_) {
        msg.frame_age_ms = static_cast<double>(now - last_result_capture_ns_) / 1e6;
      }
      msg.dropped_frames = queue_.dropped();
      pub_->publish(msg);
    }
  }
  ObserverNode & node_;
  std::unique_ptr<RowNavSdk> sdk_;
  LatestFrameQueue<Pending> queue_;
  FrameClockMapper clock_;
  std::thread worker_;
  std::mutex publish_mutex_;
  uint64_t sequence_{0}, reject_sequence_{0}, last_result_capture_ns_{0};
  int64_t last_stamp_{0};
  double max_age_ms_{250};
  bool replay_{false};
  bool has_result_{false};
  std::string expected_frame_, vehicle_frame_, calibration_id_, session_;
  Observation last_result_;
  rclcpp::Publisher<Observation>::SharedPtr pub_;
  rclcpp::Subscription<Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
ObserverNode::ObserverNode(const rclcpp::NodeOptions & options)
: Node("crop_row_perception", options), impl_(std::make_unique<Impl>(*this)) {}
ObserverNode::~ObserverNode() = default;
}  // namespace crop_row_perception
