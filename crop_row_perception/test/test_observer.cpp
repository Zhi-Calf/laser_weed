#include "crop_row_perception/observer_node.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/msg/image.hpp>
#include <weeder_msgs/msg/crop_row_observation.hpp>
#include <weeder_msgs/msg/pose.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>

using Observation = weeder_msgs::msg::CropRowObservation;
using Image = sensor_msgs::msg::Image;
using namespace std::chrono_literals;
extern "C" bool fake_sdk_busy();

class ObserverTest : public ::testing::Test
{
protected:
  virtual bool gateEnabled() const {return false;}
  virtual bool dynamicEnabled() const {return false;}
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"model_path", "fake"}, {"intrinsic_path", "fake"},
      {"bev_path", dynamicEnabled() ? "dynamic-fake" : "fake"},
      {"calibration_id", "test-only"}, {"max_frame_age_ms", 200.0},
      {"image_topic", "/crop_row_test/image"},
      {"attitude_gate.enabled", gateEnabled()},
      {"dynamic_bev.enabled", dynamicEnabled()},
      {"attitude_gate.pose_topic", "/crop_row_test/pose"},
      {"attitude_gate.reference_measured", true},
      {"attitude_gate.vehicle_axes_confirmed", true},
      {"observation_topic", "/crop_row_test/observation"}, {"replay", true}});
    observer = std::make_shared<crop_row_perception::ObserverNode>(options);
    client = std::make_shared<rclcpp::Node>("crop_row_test_client");
    pub = client->create_publisher<Image>("/crop_row_test/image", rclcpp::SensorDataQoS());
    sub = client->create_subscription<Observation>("/crop_row_test/observation",
      rclcpp::QoS(100).reliable(), [this](Observation::ConstSharedPtr msg) {messages.push_back(*msg);});
    executor.add_node(observer); executor.add_node(client);
    ASSERT_TRUE(wait([this] {return pub->get_subscription_count() > 0 &&
      sub->get_publisher_count() > 0;}));
  }
  void TearDown() override {
    executor.remove_node(observer); executor.remove_node(client);
    observer.reset(); client.reset();
  }
  bool wait(const std::function<bool()> & predicate, std::chrono::milliseconds duration = 2500ms) {
    const auto end = std::chrono::steady_clock::now() + duration;
    do {
      executor.spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() < end);
    return predicate();
  }
  Image image(uint8_t mode = 0) {
    Image value;
    value.header.stamp = client->now();
    value.header.frame_id = "camera_optical_frame";
    value.width = 2; value.height = 2; value.step = 8;  // row padding accepted
    value.encoding = "bgr8"; value.data.resize(16, mode);
    return value;
  }
  bool has(uint8_t status) const {
    return std::any_of(messages.begin(), messages.end(), [status](const auto & msg) {
      return msg.status == status;
    });
  }
  std::shared_ptr<crop_row_perception::ObserverNode> observer;
  rclcpp::Node::SharedPtr client;
  rclcpp::Publisher<Image>::SharedPtr pub;
  rclcpp::Subscription<Observation>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<Observation> messages;
};

TEST_F(ObserverTest, ValidObservationPreservesSignsCaptureAndBecomesStale) {
  ASSERT_TRUE(wait([this] {return has(Observation::NO_INPUT);}));
  const auto input = image();
  pub->publish(input);
  ASSERT_TRUE(wait([this] {return has(Observation::VALID);}));
  const auto valid = *std::find_if(messages.begin(), messages.end(), [](const auto & msg) {
    return msg.status == Observation::VALID;
  });
  EXPECT_EQ(valid.header.stamp, input.header.stamp);
  EXPECT_EQ(valid.header.frame_id, "base_link");
  EXPECT_TRUE(valid.has_capture_time); EXPECT_TRUE(valid.replay);
  EXPECT_FALSE(valid.session_id.empty()); EXPECT_EQ(valid.sdk_version, "test-only");
  EXPECT_DOUBLE_EQ(valid.lateral_error_m, 0.12);
  EXPECT_DOUBLE_EQ(valid.heading_error_rad, -0.03);
  ASSERT_TRUE(wait([this] {return has(Observation::STALE);}));
  EXPECT_TRUE(std::isnan(messages.back().lateral_error_m));
  EXPECT_DOUBLE_EQ(messages.back().confidence, 0);
}

TEST_F(ObserverTest, InvalidBuffersTimestampsAndFramesNeverReachSdk) {
  auto value = image(); value.encoding = "nv12"; pub->publish(value);
  ASSERT_TRUE(wait([this] {return has(Observation::INVALID_INPUT);}));
  for (int failure = 0; failure < 5; ++failure) {
    messages.clear(); value = image();
    if (failure == 0) {value.data.resize(1);}
    if (failure == 1) {value.header.frame_id = "wrong_frame";}
    if (failure == 2) {value.header.stamp.sec = 0; value.header.stamp.nanosec = 0;}
    if (failure == 3) {value.header.stamp.sec += 2;}
    if (failure == 4) {value.header.stamp.sec -= 2;}
    pub->publish(value);
    ASSERT_TRUE(wait([this, &value] {
      return std::any_of(messages.begin(), messages.end(), [&value](const auto & msg) {
        return msg.status == Observation::INVALID_INPUT && msg.header.stamp == value.header.stamp;
      });
    }));
    EXPECT_FALSE(has(Observation::VALID));
  }
}

TEST_F(ObserverTest, DuplicateStampDoesNotReuseValidResult) {
  const auto value = image(); pub->publish(value);
  ASSERT_TRUE(wait([this] {return has(Observation::VALID);}));
  messages.clear(); pub->publish(value);
  ASSERT_TRUE(wait([this] {return has(Observation::INVALID_INPUT);}));
  EXPECT_FALSE(has(Observation::VALID));
}

TEST_F(ObserverTest, InferenceErrorNoRowAndNanAreInvalidated) {
  for (uint8_t mode : {1, 2, 3, 5, 6, 7}) {
    messages.clear();
    const auto input = image(mode);
    pub->publish(input);
    const auto status = mode == 2 ? Observation::NO_ROW :
      (mode == 7 ? Observation::LOW_CONFIDENCE : Observation::SDK_ERROR);
    const auto matching = [status, &input](const auto & msg) {
      return msg.status == status && msg.header.stamp == input.header.stamp;
    };
    ASSERT_TRUE(wait([this, &matching] {
      return std::any_of(messages.begin(), messages.end(), matching);
    }));
    const auto & result = *std::find_if(messages.begin(), messages.end(), matching);
    EXPECT_TRUE(std::isnan(result.lateral_error_m)); EXPECT_DOUBLE_EQ(result.confidence, 0);
    EXPECT_FALSE(has(Observation::VALID));
  }
}

TEST_F(ObserverTest, SlowInferenceIsNeverPublishedAsValid) {
  pub->publish(image(4));
  ASSERT_TRUE(wait([this] {return has(Observation::STALE);}));
  EXPECT_FALSE(has(Observation::VALID));
}

TEST_F(ObserverTest, NewerInputErrorCannotBeOverwrittenByOlderInFlightResult) {
  pub->publish(image(4));
  ASSERT_TRUE(wait([] {return fake_sdk_busy();}));
  auto invalid = image(); invalid.encoding = "nv12";
  pub->publish(invalid);
  ASSERT_TRUE(wait([this] {return has(Observation::INVALID_INPUT);}));
  const auto rejected_sequence = messages.back().sequence;
  messages.clear();
  EXPECT_FALSE(wait([] {return false;}, 450ms));
  ASSERT_FALSE(messages.empty());
  for (const auto & msg : messages) {
    EXPECT_GE(msg.sequence, rejected_sequence);
    EXPECT_EQ(msg.status, Observation::INVALID_INPUT);
    EXPECT_DOUBLE_EQ(msg.confidence, 0);
  }
}

TEST_F(ObserverTest, NoRowAlsoExpiresWhenImageStreamStops) {
  pub->publish(image(2));
  ASSERT_TRUE(wait([this] {return has(Observation::NO_ROW);}));
  ASSERT_TRUE(wait([this] {return has(Observation::STALE);}));
}

TEST_F(ObserverTest, RuntimeCalibrationOrClockChangesAreRejected) {
  const auto result = observer->set_parameters_atomically({
    rclcpp::Parameter("vehicle_frame", "different_reference")});
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(observer->get_parameter("vehicle_frame").as_string(), "base_link");
  EXPECT_FALSE(observer->set_parameters_atomically({
    rclcpp::Parameter("use_sim_time", true)}).successful);
}

class AttitudeObserverTest : public ObserverTest
{
protected:
  bool gateEnabled() const override {return true;}
  void SetUp() override {
    ObserverTest::SetUp();
    pose_pub = client->create_publisher<weeder_msgs::msg::Pose>(
      "/crop_row_test/pose", rclcpp::SensorDataQoS());
    ASSERT_TRUE(wait([this] {return pose_pub->get_subscription_count() == 1;}));
  }
  void poseFor(const Image & image, bool valid = true, double roll = 0,
    int64_t offset_ns = -1000000, const std::string & frame = "odom", double pitch = 0)
  {
    weeder_msgs::msg::Pose pose;
    pose.header.stamp = rclcpp::Time(image.header.stamp) +
      rclcpp::Duration(std::chrono::nanoseconds(offset_ns));
    pose.header.frame_id = frame; pose.valid = valid; pose.roll = roll; pose.pitch = pitch;
    pose_pub->publish(pose);
    wait([] {return false;}, 30ms);  // deliver pose before image; no callback ordering assumptions
  }
  rclcpp::Publisher<weeder_msgs::msg::Pose>::SharedPtr pose_pub;
};

TEST_F(AttitudeObserverTest, NoPoseRejectsAndFreshPoseRecovers) {
  pub->publish(image());
  ASSERT_TRUE(wait([this] {return has(Observation::INVALID_INPUT);}));
  EXPECT_FALSE(has(Observation::VALID));
  messages.clear();
  const auto input = image(); poseFor(input); pub->publish(input);
  ASSERT_TRUE(wait([this] {return has(Observation::VALID);}));
  const auto & msg = *std::find_if(messages.begin(), messages.end(), [](const auto & m) {
    return m.status == Observation::VALID;
  });
  EXPECT_NE(msg.detail.find("static BEV, no dynamic compensation"), std::string::npos);
}

TEST_F(AttitudeObserverTest, InvalidStaleTiltAndWrongFrameNeverPublishValid) {
  for (int failure = 0; failure < 4; ++failure) {
    messages.clear(); const auto input = image();
    poseFor(input, failure != 0, failure == 2 ? 0.1 : 0.0,
      failure == 1 ? -100000000 : -1000000, failure == 3 ? "wrong" : "odom");
    pub->publish(input);
    ASSERT_TRUE(wait([this, &input] {
      return std::any_of(messages.begin(), messages.end(), [&input](const auto & msg) {
        return msg.status == Observation::INVALID_INPUT && msg.header.stamp == input.header.stamp;
      });
    }));
    EXPECT_FALSE(has(Observation::VALID));
    EXPECT_TRUE(std::isnan(messages.back().lateral_error_m));
    EXPECT_EQ(messages.back().confidence, 0);
    wait([] {return false;}, 120ms);  // keep even stale test source stamps increasing
  }
}

class DynamicObserverTest : public AttitudeObserverTest
{
protected:
  bool dynamicEnabled() const override {return true;}
};

TEST_F(DynamicObserverTest, MatchedPoseReachesSdkWithMappedMonotonicTime) {
  for (double roll : {-0.02,0.02}) {
    messages.clear(); const auto input = image();
    poseFor(input,true,roll,-1000000,"odom",-0.01); pub->publish(input);
    ASSERT_TRUE(wait([this] {return has(Observation::VALID);}));
    const auto & msg = *std::find_if(messages.begin(), messages.end(), [](const auto & m) {
      return m.status == Observation::VALID;
    });
    EXPECT_DOUBLE_EQ(msg.lateral_error_m,0.12+roll);
    EXPECT_DOUBLE_EQ(msg.heading_error_rad,-0.04);
    EXPECT_NE(msg.detail.find("dynamic BEV attitude applied"),std::string::npos);
    EXPECT_EQ(msg.header.stamp,input.header.stamp);
  }
}

TEST_F(DynamicObserverTest, MissingPoseNeverFallsBackToStaticSdk) {
  pub->publish(image());
  ASSERT_TRUE(wait([this] {return has(Observation::INVALID_INPUT);}));
  EXPECT_FALSE(has(Observation::VALID));
}

TEST(AttitudeConfigTest, EnablingWithoutMeasuredReferenceFailsBeforeSdkCreation) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"attitude_gate.enabled", true}});
  EXPECT_THROW(std::make_shared<crop_row_perception::ObserverNode>(options), std::invalid_argument);
  rclcpp::shutdown();
}

TEST(AttitudeConfigTest, DynamicModeMismatchAndMissingGateRejectStartup) {
  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"dynamic_bev.enabled",true}});
  EXPECT_THROW(std::make_shared<crop_row_perception::ObserverNode>(options),std::invalid_argument);
  options.parameter_overrides({{"model_path","fake"},{"intrinsic_path","fake"},
    {"bev_path","dynamic-fake"},{"calibration_id","fake"}});
  EXPECT_THROW(std::make_shared<crop_row_perception::ObserverNode>(options),std::invalid_argument);
  rclcpp::shutdown();
}
