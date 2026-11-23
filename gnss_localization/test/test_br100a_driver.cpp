// BR100A 驱动集成测试：通过伪终端注入 GIAVP，验证固定 GNSS 话题正常发布。
#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/gnss_driver.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vehicle_interfaces/msg/pose.hpp"

namespace {

class PtyPair {
 public:
  PtyPair() {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd_ < 0) {
      return;
    }
    if (grantpt(master_fd_) != 0 || unlockpt(master_fd_) != 0) {
      close(master_fd_);
      master_fd_ = -1;
      return;
    }
    const char* name = ptsname(master_fd_);
    if (name != nullptr) {
      slave_path_ = name;
    }
  }

  ~PtyPair() {
    if (master_fd_ >= 0) {
      close(master_fd_);
    }
  }

  bool ok() const { return master_fd_ >= 0 && !slave_path_.empty(); }
  int master_fd() const { return master_fd_; }
  const std::string& slave_path() const { return slave_path_; }

 private:
  int master_fd_{-1};
  std::string slave_path_;
};

bool WriteAll(int fd, const std::string& data) {
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t count =
        write(fd, data.data() + written, data.size() - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      return false;
    }
  }
  return true;
}

std::string GiavpSentence() {
  const std::string body =
      "GIAVP,2445,294303.900,90.0,0.0,0.0,34.70316078,112.39118753,"
      "102.0,0.5,0.0,0.0,1.5,20,19,4,1,1.8,0.01,0.02,9.81,0.0,0.0,0.1";
  unsigned char checksum = 0;
  for (const char character : body) {
    checksum ^= static_cast<unsigned char>(character);
  }
  std::ostringstream sentence;
  sentence << '$' << body << '*' << std::uppercase << std::hex
           << std::setw(2) << std::setfill('0')
           << static_cast<unsigned int>(checksum) << "\r\n";
  return sentence.str();
}

template <typename Count>
bool WaitForCount(rclcpp::executors::SingleThreadedExecutor& executor,
                  const Count& count, std::size_t expected) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
    if (count.load() >= expected) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(Br100aDriver, PublishesFixedGnssTopics) {
  rclcpp::init(0, nullptr);
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  rclcpp::NodeOptions options;
  options.arguments({
      "--ros-args",
      "-p", "serial_port:=" + pty.slave_path(),
      "-p", "baud_rate:=115200",
      "-p", "input_protocol:=giavp",
      "-p", "origin_lat:=34.70316078",
      "-p", "origin_lon:=112.39118753",
      "-p", "origin_alt:=102.0",
      "-p", "use_gps_timestamp:=false",
      "-p", "log_enable_console:=false",
      "-p", "log_dir:=/tmp",
  });
  auto driver = std::make_shared<gnss_localization::GnssDriver>(
      "br100a_driver_node", options);
  auto listener = std::make_shared<rclcpp::Node>("br100a_driver_test_listener");

  std::atomic<std::size_t> fix_count{0};
  std::atomic<std::size_t> pose_count{0};
  auto fix_subscription =
      listener->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/gnss_driver/fix", rclcpp::SensorDataQoS(),
          [&fix_count](sensor_msgs::msg::NavSatFix::ConstSharedPtr) {
            fix_count.fetch_add(1);
          });
  auto pose_subscription =
      listener->create_subscription<vehicle_interfaces::msg::Pose>(
          "/gnss_driver/pose", rclcpp::SensorDataQoS(),
          [&pose_count](vehicle_interfaces::msg::Pose::ConstSharedPtr) {
            pose_count.fetch_add(1);
          });

  ASSERT_TRUE(WriteAll(pty.master_fd(), GiavpSentence()));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(listener);
  ASSERT_TRUE(WaitForCount(executor, fix_count, 1));
  ASSERT_TRUE(WaitForCount(executor, pose_count, 1));

  EXPECT_EQ(driver->get_name(), std::string("br100a_driver_node"));
  const auto topics = driver->get_topic_names_and_types();
  EXPECT_NE(topics.find("/gnss_driver/fix"), topics.end());
  EXPECT_NE(topics.find("/gnss_driver/pose"), topics.end());
  EXPECT_EQ(topics.find("/br100a_driver_node/fix"), topics.end());
  EXPECT_EQ(topics.find("/br100a_driver_node/pose"), topics.end());

  executor.remove_node(listener);
  executor.remove_node(driver);
  rclcpp::shutdown();
}
