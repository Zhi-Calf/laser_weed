#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/gnss_driver.hpp"
#include "vehicle_interfaces/msg/pose.hpp"

namespace {

constexpr uint32_t kCrc32Polynomial = 0xEDB88320U;

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

  bool ok() const {
    return master_fd_ >= 0 && !slave_path_.empty();
  }

  int master_fd() const {
    return master_fd_;
  }

  const std::string& slave_path() const {
    return slave_path_;
  }

private:
  int master_fd_{-1};
  std::string slave_path_;
};

bool WriteAll(int fd, const std::string& data) {
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t count = write(fd, data.data() + written, data.size() - written);
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

uint32_t Crc32(const std::string& body) {
  uint32_t crc = 0;
  for (const unsigned char byte : body) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kCrc32Polynomial : crc >> 1U;
    }
  }
  return crc;
}

std::string Inspvaa(double gps_seconds) {
  std::ostringstream body;
  body << "INSPVAA,COM1,0,54%,FINE,2445," << std::fixed << std::setprecision(3) << gps_seconds
       << ",0,14,18;2445," << std::fixed << std::setprecision(3) << gps_seconds
       << ",34.76569927,113.55006586,135.934052,0.000000,0.000000,0.000000,"
          "0.182769,-0.947825,90.765137,INS_SOLUTION_GOOD";
  const std::string text = body.str();
  std::ostringstream sentence;
  sentence << '#' << text << '*' << std::hex << std::nouppercase << std::setw(8) << std::setfill('0') << Crc32(text);
  return sentence.str() + "\r\n";
}

bool WaitForPoseCount(rclcpp::executors::SingleThreadedExecutor& executor, const std::atomic<std::size_t>& pose_count, std::size_t count) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
    if (pose_count.load() >= count) {
      return true;
    }
  }
  return false;
}

}   // namespace

// GGA quality/satellites 必须覆盖 INSPVAA 的状态；GGA 失锁后必须继续发布 valid=false。
TEST(Nav3120Driver, GgaControlsFixAndSatelliteStatus) {
  rclcpp::init(0, nullptr);
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "-p", "serial_port:=" + pty.slave_path(),
    "-p", "baud_rate:=115200",
    "-p", "input_protocol:=inspvaa_gga",
    "-p", "origin_lat:=34.76569927",
    "-p", "origin_lon:=113.55006586",
    "-p", "origin_alt:=135.934052",
    "-p", "gga_timeout_sec:=0.5",
    "-p", "log_enable_console:=false",
    "-p", "log_dir:=/tmp",
  });
  auto driver = std::make_shared<gnss_localization::GnssDriver>(
      "nav3120_driver_node", options);
  auto listener = std::make_shared<rclcpp::Node>("nav3120_driver_test_listener");
  EXPECT_EQ(driver->get_name(), std::string("nav3120_driver_node"));
  const auto topics = driver->get_topic_names_and_types();
  EXPECT_NE(topics.find("/gnss_driver/pose"), topics.end());
  EXPECT_EQ(topics.find("/nav3120_driver_node/pose"), topics.end());

  std::mutex pose_mutex;
  std::vector<vehicle_interfaces::msg::Pose> poses;
  std::atomic<std::size_t> pose_count{0};
  auto subscription = listener->create_subscription<vehicle_interfaces::msg::Pose>(
    "/gnss_driver/pose",
    rclcpp::SensorDataQoS(),
    [&](const vehicle_interfaces::msg::Pose& pose) {
      std::lock_guard<std::mutex> lock(pose_mutex);
      poses.push_back(pose);
      pose_count.fetch_add(1);
    });

  ASSERT_TRUE(WriteAll(pty.master_fd(), "$GNGGA,094445.900,3445.94194793,N,11333.00395055,E,4,18,1.6,155.6865,M,-19.7205,M,1.9,2189*43\r\n"));
  ASSERT_TRUE(WriteAll(pty.master_fd(), Inspvaa(294303.900)));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(listener);
  ASSERT_TRUE(WaitForPoseCount(executor, pose_count, 1));
  {
    std::lock_guard<std::mutex> lock(pose_mutex);
    EXPECT_TRUE(poses.back().valid);
    EXPECT_EQ(poses.back().satellites, 18U);
  }

  ASSERT_TRUE(WriteAll(pty.master_fd(), "$GNGGA,,,,,,0,,,,,,,,*78\r\n"));
  ASSERT_TRUE(WriteAll(pty.master_fd(), Inspvaa(294303.901)));
  ASSERT_TRUE(WaitForPoseCount(executor, pose_count, 2));
  {
    std::lock_guard<std::mutex> lock(pose_mutex);
    EXPECT_FALSE(poses.back().valid);
    EXPECT_EQ(poses.back().satellites, 0U);
  }

  ASSERT_TRUE(WriteAll(pty.master_fd(), "$GNGGA,094445.900,3445.94194793,N,11333.00395055,E,4,18,1.6,155.6865,M,-19.7205,M,1.9,2189*43\r\n"));
  ASSERT_TRUE(WriteAll(pty.master_fd(), Inspvaa(294303.902)));
  ASSERT_TRUE(WaitForPoseCount(executor, pose_count, 3));
  {
    std::lock_guard<std::mutex> lock(pose_mutex);
    EXPECT_TRUE(poses.back().valid);
    EXPECT_EQ(poses.back().satellites, 18U);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  ASSERT_TRUE(WriteAll(pty.master_fd(), Inspvaa(294303.903)));
  ASSERT_TRUE(WaitForPoseCount(executor, pose_count, 4));
  {
    std::lock_guard<std::mutex> lock(pose_mutex);
    EXPECT_FALSE(poses.back().valid);
    EXPECT_EQ(poses.back().satellites, 0U);
  }

  subscription.reset();
  executor.remove_node(listener);
  executor.remove_node(driver);
  listener.reset();
  driver.reset();
  rclcpp::shutdown();
}
