#include <fcntl.h>
#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#include <stdlib.h>
#include <unistd.h>

#include <cerrno>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "gnss_localization/serial_port.hpp"

namespace {

class PtyPair {
public:
  PtyPair() {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
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
  int master_fd_ = -1;       // PTY 主端
  std::string slave_path_;   // PTY 从端路径
};

bool WriteAll(int fd, const std::string& data) {
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t count = write(fd, data.data() + written, data.size() - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

class ScopedLogCapture {
public:
  ScopedLogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(stream_)), logger_(std::make_shared<spdlog::logger>("serial_port_test", sink_)), previous_(spdlog::default_logger()) {
    logger_->set_pattern("%l|%v");
    logger_->set_level(spdlog::level::trace);
    spdlog::set_default_logger(logger_);
  }

  ~ScopedLogCapture() {
    spdlog::set_default_logger(previous_);
  }

  std::string Text() {
    logger_->flush();
    return stream_.str();
  }

private:
  std::ostringstream stream_;                              // 日志文本
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;   // 测试 sink
  std::shared_ptr<spdlog::logger> logger_;                 // 测试日志器
  std::shared_ptr<spdlog::logger> previous_;               // 原默认日志器
};

}   // namespace

TEST(SerialPort, RejectsUnsupportedBaudRate) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  ScopedLogCapture logs;
  gnss_localization::SerialPort port;
  EXPECT_FALSE(port.open(pty.slave_path(), 12345));
  EXPECT_FALSE(port.is_open());
  EXPECT_NE(logs.Text().find("不支持的串口波特率 12345"), std::string::npos);
}

TEST(SerialPort, InvalidBaudRateKeepsExistingConnection) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));

  EXPECT_FALSE(port.open(pty.slave_path(), 12345));
  EXPECT_TRUE(port.is_open());
}

TEST(SerialPort, ReadsLineSplitAcrossReads) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));

  std::vector<std::string> lines;
  const auto collect = [&lines](const std::string& line) {
    lines.push_back(line);
  };
  ASSERT_TRUE(WriteAll(pty.master_fd(), "$GPG"));
  EXPECT_EQ(port.read_lines(collect), 0);
  ASSERT_TRUE(WriteAll(pty.master_fd(), "GA,1,2,3*00\r\n"));
  EXPECT_EQ(port.read_lines(collect), 1);

  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "$GPGGA,1,2,3*00");
}

TEST(SerialPort, FlushesCachedLineAtNextDollar) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));

  std::vector<std::string> lines;
  ASSERT_TRUE(WriteAll(pty.master_fd(), "$PART$GOOD\r\n"));
  EXPECT_EQ(
    port.read_lines([&lines](const std::string& line) {
      lines.push_back(line);
    }),
    2);

  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "$PART");
  EXPECT_EQ(lines[1], "$GOOD");
}

TEST(SerialPort, ClosedPortReturnsNoLines) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));
  port.close();

  EXPECT_FALSE(port.is_open());
  EXPECT_EQ(port.read_lines([](const std::string&) {}), 0);
}

TEST(SerialPort, WritesRawBytes) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));

  const std::string data = "\x01\x02\x03";
  EXPECT_TRUE(port.write_raw(data.data(), data.size()));
}

TEST(SerialPort, RejectsNullWriteData) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));

  EXPECT_FALSE(port.write_raw(nullptr, 1));
  EXPECT_TRUE(port.is_open());
  EXPECT_TRUE(port.write_raw(nullptr, 0));
}

TEST(SerialPort, WriteFailureClosesPort) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200));

  std::vector<char> payload(1024U * 1024U, 'x');
  EXPECT_FALSE(port.write_raw(payload.data(), payload.size()));
  EXPECT_FALSE(port.is_open());
}

TEST(SerialPort, InvalidDeviceLeavesPortClosed) {
  ScopedLogCapture logs;
  gnss_localization::SerialPort port;
  EXPECT_FALSE(port.open("/dev/does-not-exist", 115200));
  EXPECT_FALSE(port.is_open());
  EXPECT_NE(logs.Text().find("/dev/does-not-exist"), std::string::npos);
}

TEST(SerialPort, InvalidTerminalLeavesPortClosed) {
  ScopedLogCapture logs;
  gnss_localization::SerialPort port;
  EXPECT_FALSE(port.open("/dev/null", 115200));
  EXPECT_FALSE(port.is_open());
  EXPECT_NE(logs.Text().find("读取串口 /dev/null 参数失败"), std::string::npos);
}
