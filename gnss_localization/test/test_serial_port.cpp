#include <gtest/gtest.h>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <string>
#include <vector>

#include "gnss_localization/nmea_line_assembler.hpp"
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
  int master_fd_ = -1;
  std::string slave_path_;
};

bool WriteAll(int fd, const std::string& data) {
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t count =
      write(fd, data.data() + written, data.size() - written);
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

}   // namespace

TEST(NmeaLineAssembler, ReassemblesSplitCrLfLine) {
  gnss_localization::NmeaLineAssembler assembler;
  std::vector<std::string> lines;
  const auto callback = [&lines](const std::string& line) {
    lines.push_back(line);
  };

  assembler.Push("$GPG", 4, callback);
  EXPECT_TRUE(lines.empty());
  assembler.Push("GA,1,2,3*00", 11, callback);
  EXPECT_TRUE(lines.empty());
  assembler.Push("\r\n", 2, callback);

  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "$GPGGA,1,2,3*00");
}

TEST(NmeaLineAssembler, DiscardsOverlongAndBinaryFragments) {
  gnss_localization::NmeaLineAssembler assembler;
  std::vector<std::string> lines;
  const auto callback = [&lines](const std::string& line) {
    lines.push_back(line);
  };

  std::string overlong(1100, 'A');
  assembler.Push("$", 1, callback);
  assembler.Push(overlong.data(), overlong.size(), callback);
  assembler.Push("\r$PART", 6, callback);
  const char binary = static_cast<char>(0x01);
  assembler.Push(&binary, 1, callback);
  assembler.Push("$GOOD\r", 6, callback);

  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "$GOOD");
}

TEST(SerialPort, RejectsUnsupportedBaudRate) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  EXPECT_FALSE(port.open(pty.slave_path(), 12345));
  EXPECT_FALSE(port.is_open());
  EXPECT_NE(port.last_error().find("不支持"), std::string::npos);
  EXPECT_FALSE(port.open(pty.slave_path(), 256000));
  EXPECT_FALSE(port.is_open());
  EXPECT_NE(port.last_error().find("不支持"), std::string::npos);
}

TEST(SerialPort, OpensExclusivelyAndReadsAvailableLines) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200))
    << port.last_error();

  const int second_fd =
    open(pty.slave_path().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  EXPECT_LT(second_fd, 0);
  if (second_fd >= 0) {
    close(second_fd);
  }

  std::vector<std::string> lines;
  const auto collect = [&lines](const std::string& line) {
    lines.push_back(line);
  };
  ASSERT_TRUE(WriteAll(pty.master_fd(), "$GPGG"));
  EXPECT_EQ(port.read_lines(collect), 0);
  ASSERT_TRUE(WriteAll(pty.master_fd(), "A,1,2,3*00\r\n"));
  EXPECT_EQ(port.read_lines(collect), 1);

  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "$GPGGA,1,2,3*00");
}

TEST(SerialPort, WriteTimesOutAndClosesWhenPeerDoesNotDrain) {
  PtyPair pty;
  ASSERT_TRUE(pty.ok());

  gnss_localization::SerialPort port;
  ASSERT_TRUE(port.open(pty.slave_path(), 115200))
    << port.last_error();

  std::vector<char> payload(1024U * 1024U, 'x');
  EXPECT_FALSE(port.write_raw(
    payload.data(), payload.size(), std::chrono::milliseconds(20)));
  EXPECT_FALSE(port.is_open());
  EXPECT_FALSE(port.last_error().empty());
}
