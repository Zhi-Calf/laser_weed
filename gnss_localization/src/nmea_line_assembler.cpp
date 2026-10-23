#include "gnss_localization/nmea_line_assembler.hpp"

namespace gnss_localization {

// 清空当前组帧状态。
void NmeaLineAssembler::Reset() {
  line_length_ = 0;
  collecting_  = false;
  discarding_  = false;
}

// 逐字节组装 NMEA 行；
// 异常字节会丢弃当前半帧并等待下一个 '$'。
void NmeaLineAssembler::Push(
  const char* data, std::size_t size,
  const std::function<void(const std::string&)>& line_callback) {
  if (data == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < size; ++i) {
    PushByte(static_cast<unsigned char>(data[i]), line_callback);
  }
}

// 处理单个字节的组帧、溢出和重新同步状态。
void NmeaLineAssembler::PushByte(
  unsigned char byte,
  const std::function<void(const std::string&)>& line_callback) {
  if (byte == '$') {
    if (collecting_ && line_length_ > 0 && line_callback) {
      line_callback(std::string(line_buffer_.data(), line_length_));
    }
    line_buffer_[0] = '$';
    line_length_    = 1;
    collecting_     = true;
    discarding_     = false;
    return;
  }

  if (byte == '\r' || byte == '\n') {
    if (collecting_ && line_length_ > 0 && line_callback) {
      line_callback(std::string(line_buffer_.data(), line_length_));
    }
    line_length_ = 0;
    collecting_  = false;
    discarding_  = false;
    return;
  }

  if (byte < 0x20 || byte > 0x7e) {
    line_length_ = 0;
    collecting_  = false;
    discarding_  = true;
    return;
  }

  if (discarding_) {
    return;
  }
  if (!collecting_) {
    return;
  }

  if (line_length_ >= line_buffer_.size()) {
    line_length_ = 0;
    collecting_  = false;
    discarding_  = true;
    return;
  }
  line_buffer_[line_length_++] = static_cast<char>(byte);
}

}   // namespace gnss_localization
