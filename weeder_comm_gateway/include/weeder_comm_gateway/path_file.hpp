#pragma once

// APP 上传 / 下载来的路径 JSON 文件的**落盘前**校验。
//
// 规则与 global_path_planner/src/file_path_parser.hpp 一致 (unit 只认 wgs84、path[] 每点 lat/lon
// 必填且在范围内、turn_flag 可选布尔、至少 2 点、crc32 给了就必须匹配)。那份解析器不随包安装,
// 这里按同一约定实现一遍; 规划器加载时还会再校验一次 —— 本处的目的只是:
// **坏文件不落盘, 不覆盖现场正在用的好文件**。
//
// 两边规则若有改动必须同步 (test/test_path_file.cpp 里有一条与规划器单测相同的 crc32 用例)。

#include <cstddef>
#include <cstdint>
#include <string>

namespace weeder_comm_gateway::path_file {

struct Summary {
  std::string task_name;
  size_t num_points   = 0;
  bool crc32_verified = false;   // 文件给了 crc32 且匹配; 没给 -> false (允许, 只是未校验)
};

// 标准 CRC-32 (IEEE 802.3, 与 zlib / Python zlib.crc32 同值)
uint32_t Crc32Ieee(const std::string& data);

// text 是文件全文。成功返回 true 并填 out; 失败返回 false, err 为中文原因 (直接回给 APP)。
bool Validate(const std::string& text, Summary& out, std::string& err);

}   // namespace weeder_comm_gateway::path_file
