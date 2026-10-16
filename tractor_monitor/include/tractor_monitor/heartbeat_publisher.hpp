#pragma once

// Drop-in heartbeat publisher for every monitored node.
//
//     // in the node's ctor:
//     heartbeat_ = std::make_unique<tractor_monitor::HeartbeatPublisher>(this, "path_controller");
//
//     // wherever health changes:
//     // 第二个参数 (故障来源) 必填: kFaultSourceSelf = 本节点自身, 否则写清是谁 (节点名/来源标签)。
//     heartbeat_->setOk();
//     heartbeat_->setStopImplement("waiting for /work_mode", tractor_monitor::kFaultSourceSelf);
//     heartbeat_->setStopMotion("no odometry", tractor_monitor::kFaultSourceSelf);
//     heartbeat_->setFault("no odometry for 2.0s", tractor_monitor::kFaultSourceSelf);
//     heartbeat_->setFault("no RTK pose", "gnss_driver");   // 别人造成的 -> 写清是谁
//
// It owns its own timer and publisher; the node never touches either. The point
// of a shared helper rather than ten hand-rolled timers is that the rate, the
// seq semantics and the meaning of DEGRADED are decided ONCE. Ten copies drift.
//
// Header-only and depending on nothing but rclcpp + tractor_msgs, so a package
// picking this up adds one leaf dependency and no build weight.

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tractor_monitor/fault_source.hpp>
#include <tractor_msgs/msg/heartbeat.hpp>
#include <type_traits>
#include <utility>
#include <vector>

namespace tractor_monitor {

// The shared liveness bus. ABSOLUTE on purpose -- see Heartbeat.msg.
inline constexpr const char* kHeartbeatTopic = "/heartbeat";
// tractor_monitor's aggregate verdict, consumed by tractor_base.
inline constexpr const char* kMonitorHeartbeatTopic = "/monitor_heartbeat";
// tractor_monitor's full per-node table. Observability only -- nothing in the
// control path consumes this, so adding to it cannot stop a tractor.
inline constexpr const char* kSystemHealthTopic = "/system_health";

namespace detail {

// LifecyclePublisher has is_activated(); plain Publisher does not. Publishing on
// an inactive lifecycle publisher is not an error, but it logs a warning EVERY
// time -- at 5 Hz, forever, for a node sitting in the unconfigured state. Detect
// the method and skip the publish instead.
template<typename P, typename = void>
struct HasIsActivated : std::false_type {};

template<typename P>
struct HasIsActivated<P, std::void_t<decltype(std::declval<const P&>().is_activated())>> : std::true_type {};

}   // namespace detail

// Templated on the node type so the workspace's one lifecycle node
// (gnss_driver) can use the same helper as everything else. Use the
// HeartbeatPublisher alias below unless you are on a LifecycleNode.
template<typename NodeT>
class BasicHeartbeatPublisher {
public:
  using PublisherT = decltype(std::declval<NodeT&>().template create_publisher<tractor_msgs::msg::Heartbeat>(std::declval<const char*>(), 10));

  // node must outlive this object (it is normally a member of the node).
  // rate_hz defaults to 5 Hz against tractor_monitor's 1.0 s default timeout,
  // i.e. five missed publishes before anyone calls it stale.
  BasicHeartbeatPublisher(NodeT* node, std::string node_name, double rate_hz = 5.0, const char* topic = kHeartbeatTopic) : node_(node), node_name_(std::move(node_name)) {
    pub_              = node_->template create_publisher<tractor_msgs::msg::Heartbeat>(topic, 10);
    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz, 0.1));
    timer_            = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {
      publish();
    });
  }

  void setOk() {
    set(tractor_msgs::msg::Heartbeat::STATUS_OK, "", kFaultSourceSelf);
  }
  // 下面三个接口的第二个参数 (故障来源) **必填** —— 见 Heartbeat.msg 的 fault_source_node:
  //   * 本节点自身         -> tractor_monitor::kFaultSourceSelf (空字符串);
  //   * 上游/依赖节点      -> 节点名 ("gnss_driver"、"local_path_planner_node");
  //   * 没有对应节点的来源 -> 来源标签 ("path_file" 外部工具写的 txt、"vcu"、"app")。
  // 刻意**不提供"只传 detail"的重载**: 故障状态会合并上报 (/monitor_heartbeat), APP 靠这个字段
  // 定位责任方 —— 漏填就变成"没人负责", 所以让编译期逼着每个调用点写清。
  void setStopImplement(const std::string& detail, const std::string& fault_source_node) {
    set(tractor_msgs::msg::Heartbeat::STATUS_STOP_IMPLEMENT, detail, fault_source_node);
  }
  // 设置农机停止运动状态 (STATUS_STOP_MOTION)。
  // detail 描述具体原因，如"no odometry"、"out of geofence"等。
  // fault_source_node 指定故障来源节点，参考上面的说明。
  void setStopMotion(const std::string& detail, const std::string& fault_source_node) {
    set(tractor_msgs::msg::Heartbeat::STATUS_STOP_MOTION, detail, fault_source_node);
  }
  void setFault(const std::string& detail, const std::string& fault_source_node) {
    set(tractor_msgs::msg::Heartbeat::STATUS_FAULT, detail, fault_source_node);
  }

  // 设置当前激活的故障码 (纯故障 ID, 无等级), 随心跳一并发布, 仅供 APP 展示。
  void setFaultCodes(const std::vector<uint32_t>& codes) {
    std::lock_guard<std::mutex> lock(mutex_);
    fault_codes_ = codes;
  }

  void set(uint8_t status, const std::string& detail, const std::string& fault_source_node) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_            = status;
    detail_            = detail;
    fault_source_node_ = (status == tractor_msgs::msg::Heartbeat::STATUS_OK) ? std::string(kFaultSourceSelf) : fault_source_node;
  }

  const std::string& name() const {
    return node_name_;
  }

private:
  void publish() {
    if constexpr (detail::HasIsActivated<typename PublisherT::element_type>::value) {
      // Lifecycle node not ACTIVE: it is not doing its job, so it must not
      // claim to be alive either.
      if (!pub_->is_activated()) {
        return;
      }
    }

    tractor_msgs::msg::Heartbeat msg;
    msg.header.stamp = node_->now();
    msg.node_name    = node_name_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.status            = status_;
      msg.detail            = detail_;
      msg.fault_codes       = fault_codes_;
      msg.fault_source_node = fault_source_node_;
    }
    msg.seq = seq_++;
    pub_->publish(msg);
  }

  NodeT* node_;
  std::string node_name_;
  PublisherT pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  // Starts FAULT (worst case), not OK: a node that constructs the publisher and
  // then wedges before it ever reports on itself must not look healthy for the
  // rest of the run.
  uint8_t status_     = tractor_msgs::msg::Heartbeat::STATUS_FAULT;
  std::string detail_ = "starting up";
  uint32_t seq_       = 0;
  std::vector<uint32_t> fault_codes_;
  // 当前这一帧要发出去的来源 (status = OK 时被清空): 见 Heartbeat.msg 的 fault_source_node。
  std::string fault_source_node_;
};

// The ordinary case. Every node in this workspace except gnss_driver.
using HeartbeatPublisher = BasicHeartbeatPublisher<rclcpp::Node>;

}   // namespace tractor_monitor
