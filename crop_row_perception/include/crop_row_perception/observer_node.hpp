#ifndef CROP_ROW_PERCEPTION__OBSERVER_NODE_HPP_
#define CROP_ROW_PERCEPTION__OBSERVER_NODE_HPP_
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace crop_row_perception
{
class ObserverNode : public rclcpp::Node
{
public:
  explicit ObserverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ObserverNode() override;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
#endif
