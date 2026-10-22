#include "crop_row_perception/observer_node.hpp"
#include <cstdio>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<crop_row_perception::ObserverNode>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "crop_row_perception: %s\n", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
