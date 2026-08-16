#include <rclcpp/rclcpp.hpp>

#include "usv_radio_bridge/radio_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<usv_radio_bridge::RadioBridgeNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
