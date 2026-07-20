#include "mission_manager_node.hpp"
#include <rclcpp/rclcpp.hpp>

using open_mower_next::mission_manager::MissionManagerNode;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<MissionManagerNode>(options);

  RCLCPP_INFO(node->get_logger(), "Starting mission manager node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}