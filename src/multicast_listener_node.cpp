#include "rclcpp/rclcpp.hpp"
#include "multicast_parser_ros2/multicast_listener.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  
  auto node = std::make_shared<multicast_parser_ros2::MulticastListener>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 