#include "rclcpp/rclcpp.hpp"
#include "multicast_parser_ros2/data_parser.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  
  auto node = std::make_shared<multicast_parser_ros2::DataParser>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
} 