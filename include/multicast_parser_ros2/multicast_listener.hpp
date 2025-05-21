#ifndef MULTICAST_LISTENER_HPP
#define MULTICAST_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>

namespace multicast_parser_ros2
{

class MulticastListener : public rclcpp::Node
{
public:
  explicit MulticastListener(const rclcpp::NodeOptions & options);
  virtual ~MulticastListener();

private:
  void setup_socket();
  void listen_thread_func();
  bool load_config();
  void shutdown_socket();

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr multicast_data_pub_;

  // Configuration parameters
  std::string multicast_group_;
  int multicast_port_;
  std::string server_address_;
  int buffer_size_;

  // Socket variables
  int sock_fd_{-1};
  struct ip_mreq mreq_;
  std::atomic<bool> running_{false};
  std::thread listen_thread_;
};

}  // namespace multicast_parser_ros2

#endif  // MULTICAST_LISTENER_HPP 