#include "multicast_parser_ros2/multicast_listener.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include <fstream>
#include <vector>

namespace multicast_parser_ros2
{

MulticastListener::MulticastListener(const rclcpp::NodeOptions & options)
: Node("multicast_listener", options)
{
  // Declare parameters
  this->declare_parameter("config_file", "");
  this->declare_parameter("multicast_group", "224.0.0.1");
  this->declare_parameter("multicast_port", 9000);
  this->declare_parameter("server_address", "192.168.0.101");
  this->declare_parameter("buffer_size", 4096);
  
  // Create publisher
  multicast_data_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
    "multicast_data", 10);
  
  if (!load_config()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load configuration. Shutting down node.");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "Multicast listener initialized with: Multicast group: %s, Port: %d, Server address: %s", 
    multicast_group_.c_str(), multicast_port_, server_address_.c_str());
  
  setup_socket();
  
  // Start listening thread
  running_ = true;
  listen_thread_ = std::thread(&MulticastListener::listen_thread_func, this);
}

MulticastListener::~MulticastListener()
{
  running_ = false;
  if (listen_thread_.joinable()) {
    listen_thread_.join();
  }
  shutdown_socket();
}

bool MulticastListener::load_config()
{
  std::string config_file = this->get_parameter("config_file").as_string();
  
  if (!config_file.empty()) {
    try {
      YAML::Node config = YAML::LoadFile(config_file);
      
      if (config["/**"]["ros__parameters"]["multicast_group"]) {
        multicast_group_ = config["/**"]["ros__parameters"]["multicast_group"].as<std::string>();
        this->set_parameter(rclcpp::Parameter("multicast_group", multicast_group_));
      }
      
      if (config["/**"]["ros__parameters"]["multicast_port"]) {
        multicast_port_ = config["/**"]["ros__parameters"]["multicast_port"].as<int>();
        this->set_parameter(rclcpp::Parameter("multicast_port", multicast_port_));
      }
      
      if (config["/**"]["ros__parameters"]["server_address"]) {
        server_address_ = config["/**"]["ros__parameters"]["server_address"].as<std::string>();
        this->set_parameter(rclcpp::Parameter("server_address", server_address_));
      }
      
      if (config["/**"]["ros__parameters"]["buffer_size"]) {
        buffer_size_ = config["/**"]["ros__parameters"]["buffer_size"].as<int>();
        this->set_parameter(rclcpp::Parameter("buffer_size", buffer_size_));
      }
      
      RCLCPP_INFO(this->get_logger(), "Configuration loaded from %s", config_file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file: %s", e.what());
      RCLCPP_INFO(this->get_logger(), "Using node parameters instead.");
    }
  }
  
  // Get parameters from node (these may have been set from the file or from command line)
  multicast_group_ = this->get_parameter("multicast_group").as_string();
  multicast_port_ = this->get_parameter("multicast_port").as_int();
  server_address_ = this->get_parameter("server_address").as_string();
  buffer_size_ = this->get_parameter("buffer_size").as_int();
  
  if (multicast_group_.empty() || multicast_port_ <= 0 || server_address_.empty() || buffer_size_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid configuration parameters.");
    return false;
  }
  
  return true;
}

void MulticastListener::setup_socket()
{
  try {
    // Create UDP socket
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
      return;
    }
    
    // Enable address reuse
    int reuse = 1;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
      return;
    }
    
    // Bind to port
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(multicast_port_);
    
    if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind socket: %s", strerror(errno));
      return;
    }
    
    // Join multicast group
    memset(&mreq_, 0, sizeof(mreq_));
    mreq_.imr_multiaddr.s_addr = inet_addr(multicast_group_.c_str());
    mreq_.imr_interface.s_addr = htonl(INADDR_ANY);
    
    if (setsockopt(sock_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq_, sizeof(mreq_)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to join multicast group: %s", strerror(errno));
      return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
      "Listening for multicast packets on %s:%d...", 
      multicast_group_.c_str(), multicast_port_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Socket setup failed: %s", e.what());
  }
}

void MulticastListener::listen_thread_func()
{
  std::vector<uint8_t> buffer(buffer_size_);
  struct sockaddr_in sender_addr;
  socklen_t sender_addr_len = sizeof(sender_addr);
  
  while (running_) {
    // Receive data
    ssize_t bytes_received = recvfrom(
      sock_fd_, 
      buffer.data(), 
      buffer.size(), 
      0, 
      reinterpret_cast<struct sockaddr*>(&sender_addr), 
      &sender_addr_len);
    
    if (bytes_received < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive data: %s", strerror(errno));
      }
      continue;
    }
    
    // Convert sender address to string
    char sender_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);
    
    RCLCPP_DEBUG(this->get_logger(), "Received %zd bytes from %s", bytes_received, sender_ip);
    
    // Check if the sender is our server
    if (std::string(sender_ip) == server_address_) {
      auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
      msg->data.assign(buffer.begin(), buffer.begin() + bytes_received);
      
      // Publish the data
      multicast_data_pub_->publish(std::move(msg));
      RCLCPP_DEBUG(this->get_logger(), "Published data: %zd bytes", bytes_received);
    }
  }
}

void MulticastListener::shutdown_socket()
{
  if (sock_fd_ >= 0) {
    // Leave the multicast group
    setsockopt(sock_fd_, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq_, sizeof(mreq_));
    close(sock_fd_);
    sock_fd_ = -1;
    RCLCPP_INFO(this->get_logger(), "Multicast listener stopped.");
  }
}

}  // namespace multicast_parser_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(multicast_parser_ros2::MulticastListener) 