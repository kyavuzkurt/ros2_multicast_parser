#include "multicast_parser_ros2/data_parser.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include <cmath>
#include <fstream>
#include <cstring>

namespace multicast_parser_ros2
{

DataParser::DataParser(const rclcpp::NodeOptions & options)
: Node("data_parser", options)
{
  // Declare parameters
  this->declare_parameter("config_file", "");
  this->declare_parameter("rigid_body_names", std::vector<std::string>{"Robot_1"});
  this->declare_parameter("log_level", "INFO");
  
  // Load configuration
  if (!load_config()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load configuration. Shutting down node.");
    return;
  }
  
  // Create subscription to multicast data
  multicast_data_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "multicast_data", 10, 
    std::bind(&DataParser::multicast_data_callback, this, std::placeholders::_1));
  
  // Create publishers for each rigid body
  for (const auto & rb_name : rigid_body_names_) {
    RigidBodyPublishers publishers;
    publishers.pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(
      "multicast_parser/" + rb_name + "/pose", 10);
    publishers.pose2d_pub = this->create_publisher<geometry_msgs::msg::Pose2D>(
      "multicast_parser/" + rb_name + "/ground_pose", 10);
    publishers.odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
      "multicast_parser/" + rb_name + "/odom", 10);
    
    publishers_[rb_name] = publishers;
  }
  
  // Construct detailed topic information for logging
  std::string topics_info;
  for (const auto & rb_name : rigid_body_names_) {
    topics_info += "- Rigid Body: " + rb_name + "\n" +
      "  - Pose Topic: multicast_parser/" + rb_name + "/pose\n" +
      "  - Ground Pose Topic: multicast_parser/" + rb_name + "/ground_pose\n" +
      "  - Odometry Topic: multicast_parser/" + rb_name + "/odom\n";
  }
  
  RCLCPP_INFO(this->get_logger(),
    "Multicast Data Parser Node Initialized.\n"
    "Publishing rigid bodies: [%s] to the following topics:\n%s",
    [this]() {
      std::stringstream ss;
      for (size_t i = 0; i < rigid_body_names_.size(); ++i) {
        ss << rigid_body_names_[i];
        if (i < rigid_body_names_.size() - 1) {
          ss << ", ";
        }
      }
      return ss.str();
    }().c_str(),
    topics_info.c_str());
}

bool DataParser::load_config()
{
  std::string config_file = this->get_parameter("config_file").as_string();
  
  if (!config_file.empty()) {
    try {
      YAML::Node config = YAML::LoadFile(config_file);
      
      if (config["/**"]["ros__parameters"]["rigid_body_names"]) {
        rigid_body_names_ = config["/**"]["ros__parameters"]["rigid_body_names"].as<std::vector<std::string>>();
        this->set_parameter(rclcpp::Parameter("rigid_body_names", rigid_body_names_));
      }
      
      if (config["/**"]["ros__parameters"]["log_level"]) {
        log_level_ = config["/**"]["ros__parameters"]["log_level"].as<std::string>();
        this->set_parameter(rclcpp::Parameter("log_level", log_level_));
      }
      
      RCLCPP_INFO(this->get_logger(), "Configuration loaded from %s", config_file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file: %s", e.what());
      RCLCPP_INFO(this->get_logger(), "Using node parameters instead.");
    }
  }
  
  // Get parameters from node (these may have been set from the file or from command line)
  rigid_body_names_ = this->get_parameter("rigid_body_names").as_string_array();
  log_level_ = this->get_parameter("log_level").as_string();
  
  if (rigid_body_names_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No rigid body names provided. Using default 'Robot_1'");
    rigid_body_names_ = {"Robot_1"};
  }
  
  return true;
}

void DataParser::multicast_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  try {
    RCLCPP_DEBUG(this->get_logger(), "Received multicast data of size %zu", msg->data.size());
    
    // Process the binary data
    process_binary_data(msg->data);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing multicast data: %s", e.what());
  }
}

void DataParser::process_binary_data(const std::vector<uint8_t> & data)
{
  try {
    if (data.size() < 4) {
      RCLCPP_WARN(this->get_logger(), "Received data is too short to contain message ID and packet size.");
      return;
    }
    
    // Extract Message ID (2 bytes)
    int16_t message_id = *reinterpret_cast<const int16_t*>(data.data());
    
    // Extract Packet Size (2 bytes)
    int16_t packet_size = *reinterpret_cast<const int16_t*>(data.data() + 2);
    
    RCLCPP_DEBUG(this->get_logger(), "Message ID: %d, Packet Size: %d bytes", message_id, packet_size);
    
    // Check if the received data matches the expected packet size
    size_t actual_payload_size = data.size() - 4;  // Subtract header size
    if (actual_payload_size < static_cast<size_t>(packet_size)) {
      RCLCPP_WARN(this->get_logger(), 
        "Insufficient data: expected %d bytes, received %zu bytes. Skipping this packet.",
        packet_size, actual_payload_size);
      return;
    } else if (actual_payload_size > static_cast<size_t>(packet_size)) {
      RCLCPP_WARN(this->get_logger(), 
        "Received more data than expected: expected %d bytes, received %zu bytes. Truncating extra bytes.",
        packet_size, actual_payload_size);
    }
    
    // Handle the message based on its ID
    if (message_id == NAT_FRAMEOFDATA) {
      // Create a view of the packet data (excluding the header)
      std::vector<uint8_t> packet_data(data.begin() + 4, data.begin() + 4 + packet_size);
      
      // Unpack the frame data
      auto [frame_data, _] = unpack_frame_data(packet_data);
      
      // Publish the frame data
      publish_frame_data(frame_data);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unrecognized or unhandled Message ID: %d", message_id);
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process binary data: %s", e.what());
  }
}

std::pair<FrameData, size_t> DataParser::unpack_frame_data(const std::vector<uint8_t> & data)
{
  size_t offset = 0;
  FrameData frame_data;
  
  try {
    // Ensure there's enough data for frame number, timestamp, and rigid_body_count
    if (data.size() < offset + 4 + 8 + 4) {
      RCLCPP_WARN(this->get_logger(), "Insufficient data for frame_number, timestamp, and rigid_body_count.");
      return {frame_data, offset};
    }
    
    // Unpack frame number (4 bytes)
    frame_data.frame_number = *reinterpret_cast<const int32_t*>(data.data() + offset);
    offset += 4;
    
    // Unpack timestamp (8 bytes)
    frame_data.timestamp = *reinterpret_cast<const double*>(data.data() + offset);
    offset += 8;
    
    // Unpack rigid body count (4 bytes)
    frame_data.rigid_body_count = *reinterpret_cast<const int32_t*>(data.data() + offset);
    offset += 4;
    
    RCLCPP_DEBUG(this->get_logger(), 
      "Frame Number: %d, Timestamp: %.6f, Rigid Body Count: %d",
      frame_data.frame_number, frame_data.timestamp, frame_data.rigid_body_count);
    
    // Iterate through each rigid body
    for (int i = 0; i < frame_data.rigid_body_count; ++i) {
      size_t expected_length = offset + 4 + 12 + 16;  // id + position + orientation
      if (data.size() < expected_length) {
        RCLCPP_WARN(this->get_logger(), 
          "Insufficient data for rigid body %d. Expected %zu bytes, got %zu bytes.",
          i + 1, expected_length, data.size());
        break;
      }
      
      RigidBody rigid_body;
      
      // Unpack rigid body ID (4 bytes)
      rigid_body.id = *reinterpret_cast<const int32_t*>(data.data() + offset);
      offset += 4;
      
      // Unpack position (12 bytes: 3 floats for x, y, z)
      const float* pos = reinterpret_cast<const float*>(data.data() + offset);
      rigid_body.position_x = pos[0];
      rigid_body.position_y = pos[1];
      rigid_body.position_z = pos[2];
      offset += 12;
      
      // Unpack orientation (16 bytes: 4 floats for x, y, z, w)
      const float* rot = reinterpret_cast<const float*>(data.data() + offset);
      rigid_body.orientation_x = rot[0];
      rigid_body.orientation_y = rot[1];
      rigid_body.orientation_z = rot[2];
      rigid_body.orientation_w = rot[3];
      offset += 16;
      
      RCLCPP_DEBUG(this->get_logger(), 
        "Rigid Body %d: ID=%d, Position=[%.2f, %.2f, %.2f], Orientation=[%.2f, %.2f, %.2f, %.2f]",
        i + 1, rigid_body.id, 
        rigid_body.position_x, rigid_body.position_y, rigid_body.position_z,
        rigid_body.orientation_x, rigid_body.orientation_y, rigid_body.orientation_z, rigid_body.orientation_w);
      
      frame_data.rigid_bodies.push_back(rigid_body);
    }
    
    return {frame_data, offset};
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error unpacking frame data: %s", e.what());
    return {frame_data, offset};
  }
}

void DataParser::publish_frame_data(const FrameData & frame_data)
{
  if (frame_data.rigid_bodies.empty()) {
    RCLCPP_WARN(this->get_logger(), "No rigid bodies found in frame data.");
    return;
  }
  
  for (const auto & rb : frame_data.rigid_bodies) {
    std::string rb_name = get_rigid_body_name(rb.id);
    if (rb_name.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rigid body ID %d not found in configuration.", rb.id);
      continue;
    }
    
    // Find publishers for this rigid body
    auto it = publishers_.find(rb_name);
    if (it == publishers_.end()) {
      RCLCPP_WARN(this->get_logger(), "No publishers found for rigid body: %s", rb_name.c_str());
      continue;
    }
    
    // Publish Pose
    auto pose_msg = parse_pose(rb);
    it->second.pose_pub->publish(pose_msg);
    
    // Publish Pose2D (Ground Pose)
    auto pose2d_msg = parse_pose2d(rb);
    it->second.pose2d_pub->publish(pose2d_msg);
    
    // Publish Odometry
    auto odom_msg = parse_odometry(rb);
    it->second.odom_pub->publish(odom_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published data for rigid body: %s", rb_name.c_str());
  }
}

std::string DataParser::get_rigid_body_name(int rb_id)
{
  if (1 <= rb_id && rb_id <= static_cast<int>(rigid_body_names_.size())) {
    return rigid_body_names_[rb_id - 1];  // Assuming IDs start at 1
  } else {
    RCLCPP_WARN(this->get_logger(), "Rigid body ID %d is out of range.", rb_id);
    return "";
  }
}

geometry_msgs::msg::Pose DataParser::parse_pose(const RigidBody & rb)
{
  geometry_msgs::msg::Pose pose_msg;
  
  // Assign positions
  pose_msg.position.x = rb.position_x;
  pose_msg.position.y = rb.position_y;
  pose_msg.position.z = rb.position_z;
  
  // Assign orientations
  pose_msg.orientation.x = rb.orientation_x;
  pose_msg.orientation.y = rb.orientation_y;
  pose_msg.orientation.z = rb.orientation_z;
  pose_msg.orientation.w = rb.orientation_w;
  
  return pose_msg;
}

geometry_msgs::msg::Pose2D DataParser::parse_pose2d(const RigidBody & rb)
{
  geometry_msgs::msg::Pose2D pose2d_msg;
  
  pose2d_msg.x = rb.position_x;
  pose2d_msg.y = rb.position_y;
  
  // Calculate theta from orientation quaternion (assuming yaw is the rotation around Z-axis)
  float siny_cosp = 2.0f * (rb.orientation_w * rb.orientation_z + rb.orientation_x * rb.orientation_y);
  float cosy_cosp = 1.0f - 2.0f * (rb.orientation_y * rb.orientation_y + rb.orientation_z * rb.orientation_z);
  pose2d_msg.theta = std::atan2(siny_cosp, cosy_cosp);
  
  return pose2d_msg;
}

nav_msgs::msg::Odometry DataParser::parse_odometry(const RigidBody & rb)
{
  nav_msgs::msg::Odometry odom_msg;
  
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom";
  
  // Assign pose
  odom_msg.pose.pose.position.x = rb.position_x;
  odom_msg.pose.pose.position.y = rb.position_y;
  odom_msg.pose.pose.position.z = rb.position_z;
  
  odom_msg.pose.pose.orientation.x = rb.orientation_x;
  odom_msg.pose.pose.orientation.y = rb.orientation_y;
  odom_msg.pose.pose.orientation.z = rb.orientation_z;
  odom_msg.pose.pose.orientation.w = rb.orientation_w;
  
  // Assign twist as zero (since velocity data isn't provided)
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;
  
  return odom_msg;
}

}  // namespace multicast_parser_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(multicast_parser_ros2::DataParser) 