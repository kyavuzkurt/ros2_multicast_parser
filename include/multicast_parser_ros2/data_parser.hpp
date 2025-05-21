#ifndef DATA_PARSER_HPP
#define DATA_PARSER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace multicast_parser_ros2
{

// Constants for binary data unpacking
constexpr int NAT_FRAMEOFDATA = 7;
constexpr int NAT_MODELDEF = 5;

struct RigidBody
{
  int id;
  float position_x;
  float position_y;
  float position_z;
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float orientation_w;
};

struct FrameData
{
  int frame_number;
  double timestamp;
  int rigid_body_count;
  std::vector<RigidBody> rigid_bodies;
};

class DataParser : public rclcpp::Node
{
public:
  explicit DataParser(const rclcpp::NodeOptions & options);
  virtual ~DataParser() = default;

private:
  void multicast_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  bool load_config();
  void process_binary_data(const std::vector<uint8_t> & data);
  std::pair<FrameData, size_t> unpack_frame_data(const std::vector<uint8_t> & data);
  void publish_frame_data(const FrameData & frame_data);
  std::string get_rigid_body_name(int rb_id);
  
  geometry_msgs::msg::Pose parse_pose(const RigidBody & rb);
  geometry_msgs::msg::Pose2D parse_pose2d(const RigidBody & rb);
  nav_msgs::msg::Odometry parse_odometry(const RigidBody & rb);

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr multicast_data_sub_;
  
  // Publishers (mapped by rigid body name)
  struct RigidBodyPublishers
  {
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  };
  std::map<std::string, RigidBodyPublishers> publishers_;
  
  // Configuration
  std::vector<std::string> rigid_body_names_;
  std::string log_level_;
};

}  // namespace multicast_parser_ros2

#endif  // DATA_PARSER_HPP 