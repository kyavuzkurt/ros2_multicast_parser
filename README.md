# Multicast Parser ROS2

A ROS2 package for parsing multicast data from Motive Tracker 2.0.2 and publishing it to ROS2 topics. This package consists of two main nodes:

1. **MulticastListener**: Listens to a multicast server from Motive Tracker 2.0.2 and publishes the raw data.
2. **DataParser**: Parses the raw data into meaningful ROS2 messages and publishes them.

## Features

- Receives multicast data from Motive Tracker 2.0.2
- Parses rigid body tracking data
- Publishes to the following topics for each rigid body:
  - `/multicast_parser/<rigid_body_name>/pose` (geometry_msgs/msg/Pose)
  - `/multicast_parser/<rigid_body_name>/ground_pose` (geometry_msgs/msg/Pose2D)
  - `/multicast_parser/<rigid_body_name>/odom` (nav_msgs/msg/Odometry)

## Installation

### Prerequisites

- ROS2 (tested on Foxy/Humble)
- yaml-cpp library

### Building from Source

```bash
# Create a ROS2 workspace (if you don't have one already)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/yourusername/multicast_parser_ros2.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select multicast_parser_ros2

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Configuration

Edit the `config/config.yaml` file to configure the package:

- `multicast_group`: The multicast group to listen to (default: '224.0.0.1')
- `multicast_port`: The multicast port to listen to (default: 9000)
- `server_address`: The server address to listen to (IP address of the machine running Motive Tracker 2.0.2)
- `buffer_size`: The buffer size for the multicast listener (default: 4096)
- `rigid_body_names`: The names of the rigid bodies to parse data for (default: ['Robot_1'])
- `log_level`: The log level for the nodes (DEBUG, INFO)

## Usage

Launch the package using the provided launch file:

```bash
ros2 launch multicast_parser_ros2 multicast_parser.launch.py
```

You can also specify a custom configuration file:

```bash
ros2 launch multicast_parser_ros2 multicast_parser.launch.py config_file:=/path/to/your/config.yaml
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

This package is a ROS2 C++ port of the original [multicast_parser](https://github.com/kyavuzkurt/multicast_parser) ROS1 Python package by [Kadir Yavuz Kurt](https://github.com/kyavuzkurt). 