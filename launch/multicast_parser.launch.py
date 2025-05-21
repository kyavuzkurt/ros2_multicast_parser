from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('multicast_parser_ros2'),
            'config',
            'config.yaml'
        ]),
        description='Path to configuration file'
    )

    # Define nodes
    multicast_listener_node = Node(
        package='multicast_parser_ros2',
        executable='multicast_listener_node',
        name='multicast_listener',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    data_parser_node = Node(
        package='multicast_parser_ros2',
        executable='data_parser_node',
        name='data_parser',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    return LaunchDescription([
        config_file_arg,
        multicast_listener_node,
        data_parser_node
    ]) 