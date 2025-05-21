cmake_minimum_required(VERSION 3.10)
project(multicast_parser_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(include)

# Libraries
add_library(multicast_listener SHARED
  src/multicast_listener.cpp
)
ament_target_dependencies(multicast_listener
  rclcpp
  rclcpp_components
  std_msgs
  yaml-cpp
  ament_index_cpp
)
target_compile_definitions(multicast_listener PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_node(multicast_listener
  PLUGIN "multicast_parser_ros2::MulticastListener"
  EXECUTABLE multicast_listener_node
)

add_library(data_parser SHARED
  src/data_parser.cpp
)
ament_target_dependencies(data_parser
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
  ament_index_cpp
)
target_compile_definitions(data_parser PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_node(data_parser
  PLUGIN "multicast_parser_ros2::DataParser"
  EXECUTABLE data_parser_node
)

# Executables
add_executable(multicast_listener_exec src/multicast_listener_node.cpp)
target_link_libraries(multicast_listener_exec multicast_listener ${YAML_CPP_LIBRARIES})
ament_target_dependencies(multicast_listener_exec
  rclcpp
  yaml-cpp
)

add_executable(data_parser_exec src/data_parser_node.cpp)
target_link_libraries(data_parser_exec data_parser ${YAML_CPP_LIBRARIES})
ament_target_dependencies(data_parser_exec
  rclcpp
  yaml-cpp
)

# Install
install(TARGETS
  multicast_listener
  data_parser
  multicast_listener_exec
  data_parser_exec
  multicast_listener_node
  data_parser_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
