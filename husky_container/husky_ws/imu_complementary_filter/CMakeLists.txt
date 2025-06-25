cmake_minimum_required(VERSION 3.5)
project(imu_complementary_filter)

# find ament build system and ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# include headers
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# core library
add_library(complementary_filter
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
)
ament_target_dependencies(complementary_filter
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  message_filters
  Boost
)

# node executable
add_executable(complementary_filter_node
  src/complementary_filter_node.cpp
)
target_link_libraries(complementary_filter_node
  complementary_filter
)
ament_target_dependencies(complementary_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  message_filters
)

# install targets
install(
  TARGETS complementary_filter complementary_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  message_filters
  Boost
)

ament_package()
