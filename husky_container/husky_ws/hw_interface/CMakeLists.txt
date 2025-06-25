cmake_minimum_required(VERSION 3.5)
project(hw_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_compile_options(-std=gnu++11)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Libraries
add_library(base_interface        src/base_interface.cpp)
ament_target_dependencies(base_interface
  pluginlib
  rclcpp
)
target_link_libraries(base_interface
  ${Boost_LIBRARIES}
)

add_library(base_serial_interface src/base_serial_interface.cpp)
ament_target_dependencies(base_serial_interface
  pluginlib
  rclcpp
)
target_link_libraries(base_serial_interface
  ${Boost_LIBRARIES}
)

add_library(base_UDP_interface    src/base_UDP_interface.cpp)
ament_target_dependencies(base_UDP_interface
  pluginlib
  rclcpp
)
target_link_libraries(base_UDP_interface
  ${Boost_LIBRARIES}
)

add_library(hw_interface          src/hw_interface.cpp)
ament_target_dependencies(hw_interface
  pluginlib
  rclcpp
)
target_link_libraries(hw_interface
  ${Boost_LIBRARIES}
)

# Executable
add_executable(hw_interface_node  src/hw_interface_node.cpp)
ament_target_dependencies(hw_interface_node
  pluginlib
  rclcpp
)
target_link_libraries(hw_interface_node
  hw_interface
  base_interface
  base_serial_interface
  base_UDP_interface
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install libraries & node
install(
  TARGETS
    base_interface
    base_serial_interface
    base_UDP_interface
    hw_interface
    hw_interface_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
