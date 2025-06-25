#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "imu_complementary_filter/complementary_filter_ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the filter node using default NodeOptions (reads parameters, sets up subscriptions)
  auto filter_node = std::make_shared<imu_tools::ComplementaryFilterROS>(rclcpp::NodeOptions());

  rclcpp::spin(filter_node);
  rclcpp::shutdown();
  return 0;
}
