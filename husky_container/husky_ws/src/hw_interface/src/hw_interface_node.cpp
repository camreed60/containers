#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <hw_interface/hw_interface.hpp>

using namespace std::chrono_literals;

// Custom SIGINT handler to allow graceful shutdown
void mySigintHandler(int /*sig*/)
{
  // You could publish a stop message here if needed
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  // Set logger level (optional; defaults to INFO)
  rclcpp::Logger logger = rclcpp::get_logger("hw_interface_main");
  if (rclcpp::Logger::Level::Info <= rclcpp::get_logger_level(logger)) {
    rclcpp::set_logger_level(logger.get_name(), rclcpp::Logger::Level::Info);
  }

  RCLCPP_INFO(logger, "hw_interface Start");

  // Initialize ROS 2 without installing its default SIGINT handler
  rclcpp::init(argc, argv);

  // Create a Node for parameter handling and logging
  auto node = std::make_shared<rclcpp::Node>("hw_interface");

  // Override SIGINT so we can run custom code on shutdown
  std::signal(SIGINT, mySigintHandler);

  // Declare and read the 'node_type' parameter (if someone wants to override it)
  std::string node_type = "hw_interface";
  node->declare_parameter("node_type", node_type);
  node->get_parameter("node_type", node_type);

  RCLCPP_INFO(logger, "Node type: %s", node_type.c_str());

  // Instantiate your hw_interface object, passing in the Node
  auto hw_interface_ptr = std::make_unique<hw_interface>(node);

  RCLCPP_DEBUG(logger, "HW_Interface object created, entering spin loop");

  // Spin until SIGINT/shutdown
  rclcpp::spin(node);

  // Teardown
  hw_interface_ptr.reset();
  RCLCPP_DEBUG(logger, "HW_Interface Closing");
  return 0;
}
