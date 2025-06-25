/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* ...
*********************************************************************/

#ifndef HW_INTERFACE_HPP__
#define HW_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind/bind.hpp>

#define NUM_THREADS_PER_PLUGIN 2

namespace interface_worker
{
  // Worker function for spinning the ASIO io_context in threads
  void worker(boost::shared_ptr<boost::asio::io_context> io_context);
}

class hw_interface
{
public:
  explicit hw_interface(const rclcpp::Node::SharedPtr & node);
  virtual ~hw_interface();

  // Load plugins via pluginlib
  void addInterfacePlugins();

  // Start/stop all interfaces
  bool startInterfaces();
  bool stopInterfaces();

private:
  // ROS 2 node handle
  rclcpp::Node::SharedPtr node_;

  // ASIO service and its work guard
  boost::shared_ptr<boost::asio::io_context>      interfaceService_;
  boost::shared_ptr<boost::asio::io_context::work> interfaceWork_;

  // Loaded interface plugins
  std::vector<boost::shared_ptr<base_classes::base_interface>> interfacePluginVector_;

  // Thread pool for ASIO
  boost::thread_group interfaceWorkerGroup_;

  // Plugin loader for base_interface implementations
  pluginlib::ClassLoader<base_classes::base_interface> pluginLoader_;

  // Initialize a single plugin instance
  bool initPlugin(
    const boost::shared_ptr<base_classes::base_interface> & plugin,
    const std::string & plugin_name);

  // Set up all plugins and thread pool
  void initInterfacePlugins();
  void initThreadPool();
};

#endif  // HW_INTERFACE_HPP__
