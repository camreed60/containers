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

#ifndef BASE_UDP_INTERFACE_HPP__
#define BASE_UDP_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>

#include "base_interface.hpp"

#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#define UDP_FRAME_BUFFER_SIZE 1500
#define UDP_MAX_PKT_SIZE     65500

namespace base_classes
{
  class base_UDP_interface : public base_interface
  {
  public:
    bool handleIORequest(
      const boost::system::error_code & ec,
      size_t bytesReceived) override;

  protected:
    base_UDP_interface();

    boost::shared_ptr<boost::asio::ip::udp::socket>   interfaceSocket;
    boost::shared_ptr<boost::asio::ip::udp::endpoint> localEndpoint;
    boost::shared_ptr<boost::asio::ip::udp::endpoint> remoteEndpoint;
    boost::asio::ip::udp::endpoint                     senderEndpoint;

    // Called by base_interface after read completes
    // Plugins implement this to process incoming data
    virtual bool interfaceReadHandler(
      const size_t & bufferSize,
      int arrayStartPos) = 0;

    // Called to allow plugin-specific ROS parameter loading / subscriptions
    virtual bool subPluginInit(
      const rclcpp::Node::SharedPtr & node) = 0;

    // Optional plugin hooks
    virtual bool pluginStart() override { return true; }
    virtual bool pluginStop()  override { return true; }

    // Socket configuration
    boost::asio::ip::address localAddress;
    int                           localPort{0};
    boost::asio::ip::address remoteAddress;
    int                           remotePort{0};

    // Write callbacks
    void interfaceWriteHandler(
      const hw_interface_support_types::shared_const_buffer & buffer) override;
    void postInterfaceWriteRequest(
      const hw_interface_support_types::shared_const_buffer & buffer) override;

  private:
    bool interfaceReady() override;
    bool initPlugin(
      const rclcpp::Node::SharedPtr & node,
      const boost::shared_ptr<boost::asio::io_context> ioContext) override;
    bool startWork() override;
    bool stopWork() override;
  };
}  // namespace base_classes

#endif  // BASE_UDP_INTERFACE_HPP__
