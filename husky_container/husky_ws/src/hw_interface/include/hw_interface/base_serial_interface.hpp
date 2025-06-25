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
*  ...
*********************************************************************/

#ifndef BASE_SERIAL_INTERFACE_HPP__
#define BASE_SERIAL_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include "base_interface.hpp"

#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/regex.hpp>

#define MAX_SERIAL_READ 250

namespace base_classes
{
  class base_serial_interface : public base_interface
  {
  public:
    bool handleIORequest(
      const boost::system::error_code & ec,
      size_t bytesReceived) override;

    // handles ASCII Roboteq runtime queries
    bool handleRegexRequest(
      const boost::system::error_code & e,
      std::size_t bytesTransferred);

    boost::regex                   regexExpr;
    std::string                    receivedRegexData;

    // Plugins can override this to match footer sequences
    virtual std::pair<MatcherIterator,bool>
    matchFooter(
      MatcherIterator begin,
      MatcherIterator end,
      const char* sequence)
    {
      return std::make_pair(end, true);
    }

  private:
    bool interfaceReady() override;
    bool initPlugin(
      const rclcpp::Node::SharedPtr & node,
      const boost::shared_ptr<boost::asio::io_context> ioContext) override;
    bool startWork() override;
    bool stopWork() override;

    // Called by base_interface after read completes
    // Plugins should implement this callback
    virtual bool interfaceReadHandler(
      const size_t & bufferSize,
      int arrayStartPos,
      const boost::system::error_code & ec) = 0;

    // Called to allow plugin-specific ROS parameter loading / subscriptions
    virtual bool subPluginInit(const rclcpp::Node::SharedPtr & node) = 0;
    virtual void setInterfaceOptions() = 0;

  protected:
    boost::shared_ptr<boost::asio::serial_port> interfacePort;
    std::string                                  deviceName;
    boost::asio::streambuf                       interfaceRegexBuffer;
    int                                          readLength;
    std::string                                  headerString, footerString;

    base_serial_interface();

    // Called by base_interface when writing
    void interfaceWriteHandler(
      const hw_interface_support_types::shared_const_buffer & buffer) override;
    void postInterfaceWriteRequest(
      const hw_interface_support_types::shared_const_buffer & buffer) override;

    // Optional plugin start/stop hooks
    virtual bool pluginStart() { return true; }
    virtual bool pluginStop()  { return true; }

    template<typename Option>
    boost::system::error_code setOption(const Option * newOption)
    {
      boost::system::error_code ec;
      if (interfacePort) {
        interfacePort->set_option<Option>(*newOption, ec);
      } else {
        ec.assign(
          static_cast<int>(boost::system::errc::no_such_device),
          boost::system::generic_category());
      }
      return ec;
    }

    template<typename Option>
    Option getOption()
    {
      Option returnValue;
      if (interfacePort) {
        interfacePort->get_option<Option>(returnValue);
      }
      return returnValue;
    }
  };
}  // namespace base_classes

#endif  // BASE_SERIAL_INTERFACE_HPP__
