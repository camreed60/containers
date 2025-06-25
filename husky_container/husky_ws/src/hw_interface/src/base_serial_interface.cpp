#include "hw_interface/base_serial_interface.hpp"

#include <boost/accumulators/statistics/mean.hpp>
#include <boost/scoped_ptr.hpp>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace base_classes
{

base_serial_interface::base_serial_interface()
: base_interface("base_serial_interface")  // assuming base_interface constructor takes node name
{
  RCLCPP_DEBUG(this->get_logger(), "Serial Plugin Initialized");
  interfaceType = Serial;
  interfaceStarted = false;

  enableCompletionFunctor = false;
  enableStreamMatcher     = false;

  enableMetrics();

  receivedData = boost::shared_array<uint8_t>(new uint8_t[500]);
  interfacePort.reset();
}

bool base_serial_interface::interfaceReady()
{
  return interfacePort && interfacePort->is_open();
}

bool base_serial_interface::initPlugin(
  const rclcpp::Node::SharedPtr & node,
  const boost::shared_ptr<boost::asio::io_context> ioService)
{
  rclcpp::Node::SharedPtr nh = node;
  enableCompletionFunctor = false;
  enableRegexReadUntil    = false;

  interfaceSynchronousStrand = std::make_shared<boost::asio::io_context::strand>(*ioService);
  interfacePort              = std::make_shared<boost::asio::serial_port>(*ioService);

  RCLCPP_DEBUG(nh->get_logger(), "Calling Plugin's Init");
  subPluginInit(nh);

  interfacePort->open(deviceName);
  setInterfaceOptions();

  return interfaceReady();
}

bool base_serial_interface::startWork()
{
  if (!interfaceReady()) {
    return false;
  }
  if (!interfaceStarted) {
    interfaceStarted = pluginStart();
  }

  if (interfaceStarted) {
    if (enableCompletionFunctor) {
      RCLCPP_DEBUG(this->get_logger(), "Async Read custom Functor");
      boost::asio::async_read(
        *interfacePort,
        boost::asio::buffer(receivedData.get(), MAX_SERIAL_READ),
        streamCompletionChecker,
        boost::bind(
          &base_serial_interface::handleIORequest, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred
        )
      );
    }
    else if (enableRegexReadUntil) {
      RCLCPP_DEBUG(this->get_logger(), "REGEX Async Read Until");
      boost::asio::async_read_until(
        *interfacePort, interfaceRegexBuffer, regexExpr,
        boost::bind(
          &base_serial_interface::handleRegexRequest, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred
        )
      );
    }
    else {
      RCLCPP_INFO_ONCE(this->get_logger(), "BASE INTERFACE_READ");
      boost::asio::async_read(
        *interfacePort,
        boost::asio::buffer(receivedData.get(), MAX_SERIAL_READ),
        boost::bind(
          &base_serial_interface::handleIORequest, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred
        )
      );
    }
  }
  return true;
}

bool base_serial_interface::stopWork()
{
  if (!interfaceStarted) {
    return false;
  }
  pluginStop();
  interfacePort->cancel();
  interfacePort->close();
  interfaceStarted = false;
  return !interfacePort->is_open();
}

bool base_serial_interface::handleRegexRequest(
  const boost::system::error_code & e,
  std::size_t bytesTransferred)
{
  printMetrics(true);
  RCLCPP_DEBUG(
    this->get_logger(),
    "Thread <%s>:: %s:: Received Packet!:: Size %zu",
    THREAD_ID_TO_C_STR, pluginName.c_str(), bytesTransferred
  );

  if (!e) {
    std::istream is(&interfaceRegexBuffer);
    std::getline(is, receivedRegexData);

    boost::smatch result;
    if (boost::regex_search(receivedRegexData, result, regexExpr)) {
      receivedRegexData = std::string(result[0].first, result[0].second);
    }
    RCLCPP_DEBUG(this->get_logger(), "Data -> %s", receivedRegexData.c_str());

    if (!interfaceReadHandler(bytesTransferred, dataArrayStart, e)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Error Occurred in data handler for plugin <%s>",
        pluginName.c_str()
      );
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error Occurred in plugin data handler: %s",
      e.message().c_str()
    );
  }

  // restart the work
  return startWork();
}

bool base_serial_interface::handleIORequest(
  const boost::system::error_code & ec,
  size_t bytesReceived)
{
  printMetrics(true);
  RCLCPP_DEBUG(
    this->get_logger(),
    "Thread <%s>:: %s:: Received Packet!:: Size %zu",
    THREAD_ID_TO_C_STR, pluginName.c_str(), bytesReceived
  );

  if (bytesReceived >= MAX_SERIAL_READ) {
    RCLCPP_WARN(
      this->get_logger(),
      "Buffer Filled! Falling behind or erroneous data. Skipping read."
    );
    return startWork();
  }

  if (!interfaceReadHandler(bytesReceived, dataArrayStart, ec)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error Occurred in plugin data Handler <%s>",
      pluginName.c_str()
    );
  }

  return startWork();
}

void base_serial_interface::postInterfaceWriteRequest(
  const hw_interface_support_types::shared_const_buffer & buffer)
{
  RCLCPP_DEBUG(this->get_logger(), "%s:: Requesting interface write", pluginName.c_str());
  interfaceSynchronousStrand->post(boost::bind(
    &base_serial_interface::interfaceWriteHandler, this, buffer
  ));
}

void base_serial_interface::interfaceWriteHandler(
  const hw_interface_support_types::shared_const_buffer & buffer)
{
  RCLCPP_DEBUG(this->get_logger(), "%s:: Writing Commands to interface", pluginName.c_str());
  try {
    boost::asio::write(*interfacePort, buffer);
  }
  catch (const boost::system::system_error & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Caught Exception on WRITE!! %s", ex.what()
    );
  }
  RCLCPP_DEBUG(this->get_logger(), "%s:: Writing done", pluginName.c_str());
}

}  // namespace base_classes
