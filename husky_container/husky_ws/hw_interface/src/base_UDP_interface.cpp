#include "hw_interface/base_udp_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <boost/bind/bind.hpp>
#include <boost/lexical_cast.hpp>

namespace base_classes
{

base_UDP_interface::base_UDP_interface()
: base_interface("base_udp_interface")
{
  RCLCPP_DEBUG(this->get_logger(), "UDP Plugin initialized");
  interfaceType    = UDP;
  interfaceStarted = false;

  receivedData     = boost::shared_array<uint8_t>(new uint8_t[UDP_MAX_PKT_SIZE]);
  remoteEndpoint.reset();
  localEndpoint .reset();
  interfaceSocket.reset();
}

bool base_UDP_interface::interfaceReady()
{
  return interfaceSocket && interfaceSocket->is_open();
}

bool base_UDP_interface::initPlugin(
  const rclcpp::Node::SharedPtr & node,
  const boost::shared_ptr<boost::asio::io_context> ioService)
{
  enableCompletionFunctor = false;
  enableRegexReadUntil    = false;

  interfaceSynchronousStrand =
    std::make_shared<boost::asio::io_context::strand>(*ioService);

  RCLCPP_DEBUG(this->get_logger(), "Calling Plugin's Init");
  subPluginInit(node);

  RCLCPP_DEBUG(this->get_logger(), "Creating Endpoints");
  remoteEndpoint = std::make_shared<boost::asio::ip::udp::endpoint>(remoteAddress, remotePort);
  localEndpoint  = std::make_shared<boost::asio::ip::udp::endpoint>(
                     boost::asio::ip::udp::v4(), localPort);

  RCLCPP_DEBUG(this->get_logger(), "Opening Local Socket");
  interfaceSocket =
    std::make_shared<boost::asio::ip::udp::socket>(*ioService, *localEndpoint);
  RCLCPP_DEBUG(this->get_logger(), "Socket Created");

  return interfaceReady();
}

bool base_UDP_interface::startWork()
{
  if (!interfaceReady()) {
    return false;
  }
  if (!interfaceStarted) {
    interfaceStarted = pluginStart();
    RCLCPP_INFO(this->get_logger(),
      "Setting Socket send size to %d", UDP_MAX_PKT_SIZE);
    boost::asio::socket_base::send_buffer_size option(UDP_MAX_PKT_SIZE);
    interfaceSocket->set_option(option);
  }
  RCLCPP_DEBUG(this->get_logger(), "Starting UDP Work");

  interfaceSocket->async_receive_from(
    boost::asio::buffer(receivedData.get(), UDP_MAX_PKT_SIZE),
    senderEndpoint,
    boost::bind(
      &base_UDP_interface::handleIORequest, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred
    )
  );
  return true;
}

bool base_UDP_interface::stopWork()
{
  if (interfaceStarted) {
    interfaceSocket->cancel();
    interfaceSocket->close();
    interfaceStarted = false;
    return !interfaceSocket->is_open();
  }
  return false;
}

bool base_UDP_interface::handleIORequest(
  const boost::system::error_code & ec,
  size_t bytesReceived)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), 5.0,
    "Thread <%s>:: %s:: Received Packet!:: Size %zu",
    THREAD_ID_TO_C_STR, pluginName.c_str(), bytesReceived
  );

  dataArrayStart = 0;
  if (!interfaceReadHandler(bytesReceived, dataArrayStart)) {
    RCLCPP_ERROR(this->get_logger(),
      "Error Occurred in plugin data Handler <%s>", pluginName.c_str());
  }

  // restart
  return startWork();
}

void base_UDP_interface::postInterfaceWriteRequest(
  const hw_interface_support_types::shared_const_buffer & buffer)
{
  RCLCPP_DEBUG(this->get_logger(), "%s:: Requesting interface write",
    pluginName.c_str());
  interfaceSynchronousStrand->post(
    boost::bind(&base_UDP_interface::interfaceWriteHandler, this, buffer)
  );
}

void base_UDP_interface::interfaceWriteHandler(
  const hw_interface_support_types::shared_const_buffer & buffer)
{
  RCLCPP_DEBUG(this->get_logger(), "%s:: Writing Commands to interface",
    pluginName.c_str());
  try {
    interfaceSocket->send_to(buffer, *remoteEndpoint);
  } catch (const boost::system::system_error & ex) {
    RCLCPP_ERROR(this->get_logger(),
      "%s:: Caught Exception on WRITE!! %s",
      pluginName.c_str(), ex.what());
  }
}

}  // namespace base_classes
