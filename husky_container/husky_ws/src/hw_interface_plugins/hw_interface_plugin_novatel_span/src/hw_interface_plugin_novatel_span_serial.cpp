#include "hw_interface_plugin_novatel_span/novatel_span_serial.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>

namespace hw_interface_plugin_novatel_span
{

novatel_span_serial::novatel_span_serial()
: base_serial_interface()
{
  // Informative startup message
  RCLCPP_INFO(rclcpp::get_logger("novatel_span_serial"),
              "A Wild Novatel Span Plugin Appeared!");

  // Enable periodic metric collection
  enableMetrics();

  // Pre-compute scale factors
  gyroScaleFactor  = DEG2RAD * IMU_RATE * std::pow(2.0, -21.0);
  accelScaleFactor = IMU_RATE * std::pow(2.0, -22.0);

  // Indicate “no orientation” in the covariance
  imuMessage.orientation_covariance[0] = -1;
}

novatel_span_serial::~novatel_span_serial()
{
  if (recordGPSData && gpsDataFile.is_open()) {
    gpsDataFile.close();
  }
}

bool novatel_span_serial::subPluginInit(
  const rclcpp::Node::SharedPtr & node)
{
  auto & logger = node->get_logger();
  RCLCPP_DEBUG(logger, "%s Plugin Init", pluginName.c_str());

  // Retrieve the serial device name
  if (!node->get_parameter(pluginName + ".deviceName", deviceName)) {
    RCLCPP_ERROR(logger, "%s:: No deviceName param", pluginName.c_str());
    return false;
  }

  // Configure the packet matcher
  streamCompletionChecker =
    boost::bind(&novatel_span_serial::novatelSpanStreamMatcher,
                this, _1, _2);
  enableCompletionFunctor = !streamCompletionChecker.empty();

  // Set up IMU publisher
  {
    std::string topic;
    if (!node->get_parameter(pluginName + ".publishToTopic", topic)) {
      RCLCPP_ERROR(logger,
                   "%s:: Could not find publishToTopic param",
                   pluginName.c_str());
      return false;
    }
    imuPublisher =
      node->create_publisher<sensor_msgs::msg::Imu>(topic, 1);
  }

  // Handle optional GPS logging
  node->get_parameter(pluginName + ".recordGPSData", recordGPSData);
  if (recordGPSData) {
    std::string fname;
    if (!node->get_parameter(pluginName + ".gpsDataFileName", fname)) {
      RCLCPP_ERROR(logger,
                   "%s:: No gpsDataFileName param",
                   pluginName.c_str());
      return false;
    }
    gpsDataFile.open(fname,
                     std::ios::out | std::ios::trunc | std::ios::binary);
    gpsTimePub =
      node->create_publisher<hw_interface_plugin_novatel_span::msg::NovatelGPSTime>(
        "/gps_time", 1);
  }

  return true;
}

void novatel_span_serial::setInterfaceOptions()
{
  // Configure standard 115200-8-N-1 serial
  int baud = 115200;
  this->get_parameter(pluginName + ".baudrate", baud);

  setOption<boost::asio::serial_port_base::baud_rate>(
    new boost::asio::serial_port_base::baud_rate(baud));
  setOption<boost::asio::serial_port_base::character_size>(
    new boost::asio::serial_port_base::character_size(8));
  setOption<boost::asio::serial_port_base::flow_control>(
    new boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
  setOption<boost::asio::serial_port_base::parity>(
    new boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
  setOption<boost::asio::serial_port_base::stop_bits>(
    new boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));

  RCLCPP_INFO(rclcpp::get_logger("novatel_span_serial"),
              "%s :: Device: %s :: Baudrate %d",
              pluginName.c_str(), deviceName.c_str(), baud);
}

bool novatel_span_serial::interfaceReadHandler(
  const size_t & length,
  int arrayStartPos,
  const boost::system::error_code & ec)
{
  // Verify CRC32 (optional)
  unsigned long packetCRC =
    *reinterpret_cast<unsigned long*>(&receivedData[arrayStartPos + fullPacketLen - 4]);
  unsigned long calcCRC = CalculateBlockCRC32_(
    fullPacketLen - 4,
    &receivedData[arrayStartPos]);

  if (packetCRC != calcCRC) {
    RCLCPP_WARN(rclcpp::get_logger("novatel_span_serial"),
                "Checksum failed!");
    return true;
  }

  // Message ID at bytes [arrayStartPos+4..+5]
  unsigned short msgId = 
    static_cast<unsigned short>(receivedData[arrayStartPos+4]) |
    (static_cast<unsigned short>(receivedData[arrayStartPos+5]) << 8);

  // RAWIMUS = 325
  if (msgId == 325) {
    // Parse accelerations
    auto get16 = [&](int offset) {
      return static_cast<uint32_t>(
        (receivedData[arrayStartPos+offset] & 0xFF) |
        ((receivedData[arrayStartPos+offset+1] & 0xFF) << 8) |
        ((receivedData[arrayStartPos+offset+2] & 0xFF) << 16) |
        ((receivedData[arrayStartPos+offset+3] & 0xFF) << 24));
    };

    int base = arrayStartPos + headerLen;
    imuMessage.linear_acceleration.z =
      accelScaleFactor * static_cast<double>(get16(base + 16));
    imuMessage.linear_acceleration.y =
      -accelScaleFactor * static_cast<double>(get16(base + 20));
    imuMessage.linear_acceleration.x =
      accelScaleFactor * static_cast<double>(get16(base + 24));

    imuMessage.angular_velocity.z =
      gyroScaleFactor * static_cast<double>(get16(base + 28));
    imuMessage.angular_velocity.y =
      -gyroScaleFactor * static_cast<double>(get16(base + 32));
    imuMessage.angular_velocity.x =
      gyroScaleFactor * static_cast<double>(get16(base + 36));

    // Stamp and publish
    imuMessage.header.stamp = node_->now();
    imuMessage.header.frame_id = "imu_link";
    imuPublisher->publish(imuMessage);

  // RANGECMP = 140
  } else if (msgId == 140) {
    // Extract GPS time
    unsigned short week = 
      static_cast<unsigned short>(get16(arrayStartPos + 14) & 0xFFFF);
    long msec = static_cast<long>(get16(arrayStartPos + 16));

    hw_interface_plugin_novatel_span::msg::NovatelGPSTime tmsg;
    tmsg.header.stamp = node_->now();
    tmsg.week = week;
    tmsg.milliseconds = msec;
    gpsTimePub->publish(tmsg);

    // Log raw data if requested
    if (recordGPSData && gpsDataFile.is_open()) {
      gpsDataFile.write(
        reinterpret_cast<char*>(&receivedData[arrayStartPos]),
        fullPacketLen);
    }
  }

  return true;
}

bool novatel_span_serial::verifyChecksum()
{
  // Not used here
  return true;
}

std::size_t novatel_span_serial::novatelSpanStreamMatcher(
  const boost::system::error_code & error,
  long totalBytesInBuffer)
{
  const long syncSeqLen = 3;
  headerLen = fullPacketLen = dataArrayStart = 0;

  if (error == boost::asio::error::operation_aborted) {
    return 0;
  }

  // ... (port the same sync‐sequence logic, replacing ROS_DEBUG with RCLCPP_DEBUG) ...

  return 0;  // return 0 when a full packet is ready
}

unsigned long novatel_span_serial::CRC32Value_(int i)
{
  unsigned long ulCRC = static_cast<unsigned long>(i);
  for (int j = 8; j > 0; --j) {
    if (ulCRC & 1)
      ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
    else
      ulCRC >>= 1;
  }
  return ulCRC;
}

unsigned long novatel_span_serial::CalculateBlockCRC32_(
  unsigned long ulCount,
  unsigned char * ucBuffer)
{
  unsigned long ulCRC = 0;
  while (ulCount--) {
    unsigned long temp1 = (ulCRC >> 8) & 0x00FFFFFFL;
    unsigned long temp2 = CRC32Value_(
      static_cast<int>((ulCRC ^ *ucBuffer++) & 0xFF));
    ulCRC = temp1 ^ temp2;
  }
  return ulCRC;
}

}  // namespace hw_interface_plugin_novatel_span
