#ifndef HW_INTERFACE_PLUGIN_NOVATEL_SPAN_HPP__
#define HW_INTERFACE_PLUGIN_NOVATEL_SPAN_HPP__

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <hw_interface_plugin_novatel_span/msg/novatel_gps_time.hpp>
#include <fstream>

#define CRC32_POLYNOMIAL 0xEDB88320UL
#define PI               3.14159265358979323846
#define RAD2DEG          (180.0/PI)
#define DEG2RAD          (PI/180.0)
#define IMU_RATE         125.0  // Hz

namespace hw_interface_plugin_novatel_span
{

class novatel_span_serial : public base_classes::base_serial_interface
{
public:
  novatel_span_serial();
  ~novatel_span_serial() override = default;

protected:
  bool subPluginInit(const rclcpp::Node::SharedPtr & node) override;
  void setInterfaceOptions() override;
  bool interfaceReadHandler(
    const size_t & length,
    int arrayStartPos,
    const boost::system::error_code & ec) override;
  bool verifyChecksum() override;

  std::size_t novatelSpanStreamMatcher(
    const boost::system::error_code & error,
    long totalBytesInBuffer) override;

  long headerLen;
  long fullPacketLen;

  sensor_msgs::msg::Imu imuMessage;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

  hw_interface_plugin_novatel_span::msg::NovatelGPSTime gpsTimeMsg;
  rclcpp::Publisher<hw_interface_plugin_novatel_span::msg::NovatelGPSTime>::SharedPtr gpsTimePub;

  double gyroScaleFactor;
  double accelScaleFactor;
  bool recordGPSData{false};
  std::ofstream gpsDataFile;

private:
  unsigned long CRC32Value_(int i);
  unsigned long CalculateBlockCRC32_(
    unsigned long ulCount,
    unsigned char * ucBuffer);
};

}  // namespace hw_interface_plugin_novatel_span

PLUGINLIB_EXPORT_CLASS(
  hw_interface_plugin_novatel_span::novatel_span_serial,
  base_classes::base_interface
)

#endif  // HW_INTERFACE_PLUGIN_NOVATEL_SPAN_HPP__
