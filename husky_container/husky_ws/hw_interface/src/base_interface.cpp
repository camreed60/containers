#include "hw_interface/base_interface.hpp"

#include <boost/accumulators/statistics/mean.hpp>
#include <boost/scoped_ptr.hpp>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace base_classes
{

base_interface::base_interface()
: lastTimeMetric(this->get_clock()->now()),
  acc(boost::accumulators::tag::rolling_window::window_size = 150)
{
  enabled = true;
}

bool base_interface::enableMetrics()
{
  metricsEnabled = true;
  lastTimeMetric = this->get_clock()->now();
  return metricsEnabled;
}

bool base_interface::disableMetrics()
{
  metricsEnabled = false;
  return metricsEnabled;
}

std::string base_interface::printMetrics(bool printSideEffect)
{
  if (!metricsEnabled) {
    return "";
  }

  // Compute delta time
  auto now = this->get_clock()->now();
  double delta = (now - lastTimeMetric).seconds();
  acc(delta);
  lastTimeMetric = now;

  // Format output
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer),
    "Thread <%s>:: Interface <%15s>:: Delta Time %2.3f:: Hz %2.2f:: Avg Hz %2.2f",
    THREAD_ID_TO_C_STR,
    this->pluginName.c_str(),
    delta,
    1.0 / delta,
    1.0 / boost::accumulators::rolling_mean(acc)
  );

  if (printSideEffect) {
    RCLCPP_DEBUG(this->get_logger(), "%s", buffer);
  }

  return std::string(buffer);
}

uint16_t base_interface::calcCRC16Block(const void * buf, std::size_t numOfBytes)
{
  return boost::crc<16, 0x1021, 0xFFFF, 0, false, false>(buf, numOfBytes);
}

uint32_t base_interface::calcCRC32Block(const void * buf, std::size_t numOfBytes)
{
  boost::crc_32_type crcComputer;
  crcComputer.process_bytes(buf, numOfBytes);
  return crcComputer.checksum();
}

std::size_t base_interface::streamMatcherDelimAndLength(
  const boost::system::error_code & error,
  long totalBytesInBuffer,
  const int & packetLengthInBytes,
  const char * headerSequence,
  const char * footerSequence)
{
  RCLCPP_DEBUG(this->get_logger(),
    "%s:: Length and footer matcher: buffer length %ld",
    pluginName.c_str(), totalBytesInBuffer);
  RCLCPP_DEBUG(this->get_logger(), "packetLength %d", packetLengthInBytes);
  RCLCPP_DEBUG(this->get_logger(), "Candidate Header: %s", headerSequence);

  if (totalBytesInBuffer <= static_cast<long>(std::strlen(footerSequence))) {
    RCLCPP_DEBUG(this->get_logger(), "%s:: Matcher Returning Early", pluginName.c_str());
    return packetLengthInBytes - totalBytesInBuffer;
  }

  if ((packetLengthInBytes - totalBytesInBuffer) <= 0) {
    RCLCPP_DEBUG(this->get_logger(), "%s:: Full Length Packet Received", pluginName.c_str());
    std::printf("Contents: ");
    for (int i = 0; i < totalBytesInBuffer; ++i) {
      std::printf("%c, %X | ", receivedData[i], receivedData[i]);
    }
    std::printf("\n");

    // Check footer
    int footerLength = std::strlen(footerSequence);
    for (int i = 0; i < footerLength; ++i) {
      char received = receivedData[totalBytesInBuffer - 1 - i];
      if (received != footerSequence[footerLength - 1 - i]) {
        RCLCPP_ERROR(this->get_logger(), "%s:: Invalid Footer", pluginName.c_str());
        return footerLength;
      }
    }
    RCLCPP_DEBUG(this->get_logger(), "Footer OK");

    // Check header
    int headerLength = std::strlen(headerSequence);
    for (int i = 0; i < headerLength; ++i) {
      char received = receivedData[totalBytesInBuffer - packetLengthInBytes + i];
      if (received != headerSequence[i]) {
        RCLCPP_ERROR(this->get_logger(), "%s:: Invalid Header", pluginName.c_str());
        return packetLengthInBytes;
      }
    }
    RCLCPP_DEBUG(this->get_logger(),
      "%s:: Header Found, Footer Found, Correct Length, Good Packet",
      pluginName.c_str());

    dataArrayStart = totalBytesInBuffer - packetLengthInBytes;
    return 0;
  }

  return packetLengthInBytes - totalBytesInBuffer;
}

}  // namespace base_classes
