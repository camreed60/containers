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

#ifndef BASE_INTERFACE_HPP__
#define BASE_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <boost/system/error_code.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/cstdint.hpp>
#include <boost/crc.hpp>
#include <cstdint>
#include <boost/filesystem/path.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <hw_interface/shared_const_buffer.hpp>

#define THREAD_ID_TO_C_STR \
  boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()
#define FILENAME_C_STR \
  boost::filesystem::path(__FILE__).filename().c_str()
#define LINENUMBER_C_STR \
  boost::lexical_cast<std::string>(__LINE__).c_str()

// Info
#define RCLCPP_INFO_EXTRA_SINGLE(fmt) \
  RCLCPP_INFO(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR)
#define RCLCPP_INFO_EXTRA(fmt, ...) \
  RCLCPP_INFO(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR, ##__VA_ARGS__)
#define RCLCPP_INFO_EXTRA_NAME(fmt, ...) \
  RCLCPP_INFO(this->get_logger(), "%s:: " fmt, pluginName.c_str(), ##__VA_ARGS__)
// Debug
#define RCLCPP_DEBUG_EXTRA_SINGLE(fmt) \
  RCLCPP_DEBUG(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR)
#define RCLCPP_DEBUG_EXTRA(fmt, ...) \
  RCLCPP_DEBUG(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR, ##__VA_ARGS__)
#define RCLCPP_DEBUG_EXTRA_NAME(fmt, ...) \
  RCLCPP_DEBUG(this->get_logger(), "%s:: " fmt, pluginName.c_str(), ##__VA_ARGS__)
// Warn
#define RCLCPP_WARN_EXTRA_SINGLE(fmt) \
  RCLCPP_WARN(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR)
#define RCLCPP_WARN_EXTRA(fmt, ...) \
  RCLCPP_WARN(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR, ##__VA_ARGS__)
#define RCLCPP_WARN_EXTRA_NAME(fmt, ...) \
  RCLCPP_WARN(this->get_logger(), "%s:: " fmt, pluginName.c_str(), ##__VA_ARGS__)
// Error
#define RCLCPP_ERROR_EXTRA_SINGLE(fmt) \
  RCLCPP_ERROR(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR)
#define RCLCPP_ERROR_EXTRA(fmt, ...) \
  RCLCPP_ERROR(this->get_logger(), "%s %s:: " fmt, FILENAME_C_STR, LINENUMBER_C_STR, ##__VA_ARGS__)
#define RCLCPP_ERROR_EXTRA_NAME(fmt, ...) \
  RCLCPP_ERROR(this->get_logger(), "%s:: " fmt, pluginName.c_str(), ##__VA_ARGS__)

namespace base_classes
{
  enum interfaceType_t { Serial, UDP, Custom };

  class base_interface : public rclcpp::Node
  {
  public:
    using MatcherIterator =
      boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type>;

    std::string pluginName;
    interfaceType_t interfaceType{ Custom };
    bool interfaceStarted{ false };
    bool enabled{ false };

    bool enableCompletionFunctor{ false };
    bool enableStreamMatcher{ false };
    bool enableRegexReadUntil{ false };

    rclcpp::PublisherBase::SharedPtr    rosDataPub;
    rclcpp::SubscriptionBase::SharedPtr rosDataSub;

    boost::shared_ptr<boost::asio::io_context::strand> interfaceSynchronousStrand;

    boost::function<std::size_t(const boost::system::error_code&, std::size_t)>
      streamCompletionChecker;
    boost::function<std::pair<MatcherIterator,bool>(MatcherIterator,MatcherIterator)>
      streamSequenceMatcher;

    // Pure virtuals for derived plugins
    virtual bool interfaceReady() = 0;
    virtual bool initPlugin(
      const rclcpp::Node::SharedPtr & node,
      const boost::shared_ptr<boost::asio::io_context> & ioContext) = 0;
    virtual bool startWork() = 0;
    virtual bool stopWork() = 0;
    virtual bool verifyChecksum() = 0;
    virtual bool handleIORequest(
      const boost::system::error_code & ec,
      std::size_t bytesReceived) = 0;
    virtual void postInterfaceWriteRequest(
      const hw_interface_support_types::shared_const_buffer & buffer) = 0;
    virtual void interfaceWriteHandler(
      const hw_interface_support_types::shared_const_buffer & buffer,
      const boost::system::error_code & ec) = 0;

    static uint16_t calcCRC16Block(const void * buf, std::size_t numBytes);
    static uint32_t calcCRC32Block(const void * buf, std::size_t numBytes);

    void setupStreamMatcherDelimAndLength(
      int packetLengthInBytes,
      const char * headerSequence,
      const char * footerSequence)
    {
      streamCompletionChecker =
        boost::bind(
          &base_interface::streamMatcherDelimAndLength, this,
          std::placeholders::_1, std::placeholders::_2,
          packetLengthInBytes, headerSequence, footerSequence
        );
      enableCompletionFunctor = !streamCompletionChecker.empty();
    }

    std::size_t streamMatcherDelimAndLength(
      const boost::system::error_code & error,
      long totalBytesInBuffer,
      const int & packetLengthInBytes,
      const char * headerSequence,
      const char * footerSequence);

    bool enableMetrics();
    bool disableMetrics();
    std::string printMetrics(bool printSideEffect);

    base_interface();
    virtual ~base_interface();

  protected:
    boost::shared_array<uint8_t> receivedData;
    int dataArrayStart{ 0 };

  private:
    bool metricsEnabled{ false };
    boost::accumulators::accumulator_set<
      double, boost::accumulators::stats<
        boost::accumulators::tag::rolling_mean
      >
    > acc;
    rclcpp::Time lastTimeMetric;
    double deltaTime{ 0.0 };
  };
}  // namespace base_classes

#endif  // BASE_INTERFACE_HPP__
