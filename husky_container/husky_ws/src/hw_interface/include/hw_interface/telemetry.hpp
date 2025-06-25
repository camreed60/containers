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

#ifndef TELEMETRY_TIME_HPP__
#define TELEMETRY_TIME_HPP__

#include <rclcpp/rclcpp.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <cstdio>
#include <string>

class delta_loop_time
{
private:
  bool metricsEnabled;
  boost::accumulators::accumulator_set<
    double,
    boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
  > acc;
  rclcpp::Time lastTimeMetric;
  double deltaTime;
  rclcpp::Clock clock;

public:
  delta_loop_time()
  : metricsEnabled(true),
    acc(boost::accumulators::tag::rolling_window::window_size = 150),
    lastTimeMetric(clock.now()),
    deltaTime(0.0)
  {}

  bool enableMetrics()
  {
    metricsEnabled = true;
    lastTimeMetric = clock.now();
    return metricsEnabled;
  }

  bool disableMetrics()
  {
    metricsEnabled = false;
    return metricsEnabled;
  }

  float updateMetrics()
  {
    if (!metricsEnabled) {
      return 0.0f;
    }
    auto now = clock.now();
    deltaTime = (now - lastTimeMetric).seconds();
    acc(deltaTime);
    lastTimeMetric = now;
    return 1.0f / static_cast<float>(boost::accumulators::rolling_mean(acc));
  }

  std::string printMetrics(bool printSideEffect)
  {
    if (!metricsEnabled) {
      return "";
    }
    auto now = clock.now();
    deltaTime = (now - lastTimeMetric).seconds();
    acc(deltaTime);
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer),
      "Delta Time %2.3f:: Hz %2.2f:: Avg Hz %2.2f",
      deltaTime,
      1.0 / deltaTime,
      1.0 / static_cast<double>(boost::accumulators::rolling_mean(acc))
    );
    lastTimeMetric = now;
    if (printSideEffect) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("delta_loop_time"),
        "%s", buffer
      );
    }
    return std::string(buffer);
  }
};

#endif  // TELEMETRY_TIME_HPP__
