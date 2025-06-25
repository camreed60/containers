#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_HPP
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "imu_complementary_filter/complementary_filter.hpp"

namespace imu_tools
{

class ComplementaryFilterROS : public rclcpp::Node
{
public:
  explicit ComplementaryFilterROS(const rclcpp::NodeOptions & options);
  ~ComplementaryFilterROS() override;

private:
  // Message typedefs
  using ImuMsg     = sensor_msgs::msg::Imu;
  using MagMsg     = sensor_msgs::msg::MagneticField;
  using Vector3Msg = geometry_msgs::msg::Vector3Stamped;

  // Synchronizer typedefs
  using SyncPolicy    = message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>;
  using Synchronizer  = message_filters::Synchronizer<SyncPolicy>;
  using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
  using MagSubscriber = message_filters::Subscriber<MagMsg>;

  // Subscribers & synchronizer
  std::shared_ptr<ImuSubscriber>    imu_subscriber_;
  std::shared_ptr<MagSubscriber>    mag_subscriber_;
  std::shared_ptr<Synchronizer>     sync_;

  // Publishers
  rclcpp::Publisher<ImuMsg>::SharedPtr     imu_publisher_;
  rclcpp::Publisher<Vector3Msg>::SharedPtr rpy_publisher_;
  rclcpp::Publisher<ImuMsg>::SharedPtr      state_publisher_;

  // TF broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Parameters
  bool   use_mag_;
  bool   publish_tf_;
  bool   reverse_tf_;
  double constant_dt_;
  bool   publish_debug_topics_;
  std::string fixed_frame_;
  double orientation_variance_;

  // Filter state
  ComplementaryFilter filter_;
  rclcpp::Time        time_prev_;
  bool                initialized_filter_;

  // Internal methods
  void initializeParams();
  void imuCallback(const ImuMsg::ConstSharedPtr imu_msg_raw);
  void imuMagCallback(const ImuMsg::ConstSharedPtr imu_msg_raw,
                      const MagMsg::ConstSharedPtr mag_msg);
  void publish(const ImuMsg::ConstSharedPtr imu_msg_raw);

  tf2::Quaternion hamiltonToTFQuaternion(double q0,
                                         double q1,
                                         double q2,
                                         double q3) const;
};

}  // namespace imu_tools

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_HPP
