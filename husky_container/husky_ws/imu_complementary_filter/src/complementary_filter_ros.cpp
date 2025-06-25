#include "imu_complementary_filter/complementary_filter_ros.hpp"

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

namespace imu_tools
{

ComplementaryFilterROS::ComplementaryFilterROS(const rclcpp::NodeOptions & options)
: Node("ComplementaryFilterROS", options),
  filter_(),
  initialized_filter_(false)
{
  RCLCPP_INFO(get_logger(), "Starting ComplementaryFilterROS");
  initializeParams();

  const size_t queue_size = 5;
  const std::string imu_ns = this->declare_parameter<std::string>("imu_namespace", "imu");

  // Publishers
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    imu_ns + "/data", queue_size);

  if (publish_debug_topics_) {
    rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      imu_ns + "/rpy/filtered", queue_size);
    if (filter_.getDoBiasEstimation()) {
      state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        imu_ns + "/steady_state", queue_size);
    }
  }

  // IMU subscriber
  imu_subscriber_ = std::make_shared<ImuSubscriber>(
    *this, imu_ns + "/data_raw", queue_size);

  if (use_mag_) {
    mag_subscriber_ = std::make_shared<MagSubscriber>(
      *this, imu_ns + "/mag", queue_size);

    sync_ = std::make_shared<Synchronizer>(
      SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_);
    sync_->registerCallback(
      std::bind(&ComplementaryFilterROS::imuMagCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  } else {
    imu_subscriber_->registerCallback(
      std::bind(&ComplementaryFilterROS::imuCallback, this,
                std::placeholders::_1));
  }
}

ComplementaryFilterROS::~ComplementaryFilterROS()
{
  RCLCPP_INFO(get_logger(), "Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams()
{
  // Private parameters
  fixed_frame_          = this->declare_parameter<std::string>("fixed_frame", "odom");
  use_mag_              = this->declare_parameter<bool>("use_mag", false);
  publish_tf_           = this->declare_parameter<bool>("publish_tf", false);
  reverse_tf_           = this->declare_parameter<bool>("reverse_tf", false);
  constant_dt_          = this->declare_parameter<double>("constant_dt", 0.0);
  publish_debug_topics_ = this->declare_parameter<bool>("publish_debug_topics", false);

  double gain_acc       = this->declare_parameter<double>("gain_acc", 0.01);
  double gain_mag       = this->declare_parameter<double>("gain_mag", 0.01);
  bool   do_bias_est    = this->declare_parameter<bool>("do_bias_estimation", true);
  double bias_alpha     = this->declare_parameter<double>("bias_alpha", 0.01);
  bool   do_adapt_gain  = this->declare_parameter<bool>("do_adaptive_gain", true);
  double orientation_stddev =
    this->declare_parameter<double>("orientation_stddev", 0.0);
  orientation_variance_ = orientation_stddev * orientation_stddev;

  filter_.setDoBiasEstimation(do_bias_est);
  filter_.setDoAdaptiveGain(do_adapt_gain);
  if (!filter_.setGainAcc(gain_acc)) {
    RCLCPP_WARN(get_logger(), "Invalid gain_acc passed to ComplementaryFilter.");
  }
  if (use_mag_) {
    if (!filter_.setGainMag(gain_mag)) {
      RCLCPP_WARN(get_logger(), "Invalid gain_mag passed to ComplementaryFilter.");
    }
  }
  if (do_bias_est) {
    if (!filter_.setBiasAlpha(bias_alpha)) {
      RCLCPP_WARN(get_logger(), "Invalid bias_alpha passed to ComplementaryFilter.");
    }
  }
  if (constant_dt_ < 0.0f) {
    RCLCPP_WARN(
      get_logger(),
      "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
      constant_dt_);
    constant_dt_ = 0.0;
  }
}

void ComplementaryFilterROS::imuCallback(const ImuMsg::ConstSharedPtr imu_msg_raw)
{
  const auto & a = imu_msg_raw->linear_acceleration;
  const auto & w = imu_msg_raw->angular_velocity;
  const auto time = imu_msg_raw->header.stamp;

  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  double dt = (constant_dt_ > 0.0) ?
    constant_dt_ :
    (time - time_prev_).seconds();
  time_prev_ = time;

  filter_.update(a.x, a.y, a.z,
                 w.x, w.y, w.z,
                 dt);

  publish(imu_msg_raw);
}

void ComplementaryFilterROS::imuMagCallback(
  const ImuMsg::ConstSharedPtr imu_msg_raw,
  const MagMsg::ConstSharedPtr mag_msg)
{
  const auto & a = imu_msg_raw->linear_acceleration;
  const auto & w = imu_msg_raw->angular_velocity;
  const auto & m = mag_msg->magnetic_field;
  const auto time = imu_msg_raw->header.stamp;

  if (!initialized_filter_) {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  double dt = (time - time_prev_).seconds();
  time_prev_ = time;

  if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z)) {
    filter_.update(a.x, a.y, a.z,
                   w.x, w.y, w.z,
                   dt);
  } else {
    filter_.update(a.x, a.y, a.z,
                   w.x, w.y, w.z,
                   m.x, m.y, m.z,
                   dt);
  }

  publish(imu_msg_raw);
}

tf2::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(
  double q0, double q1, double q2, double q3) const
{
  // Convert Hamiltonian (w = q0) → tf2 (w = q3)
  return tf2::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(const ImuMsg::ConstSharedPtr imu_msg_raw)
{
  // Get orientation
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  auto q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // Copy and fill IMU msg
  ImuMsg imu = *imu_msg_raw;
  imu.orientation = tf2::toMsg(q);
  for (int i = 0; i < 9; ++i) {
    imu.orientation_covariance[i] =
      (i % 4 == 0) ? orientation_variance_ : 0.0;
  }
  if (filter_.getDoBiasEstimation()) {
    imu.angular_velocity.x -= filter_.getAngularVelocityBiasX();
    imu.angular_velocity.y -= filter_.getAngularVelocityBiasY();
    imu.angular_velocity.z -= filter_.getAngularVelocityBiasZ();
  }
  imu_publisher_->publish(imu);

  // Debug topics
  if (publish_debug_topics_) {
    Vector3Msg rpy;
    rpy.header = imu.header;
    tf2::Matrix3x3(q).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    rpy_publisher_->publish(rpy);

    if (filter_.getDoBiasEstimation()) {
      std_msgs::msg::Bool state_msg;
      state_msg.data = filter_.getSteadyState();
      state_publisher_->publish(state_msg);
    }
  }

  // TF broadcasting
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = imu.header.stamp;
    t.header.frame_id = fixed_frame_;
    t.child_frame_id  = imu.header.frame_id;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(reverse_tf_ ? q.inverse() : q);
    tf_broadcaster_.sendTransform(t);
  }
}

}  // namespace imu_tools
