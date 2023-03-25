#ifndef SIMROBO_DRIVER__DRIVER_HPP_
#define SIMROBO_DRIVER__DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::placeholders;

namespace simrobo_driver
{
  
class Driver : public rclcpp::Node
{
public:
  explicit Driver(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  
private:
  void initVariables();
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void updateCallback();
  void publishOdom();
  void publishTF();
  void publishVelCommand();
  double velocityToRound(double vel);
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_servo_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Pose2D pose_;
  geometry_msgs::msg::Twist twist_buff;
  nav_msgs::msg::Odometry odom_msg;
  double wheel_radius_m_;
  double tread_width_m_;
  double frequency_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
};

}// namespace simrobo_driver

#endif // SIMROBO_DRIVER__DRIVER_HPP_
