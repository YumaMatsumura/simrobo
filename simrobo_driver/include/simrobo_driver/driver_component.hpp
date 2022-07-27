#ifndef SIMROBO_DRIVER__DRIVER_HPP_
#define SIMROBO_DRIVER__DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace simrobo_driver
{
  
class Driver : public rclcpp::Node
{
public:
  explicit Driver(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  
private:
  float velocityToRound(float vel);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void updateCallback();
  void publishOdom(const rclcpp::Time & now, const rclcpp::Duration & duration);
  void publishVelCommand();
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_servo_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time prev_time;
  
  geometry_msgs::msg::Twist twist_buff;
  float wheel_radius_m_;
  float tread_width_m_;
  double pos[3];
  std::string global_frame_id_;
};

}// namespace simrobo_driver

#endif // SIMROBO_DRIVER__DRIVER_HPP_
