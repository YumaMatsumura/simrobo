#include "simrobo_driver/driver_component.hpp"

namespace simrobo_driver
{

// ========== コンストラクタ ========== //
Driver::Driver(
  const rclcpp::NodeOptions& options
): Node("simrobo_driver", options)
{
  // parameters
  wheel_radius_m_ = this->declare_parameter<float>("wheel_radius_size_m", 0.1);
  tread_width_m_ = this->declare_parameter<float>("tread_width_m", 0.33);

  // publisher, subscriber & timer
  pub_servo_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 1);
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&Driver::twistCallback, this, _1)
  );
  timer_ = this->create_wall_timer(10ms, std::bind(&Driver::updateCallback, this));
}

// ========== 速度(m/s)->角速度(rad/s)変換関数
float Driver::velocityToRound(float vel)
{
  return vel / (wheel_radius_m_ * 2.0 * M_PI);
}

// ========== Twistのコールバック関数 ========== //
void Driver::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  twist_buff.linear.x  = msg->linear.x;
  twist_buff.linear.y  = msg->linear.y;
  twist_buff.angular.z = msg->angular.z;
  
}

// ========== Timerのコールバック関数 ========== //
void Driver::updateCallback()
{
  // Velocity Commandの計算
  publishVelCommand();
}

// ========== Velocity CommandをPublishする関数 ========== //
void Driver::publishVelCommand()
{
  float v_left = twist_buff.linear.x - ((tread_width_m_ / 2.0) * twist_buff.angular.z);
  float v_right = twist_buff.linear.x + ((tread_width_m_ / 2.0) * twist_buff.angular.z);

  auto vel_msg = std_msgs::msg::Float64MultiArray();
  vel_msg.data.resize(2);

  vel_msg.data[0] = velocityToRound(v_left);
  vel_msg.data[1] = velocityToRound(v_right);

  pub_servo_->publish(vel_msg);
}

} // namespace simrobo_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(simrobo_driver::Driver)
