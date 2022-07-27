#include "simrobo_driver/driver_component.hpp"

namespace simrobo_driver
{

// ========== コンストラクタ ========== //
Driver::Driver(
  const rclcpp::NodeOptions& options
): Node("simrobo_driver", options),
   pos{0.0, 0.0, 0.0}
{
  // initialize
  prev_time = this->now();

  // parameters
  wheel_radius_m_ = this->declare_parameter<float>("wheel_radius_size_m", 0.1);
  tread_width_m_ = this->declare_parameter<float>("tread_width_m", 0.33);
  global_frame_id_ = this->declare_parameter<std::string>("global_frame_id", "odom");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");

  // publisher, subscriber & timer
  pub_servo_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 1);
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&Driver::twistCallback, this, _1)
  );
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
  rclcpp::Time current_time = this->now();
  rclcpp::Duration duration(current_time - prev_time);
  prev_time = current_time;

  // Odometryの計算
  publishOdom(current_time, duration);

  // TFの計算
  publishTF(current_time);

  // Velocity Commandの計算
  publishVelCommand();
}

// ========== OdometryをPublishする関数 ========== //
void Driver::publishOdom(const rclcpp::Time & now, const rclcpp::Duration & duration)
{
  double dt_sec = duration.nanoseconds() / 1e9;
  pos[0] += twist_buff.linear.x * cos(pos[2] + (twist_buff.angular.z * dt_sec / 2.0)) * dt_sec;
  pos[1] += twist_buff.linear.x * sin(pos[2] + (twist_buff.angular.z * dt_sec / 2.0)) * dt_sec;
  pos[2] += twist_buff.angular.z * dt_sec;

  tf2::Quaternion q;
  q.setRPY(0, 0, pos[2]);

  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = global_frame_id_;
  odom_msg.pose.pose.position.x = pos[0];
  odom_msg.pose.pose.position.y = pos[1];
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  odom_msg.twist.twist.linear.x = twist_buff.linear.x;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = twist_buff.angular.z;

  pub_odom_->publish(odom_msg);
}

// ========== TFをPublishする関数 ========== //
void Driver::publishTF(const rclcpp::Time & now)
{
  auto tf_msg = geometry_msgs::msg::TransformStamped();
  auto tf_odom_msg = tf2_msgs::msg::TFMessage();
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = base_frame_id_;
  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

  tf_odom_msg.transforms.push_back(tf_msg);
  pub_tf_->publish(tf_odom_msg);
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
