#include "simrobo_driver/driver_component.hpp"

namespace simrobo_driver
{

// ========== コンストラクタ ========== //
Driver::Driver(const rclcpp::NodeOptions & options)
: Node("simrobo_driver", options)
{
  // initialize
  initVariables();

  // parameters
  frequency_ = this->declare_parameter<double>("frequency", 100.0);
  wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_size_m", 0.1);
  tread_width_m_ = this->declare_parameter<double>("tread_width_m", 0.33);
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");

  // publisher, subscriber & timer
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  pub_servo_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", rclcpp::SystemDefaultsQoS());
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS());
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&Driver::twistCallback, this, _1));

  double duration_ms = 1000.0 / frequency_;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(duration_ms)),
    std::bind(&Driver::updateCallback, this));
}

// ========== 変数の初期化 ========== //
void Driver::initVariables()
{
  twist_buff.linear.x = 0.0;
  twist_buff.linear.y = 0.0;
  twist_buff.angular.z = 0.0;
  pose_.x = 0.0;
  pose_.y = 0.0;
  pose_.theta = 0.0;
}

// ========== Twistのコールバック関数 ========== //
void Driver::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  twist_buff.linear.x = msg->linear.x;
  twist_buff.linear.y = msg->linear.y;
  twist_buff.angular.z = msg->angular.z;

}

// ========== Timerのコールバック関数 ========== //
void Driver::updateCallback()
{
  // pose(x, y, theta)の計算
  double duration_sec = 1.0 / frequency_;
  pose_.x += twist_buff.linear.x * cos(pose_.theta + (twist_buff.angular.z * duration_sec / 2.0)) *
    duration_sec;
  pose_.y += twist_buff.linear.x * sin(pose_.theta + (twist_buff.angular.z * duration_sec / 2.0)) *
    duration_sec;
  pose_.theta += twist_buff.angular.z * duration_sec;

  // Odometryの計算
  publishOdom();

  // TFの計算
  publishTF();

  // Velocity Commandの計算
  publishVelCommand();
}

// ========== OdometryをPublishする関数 ========== //
void Driver::publishOdom()
{
  nav_msgs::msg::Odometry odom_msg;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_.theta);

  odom_msg.header.stamp = this->now();

  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = base_frame_id_;

  odom_msg.pose.pose.position.x = pose_.x;
  odom_msg.pose.pose.position.y = pose_.y;
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
void Driver::publishTF()
{
  geometry_msgs::msg::TransformStamped tf_msg;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_.theta);

  tf_msg.header.stamp = this->now();

  tf_msg.header.frame_id = odom_frame_id_;
  tf_msg.child_frame_id = base_frame_id_;

  tf_msg.transform.translation.x = pose_.x;
  tf_msg.transform.translation.y = pose_.y;
  tf_msg.transform.translation.z = 0.0;

  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf_msg);
}

// ========== Velocity CommandをPublishする関数 ========== //
void Driver::publishVelCommand()
{
  double v_left = twist_buff.linear.x - ((tread_width_m_ / 2.0) * twist_buff.angular.z);
  double v_right = twist_buff.linear.x + ((tread_width_m_ / 2.0) * twist_buff.angular.z);

  auto vel_msg = std_msgs::msg::Float64MultiArray();
  vel_msg.data.resize(2);

  vel_msg.data[0] = velocityToRound(v_left);
  vel_msg.data[1] = velocityToRound(v_right);

  pub_servo_->publish(vel_msg);
}

// ========== 速度(m/s)->角速度(rad/s)変換関数 ========== //
double Driver::velocityToRound(double vel)
{
  return vel / wheel_radius_m_;
}

} // namespace simrobo_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(simrobo_driver::Driver)
