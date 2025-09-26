#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace chassis_driver{

class ChassisDriver : public rclcpp::Node {
public:
    explicit ChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ChassisDriver();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_vel;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_joy_vel;

  void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void _subscriber_callback_joy_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void calculate_chassis(const geometry_msgs::msg::Twist::SharedPtr vel);

  rclcpp::QoS _qos = rclcpp::QoS(10);

  // シリアル通信
  int serial_fd;
  void init_serial();
  void send_data_to_serial(const int left_rpm, const int right_rpm);
  const int serial_byte_size = 16;

  // 定数
  const double wheel_radius;
  const double tread;
  const double linear_max_vel;
  const double angular_max_vel;
  const std::string serial_port;
  const int pole_pairs;

  // 動作モード
  enum class Mode{
    cmd,
    joy,
    stay,
    stop
  } mode = Mode::stay;

  rclcpp::Time last_joy_vel_time;


};

}  // namespace chassis_driver
