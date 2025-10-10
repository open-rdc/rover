#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <array>

namespace chassis_driver{

class ChassisDriver : public rclcpp::Node {
public:
    explicit ChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ChassisDriver();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_vel;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_joy_vel;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _publisher_odom;
  rclcpp::TimerBase::SharedPtr _timer;

  void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void _subscriber_callback_joy_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void calculate_chassis(const geometry_msgs::msg::Twist::SharedPtr vel);
  void _timer_callback();

  rclcpp::QoS _qos = rclcpp::QoS(10);
  std::array<int, 4> wheel_erpms{0, 0, 0, 0};

  // シリアル通信
  int serial_fd;
  void init_serial();
  void send_data_to_serial(const int left_rpm, const int right_rpm);
  void read_wheel_erpms_from_serial();
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
