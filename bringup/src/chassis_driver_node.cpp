#include "chassis_driver/chassis_driver_node.hpp"

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"
#include <cmath>
#include <cstring>

using namespace utils;

namespace chassis_driver{

ChassisDriver::ChassisDriver(const rclcpp::NodeOptions& options) : ChassisDriver("", options) {}

ChassisDriver::ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("chassis_driver_node", name_space, options),
wheel_radius(get_parameter("wheel_radius").as_double()),
tread(get_parameter("tread").as_double()),
linear_max_vel(get_parameter("linear_max_vel").as_double()),
angular_max_vel(dtor(get_parameter("angular_max_vel").as_double())),
serial_port(get_parameter("serial_port").as_string()),
pole_pairs(get_parameter("pole_pairs").as_int())
{
    _subscription_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_vel, this, std::placeholders::_1)
    );
    _subscription_joy_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "joy_vel",
        _qos,
        std::bind(&ChassisDriver::_subscriber_callback_joy_vel, this, std::placeholders::_1)
    );

    _publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", _qos);

    init_serial();
    last_joy_vel_time = this->now();

    // 100Hzで車輪速度を出力
    _timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ChassisDriver::_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Wheel Radius:%f  Tread:%f", wheel_radius, tread);
    RCLCPP_INFO(this->get_logger(), "Max Linear Velocity:%f  Max Angular Velocity:%f", linear_max_vel, angular_max_vel);
}

ChassisDriver::~ChassisDriver() {
    if (serial_fd >= 0) {
        close(serial_fd);
        RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }
}

void ChassisDriver::_subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(mode == Mode::stop) return;

    // joy_velの受信から1秒以上経過していたらcmd_velを有効化
    if((this->now() - last_joy_vel_time).seconds() >= 1.0) {
        mode = Mode::cmd;
        calculate_chassis(msg);
    }
}
void ChassisDriver::_subscriber_callback_joy_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(mode == Mode::stop) return;

    mode = Mode::joy;
    last_joy_vel_time = this->now();
    calculate_chassis(msg);
}

void ChassisDriver::calculate_chassis(const geometry_msgs::msg::Twist::SharedPtr vel){
    const double linear_vel = constrain(vel->linear.x, -linear_max_vel, linear_max_vel);
    const double angular_vel = constrain(vel->angular.z, -angular_max_vel, angular_max_vel);

    const double left_vel = (-tread*angular_vel + 2.0*linear_vel) / (2.0*wheel_radius);
    const double right_vel = (tread*angular_vel + 2.0*linear_vel) / (2.0*wheel_radius);

    // rad/s -> rpm -> erpm
    const double left_rpm = left_vel * 30.0 / d_pi;
    const double right_rpm = right_vel * 30.0 / d_pi;

    // RPM to ERPM
    const int left_erpm = static_cast<int>(left_rpm * pole_pairs);
    const int right_erpm = static_cast<int>(right_rpm * pole_pairs);

    send_data_to_serial(left_erpm, right_erpm);

    RCLCPP_INFO(this->get_logger(), "Sent left_erpm: %d, right_erpm: %d", left_erpm, right_erpm);
}

// タイマーコールバック
void ChassisDriver::_timer_callback() {
    read_wheel_erpms_from_serial();

    // RCLCPP_INFO(this->get_logger(),
    //             "Wheel speeds - Wheel1: %d, Wheel2: %d, Wheel3: %d, Wheel4: %d",
    //             wheel_erpms[0], wheel_erpms[1], wheel_erpms[2], wheel_erpms[3]);

    // ERPMからRPMに変換してから角速度に変換
    const double left_vel = ((wheel_erpms[0] + wheel_erpms[1]) / 2.0) / pole_pairs * d_pi / 30.0;
    const double right_vel = ((wheel_erpms[2] + wheel_erpms[3]) / 2.0) / pole_pairs * d_pi / 30.0;

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.twist.twist.linear.x = (left_vel + right_vel) * wheel_radius / 2.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = (right_vel - left_vel) * wheel_radius / tread;

    _publisher_odom->publish(odom_msg);
}

/*シリアル通信*/
void ChassisDriver::init_serial() {
    serial_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s", serial_port.c_str());
        return;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);

    // ボーレート設定（9600bps）
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // 8N1設定
    options.c_cflag &= ~PARENB;   // パリティなし
    options.c_cflag &= ~CSTOPB;   // ストップビット1
    options.c_cflag &= ~CSIZE;    // データビットクリア
    options.c_cflag |= CS8;       // データビット8
    options.c_cflag |= CLOCAL | CREAD; // ローカル接続、受信有効

    // Raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(serial_fd, TCSANOW, &options);

    RCLCPP_INFO(this->get_logger(), "Serial port %s initialized", serial_port.c_str());
}

void ChassisDriver::send_data_to_serial(const int left_rpm, const int right_rpm) {
    if (serial_fd < 0) {
        RCLCPP_WARN(this->get_logger(), "Serial port not initialized");
        return;
    }

    uint8_t bytes[serial_byte_size];
    int_to_bytes(bytes, static_cast<int32_t>(left_rpm));
    int_to_bytes(bytes + 4, static_cast<int32_t>(right_rpm));
    if(mode == Mode::cmd){
        bytes[8] = 0x01;
    } else if(mode == Mode::joy){
        bytes[8] = 0x02;
    } else{
        bytes[8] = 0x00;
    }

    // シリアルポートに送信
    ssize_t written = write(serial_fd, bytes, serial_byte_size);
    if (written != serial_byte_size) {
        RCLCPP_WARN(this->get_logger(), "Failed to write complete data to serial");
    }
}

void ChassisDriver::read_wheel_erpms_from_serial() {
    if (serial_fd < 0) {
        RCLCPP_WARN(this->get_logger(), "Serial port not initialized");
        return;
    }

    uint8_t buffer[serial_byte_size];
    ssize_t bytes_read = read(serial_fd, buffer, serial_byte_size);

    if (bytes_read == serial_byte_size) {
        for (int i = 0; i < 4; i++) {
            wheel_erpms[i] = bytes_to_int(buffer + i*4);
        }
    } else if (bytes_read > 0) {
        RCLCPP_WARN(this->get_logger(), "Incomplete data received: %ld bytes", bytes_read);
    }
}

}  // namespace chassis_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto chassis_node = std::make_shared<chassis_driver::ChassisDriver>(options);

  rclcpp::spin(chassis_node);
  rclcpp::shutdown();
  return 0;
}
