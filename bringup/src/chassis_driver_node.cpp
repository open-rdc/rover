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

    init_serial();
    last_joy_vel_time = this->now();

    RCLCPP_INFO(this->get_logger(), "車輪半径:%f  トレッド:%f", wheel_radius, tread);
    RCLCPP_INFO(this->get_logger(), "最大並進速度:%f  最大回転速度:%f", linear_max_vel, angular_max_vel);
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

/*シリアル通信*/
void ChassisDriver::init_serial() {
    serial_fd = open(serial_port.c_str(), O_WRONLY | O_NOCTTY);
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
    int_to_bytes(bytes, left_rpm);
    int_to_bytes(bytes + 4, right_rpm);
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

}  // namespace chassis_driver
