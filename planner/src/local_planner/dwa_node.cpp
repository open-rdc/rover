#include "local_planner/dwa_node.hpp"
#include <cmath>
#include <chrono>
#include "utilities/utils.hpp"

using namespace std::chrono_literals;
using namespace utils;

namespace local_planner
{

DWA::DWA(const rclcpp::NodeOptions& options) : DWA("", options) {}

DWA::DWA(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("dwa_node", name_space, options),
linear_max_vel_(get_parameter("linear_max_vel").as_double()),
angular_max_vel_(get_parameter("angular_max_vel").as_double()),
control_frequency_(get_parameter("control_frequency").as_double())
{
    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS interfaces
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "target_pose", 10,
        std::bind(&DWA::target_pose_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Control timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&DWA::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "DWA Node has been started");
    RCLCPP_INFO(this->get_logger(), "Max linear:%.2fm/s angular:%.2frad/s", linear_max_vel_, angular_max_vel_);
    RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_frequency_);
}

void DWA::target_pose_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
    current_target_pose_ = msg;
}

void DWA::control_timer_callback(){
    if (!current_target_pose_) {
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }
}

bool DWA::get_robot_pose(geometry_msgs::msg::PoseStamped& robot_pose){
    try {
        auto transform = tf_buffer_->lookupTransform("utm", "base_link", tf2::TimePointZero);

        robot_pose.header.stamp = this->get_clock()->now();
        robot_pose.header.frame_id = "utm";
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;

        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Could not get robot pose from utm->base_link: %s", ex.what());
        return false;
    }
}


}  // namespace local_planner

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::spin(std::make_shared<local_planner::DWA>(options));
    rclcpp::shutdown();
    return 0;
}
