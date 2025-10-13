#include "local_planner/local_planner_node.hpp"
#include <cmath>
#include <chrono>
#include "utilities/utils.hpp"

using namespace std::chrono_literals;
using namespace utils;

namespace local_planner
{

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options) : LocalPlanner("", options) {}

LocalPlanner::LocalPlanner(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("local_planner_node", name_space, options),
linear_max_vel_(get_parameter("linear_max_vel").as_double()),
angular_max_vel_(get_parameter("angular_max_vel").as_double()),
linear_gain_(get_parameter("linear_gain").as_double()),
angular_gain_(get_parameter("angular_gain").as_double()),
goal_tolerance_(get_parameter("goal_tolerance").as_double()),
control_frequency_(get_parameter("control_frequency").as_double())
{
    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS interfaces
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "target_pose", 10,
        std::bind(&LocalPlanner::target_pose_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Control timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LocalPlanner::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Local Planner Node has been started");
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", linear_max_vel_);
    RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", angular_max_vel_);
    RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(), "Linear gain: %.2f  Angular gain: %.2f", linear_gain_, angular_gain_);
}

void LocalPlanner::target_pose_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    current_target_pose_ = msg;
}

void LocalPlanner::control_timer_callback()
{
    if (!current_target_pose_) {
        // No target pose received yet, publish zero velocity
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    // Calculate and publish cmd_vel
    auto cmd_vel = calculate_cmd_vel();
    cmd_vel_pub_->publish(cmd_vel);
}

geometry_msgs::msg::Twist LocalPlanner::calculate_cmd_vel()
{
    auto cmd_vel = geometry_msgs::msg::Twist();

    // Get current robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!get_robot_pose(robot_pose)) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose, stopping");
        return cmd_vel;  // Return zero velocity
    }

    // Extract target position and yaw from Vector3 message (already in utm frame)
    geometry_msgs::msg::Point target_position;
    target_position.x = current_target_pose_->x;
    target_position.y = current_target_pose_->y;
    target_position.z = 0.0;

    // Calculate distance to target
    double distance = calculate_distance(robot_pose.pose.position, target_position);
    
    // Check if we've reached the goal
    if (distance < goal_tolerance_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal reached!");
        return cmd_vel;  // Return zero velocity
    }

    // Use target yaw directly from Vector3 z component
    double target_yaw = current_target_pose_->z;

    // Get current robot yaw
    tf2::Quaternion q_current;
    tf2::fromMsg(robot_pose.pose.orientation, q_current);
    double current_yaw = tf2::getYaw(q_current);

    // Calculate angle difference
    double angle_diff = normalize_angle(target_yaw - current_yaw);

    // Proportional control
    double linear_vel = linear_gain_ * distance;
    double angular_vel = angular_gain_ * angle_diff;

    // Apply velocity limits
    linear_vel = constrain(linear_vel, -linear_max_vel_, linear_max_vel_);
    angular_vel = constrain(angular_vel, -angular_max_vel_, angular_max_vel_);

    // If angle difference is large, prioritize rotation over translation
    if (std::abs(angle_diff) > d_pi / 4) {
        linear_vel *= 0.5;  // Reduce linear velocity when turning
    }

    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;

    RCLCPP_INFO(this->get_logger(), "Current angle: %.2f, Target angle: %.2f, Angle diff: %.2f, Distance: %.2f",
                 current_yaw, target_yaw, angle_diff, distance);

    return cmd_vel;
}

bool LocalPlanner::get_robot_pose(geometry_msgs::msg::PoseStamped& robot_pose)
{
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

double LocalPlanner::calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}


double LocalPlanner::normalize_angle(double angle)
{
    while (angle > d_pi) angle -= 2.0 * d_pi;
    while (angle < -d_pi) angle += 2.0 * d_pi;
    return angle;
}

}  // namespace local_planner

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::spin(std::make_shared<local_planner::LocalPlanner>(options));
    rclcpp::shutdown();
    return 0;
}
