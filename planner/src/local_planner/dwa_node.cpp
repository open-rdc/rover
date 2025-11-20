#include "local_planner/dwa_node.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <limits>
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
control_frequency_(get_parameter("control_frequency").as_double()),
dt_(1.0 / control_frequency_),
predict_time_(get_parameter("predict_time").as_double()),
linear_acceleration_(get_parameter("linear_acceleration").as_double()),
angular_acceleration_(get_parameter("angular_acceleration").as_double()),
linear_resolution_(get_parameter("linear_resolution").as_double()),
angular_resolution_(get_parameter("angular_resolution").as_double()),
robot_radius_(get_parameter("robot_radius").as_double()),
heading_gain_(get_parameter("heading_gain").as_double()),
obstacle_gain_(get_parameter("obstacle_gain").as_double()),
velocity_gain_(get_parameter("velocity_gain").as_double())
{
    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ROS interfaces
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "target_pose", 10,
        std::bind(&DWA::target_pose_callback, this, std::placeholders::_1));

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_filtered", 10,
        std::bind(&DWA::laser_scan_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Control timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&DWA::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "DWA node initialized: freq=%.1fHz, radius=%.2fm, predict_time=%.2fs",
                control_frequency_, robot_radius_, predict_time_);
}

void DWA::target_pose_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
    current_target_pose_ = std::make_unique<geometry_msgs::msg::Vector3>(*msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "New target: (%.2f, %.2f)", msg->x, msg->y);
}

void DWA::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    current_laser_scan_ = msg;
}

void DWA::timer_callback(){
    if (!current_target_pose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for target pose");
        return;
    }
    if(!current_laser_scan_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for laser scan data");
        return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!get_robot_pose(robot_pose)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Unable to get robot pose");
        return;
    }

    // Calculate optimal velocity using DWA
    auto cmd_vel = calculate_optimal_velocity(robot_pose);
    current_velocity_ = cmd_vel;
    cmd_vel_pub_->publish(cmd_vel);
}

std::vector<std::pair<double, double>> DWA::calculate_dynamic_window(double current_linear, double current_angular){
    std::vector<std::pair<double, double>> velocities;

    // Calculate velocity limits based on maximum velocities
    double min_linear = -linear_max_vel_;
    double max_linear = linear_max_vel_;
    double min_angular = -angular_max_vel_;
    double max_angular = angular_max_vel_;

    // Apply acceleration constraints
    double linear_accel_limit = linear_acceleration_ * dt_;
    double angular_accel_limit = angular_acceleration_ * dt_;

    min_linear = std::max(min_linear, current_linear - linear_accel_limit);
    max_linear = std::min(max_linear, current_linear + linear_accel_limit);
    min_angular = std::max(min_angular, current_angular - angular_accel_limit);
    max_angular = std::min(max_angular, current_angular + angular_accel_limit);

    // Generate velocity candidates
    for (double v = min_linear; v <= max_linear; v += linear_resolution_) {
        for (double w = min_angular; w <= max_angular; w += angular_resolution_) {
            velocities.emplace_back(v, w);
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Generated %zu velocity candidates", velocities.size());
    return velocities;
}

double DWA::calculate_heading_cost(double linear, double angular, const geometry_msgs::msg::PoseStamped& robot_pose){
    double current_yaw = tf2::getYaw(robot_pose.pose.orientation);

    // Calculate direct angle to target from current position
    double target_x = current_target_pose_->x;
    double target_y = current_target_pose_->y;
    double target_dx = target_x - robot_pose.pose.position.x;
    double target_dy = target_y - robot_pose.pose.position.y;

    // Direct angle to target
    double target_angle = atan2(target_dy, target_dx);

    // Predicted robot orientation after this command
    double predicted_yaw = current_yaw + angular * predict_time_;

    // Angle difference between predicted heading and target direction
    double angle_diff = normalize_angle(target_angle - predicted_yaw);

    return std::abs(angle_diff);
}

double DWA::calculate_obstacle_cost(double linear, double angular, const geometry_msgs::msg::PoseStamped& robot_pose){

    double min_distance = std::numeric_limits<double>::max();

    // Simulate trajectory and find minimum distance to obstacles
    double current_angle = tf2::getYaw(robot_pose.pose.orientation);
    double current_x = robot_pose.pose.position.x;
    double current_y = robot_pose.pose.position.y;

    for (double t = 0; t <= predict_time_; t += dt_) {
        current_angle += angular * dt_;
        current_x += linear * cos(current_angle) * dt_;
        current_y += linear * sin(current_angle) * dt_;

        // Check distance to all valid laser points
        for (size_t i = 0; i < current_laser_scan_->ranges.size(); ++i) {
            double range = current_laser_scan_->ranges[i];

            // Skip invalid or too-close readings
            if (std::isnan(range) || std::isinf(range) ||
                range < current_laser_scan_->range_min ||
                range > current_laser_scan_->range_max ||
                range < 0.05) { // Skip very close readings
                continue;
            }

            // Calculate obstacle position in world coordinates
            double angle = current_laser_scan_->angle_min + i * current_laser_scan_->angle_increment;
            double obs_x = robot_pose.pose.position.x + range * cos(tf2::getYaw(robot_pose.pose.orientation) + angle);
            double obs_y = robot_pose.pose.position.y + range * sin(tf2::getYaw(robot_pose.pose.orientation) + angle);

            // Calculate distance from predicted robot position to obstacle
            double distance = sqrt(pow(current_x - obs_x, 2) + pow(current_y - obs_y, 2));
            min_distance = std::min(min_distance, distance);
        }
    }

    if (min_distance == std::numeric_limits<double>::max()) {
        return 0.0; // No obstacles detected
    }

    // Return inverse cost: closer obstacles = higher cost, normalized to [0,1]
    return std::max(0.0, 1.0 - (min_distance / 2.0));
}

double DWA::calculate_velocity_cost(double linear, double angular){
    // Prefer moderate forward velocity
    double normalized_linear = std::abs(linear) / linear_max_vel_;

    // Penalize backward motion more heavily
    double direction_penalty = (linear > 0) ? 1.0 : 2.0;

    // Penalize high angular velocities
    double normalized_angular = std::abs(angular) / angular_max_vel_;

    // Combined cost: balance forward motion preference with smooth movement
    return direction_penalty * (1.0 - normalized_linear * 0.8) + normalized_angular * 0.5;
}

bool DWA::check_collision(double linear, double angular, const geometry_msgs::msg::PoseStamped& robot_pose){
    // Simulate trajectory and check for collisions
    double current_angle = tf2::getYaw(robot_pose.pose.orientation);
    double current_x = robot_pose.pose.position.x;
    double current_y = robot_pose.pose.position.y;

    for (double t = 0; t <= predict_time_; t += dt_) {
        current_angle += angular * dt_;
        current_x += linear * cos(current_angle) * dt_;
        current_y += linear * sin(current_angle) * dt_;

        // Check distance to all laser scan points
        for (size_t i = 0; i < current_laser_scan_->ranges.size(); ++i) {
            double range = current_laser_scan_->ranges[i];

            // Skip invalid readings
            if (std::isnan(range) || std::isinf(range) ||
                range < current_laser_scan_->range_min ||
                range > current_laser_scan_->range_max ||
                range < 0.05) { // Skip very close readings (5cm minimum)
                continue;
            }

            // Calculate obstacle position
            double angle = current_laser_scan_->angle_min + i * current_laser_scan_->angle_increment;
            double obs_x = robot_pose.pose.position.x + range * cos(tf2::getYaw(robot_pose.pose.orientation) + angle);
            double obs_y = robot_pose.pose.position.y + range * sin(tf2::getYaw(robot_pose.pose.orientation) + angle);

            // Check if predicted robot position collides with obstacle
            double distance = sqrt(pow(current_x - obs_x, 2) + pow(current_y - obs_y, 2));
            if (distance < robot_radius_) {
                return true; // Collision detected
            }
        }
    }

    return false; // No collision
}

geometry_msgs::msg::Twist DWA::calculate_optimal_velocity(const geometry_msgs::msg::PoseStamped& robot_pose){
    auto velocities = calculate_dynamic_window(current_velocity_.linear.x, current_velocity_.angular.z);

    double best_cost = std::numeric_limits<double>::max();
    geometry_msgs::msg::Twist best_cmd;
    best_cmd.linear.x = 0.0;
    best_cmd.angular.z = 0.0;

    int valid_candidates = 0;

    // If no candidates, return zero velocity
    if (velocities.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No velocity candidates generated");
        return best_cmd;
    }

    for (const auto& vel : velocities) {
        double linear = vel.first;
        double angular = vel.second;

        // Skip if collision detected
        if (check_collision(linear, angular, robot_pose)) {
            continue;
        }

        valid_candidates++;

        // Calculate costs
        double heading_cost = calculate_heading_cost(linear, angular, robot_pose);
        double obstacle_cost = calculate_obstacle_cost(linear, angular, robot_pose);
        double velocity_cost = calculate_velocity_cost(linear, angular);

        // Total cost calculation
        double total_cost = heading_gain_ * heading_cost +
                           obstacle_gain_ * obstacle_cost +
                           velocity_gain_ * velocity_cost;

        // Update best velocity if this is better
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_cmd.linear.x = linear;
            best_cmd.angular.z = angular;
        }
    }

    // Log summary information
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "DWA: %d/%zu valid candidates, selected v=%.2f w=%.2f (cost=%.3f)",
                         valid_candidates, velocities.size(),
                         best_cmd.linear.x, best_cmd.angular.z, best_cost);

    // Safety check
    if (best_cost == std::numeric_limits<double>::max()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "All velocity candidates rejected due to collisions");
    }

    return best_cmd;
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
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Transform unavailable: %s", ex.what());
        return false;
    }
}

double DWA::normalize_angle(double angle){
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace local_planner

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::spin(std::make_shared<local_planner::DWA>(options));
    rclcpp::shutdown();
    return 0;
}
