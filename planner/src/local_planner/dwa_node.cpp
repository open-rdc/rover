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
velocity_gain_(get_parameter("velocity_gain").as_double()),
min_turning_radius_(get_parameter("min_turning_radius").as_double()),
min_linear_vel_for_turn_(get_parameter("min_linear_vel_for_turn").as_double()),
goal_tolerance_(get_parameter("goal_tolerance").as_double()),
heading_angle_weight_(get_parameter("heading_angle_weight").as_double()),
obstacle_margin_(get_parameter("obstacle_margin").as_double()),
obstacle_predict_time_(get_parameter("obstacle_predict_time").as_double())
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

    double goal_distance = std::hypot(
        current_target_pose_->x - robot_pose.pose.position.x,
        current_target_pose_->y - robot_pose.pose.position.y);

    if (goal_distance < goal_tolerance_) {
        geometry_msgs::msg::Twist stop_cmd;
        current_velocity_ = stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // Calculate optimal velocity using DWA
    auto cmd_vel = calculate_optimal_velocity(robot_pose);
    current_velocity_ = cmd_vel;
    cmd_vel_pub_->publish(cmd_vel);
}

std::vector<geometry_msgs::msg::Point> DWA::extract_obstacle_points(const geometry_msgs::msg::PoseStamped& robot_pose) const{
    std::vector<geometry_msgs::msg::Point> obstacles;
    if (!current_laser_scan_) {
        return obstacles;
    }

    obstacles.reserve(current_laser_scan_->ranges.size());
    const double base_yaw = tf2::getYaw(robot_pose.pose.orientation);

    for (size_t i = 0; i < current_laser_scan_->ranges.size(); ++i) {
        const double range = current_laser_scan_->ranges[i];

        if (std::isnan(range) || std::isinf(range) ||
            range < current_laser_scan_->range_min ||
            range > current_laser_scan_->range_max ||
            range < 0.05) {
            continue;
        }

        geometry_msgs::msg::Point obstacle;
        const double angle = current_laser_scan_->angle_min + static_cast<double>(i) * current_laser_scan_->angle_increment;
        obstacle.x = robot_pose.pose.position.x + range * std::cos(base_yaw + angle);
        obstacle.y = robot_pose.pose.position.y + range * std::sin(base_yaw + angle);
        obstacle.z = robot_pose.pose.position.z;
        obstacles.push_back(obstacle);
    }

    return obstacles;
}

Trajectory DWA::simulate_trajectory(double linear, double angular, const geometry_msgs::msg::PoseStamped& robot_pose, const std::vector<geometry_msgs::msg::Point>& obstacles) const{
    Trajectory trajectory;
    trajectory.linear_vel = linear;
    trajectory.angular_vel = angular;

    double yaw = tf2::getYaw(robot_pose.pose.orientation);
    double x = robot_pose.pose.position.x;
    double y = robot_pose.pose.position.y;

    const double collision_threshold = robot_radius_ + obstacle_margin_;
    const double heading_horizon = predict_time_;
    const double obstacle_horizon = std::max(obstacle_predict_time_, heading_horizon);

    double accumulated_time = 0.0;
    bool heading_state_recorded = (heading_horizon <= 0.0);

    trajectory.heading_position.x = robot_pose.pose.position.x;
    trajectory.heading_position.y = robot_pose.pose.position.y;
    trajectory.heading_position.z = robot_pose.pose.position.z;
    trajectory.heading_yaw = yaw;
    trajectory.final_position = trajectory.heading_position;
    trajectory.final_yaw = yaw;

    trajectory.path.reserve(static_cast<size_t>(obstacle_horizon / dt_) + 1);

    while (accumulated_time < obstacle_horizon) {
        const double remaining_time = obstacle_horizon - accumulated_time;
        const double step = std::min(dt_, remaining_time);

        const double previous_x = x;
        const double previous_y = y;
        const double previous_yaw = yaw;

        yaw += angular * step;
        x += linear * std::cos(yaw) * step;
        y += linear * std::sin(yaw) * step;

        geometry_msgs::msg::Point waypoint;
        waypoint.x = x;
        waypoint.y = y;
        waypoint.z = robot_pose.pose.position.z;
        trajectory.path.push_back(waypoint);

        if (!heading_state_recorded && accumulated_time + step >= heading_horizon) {
            const double overshoot = accumulated_time + step - heading_horizon;
            double ratio = (step - overshoot) / std::max(step, 1e-6);
            ratio = std::clamp(ratio, 0.0, 1.0);
            const double interpolated_x = previous_x + ratio * (x - previous_x);
            const double interpolated_y = previous_y + ratio * (y - previous_y);
            const double interpolated_yaw = previous_yaw + ratio * (yaw - previous_yaw);

            trajectory.heading_position.x = interpolated_x;
            trajectory.heading_position.y = interpolated_y;
            trajectory.heading_position.z = robot_pose.pose.position.z;
            trajectory.heading_yaw = interpolated_yaw;
            heading_state_recorded = true;
        }

        for (const auto& obstacle : obstacles) {
            const double distance = std::hypot(x - obstacle.x, y - obstacle.y);
            if (distance < trajectory.min_distance) {
                trajectory.min_distance = distance;
            }
            if (distance <= collision_threshold) {
                trajectory.collision = true;
                break;
            }
        }

        accumulated_time += step;

        if (trajectory.collision) {
            break;
        }
    }

    if (!heading_state_recorded) {
        trajectory.heading_position.x = x;
        trajectory.heading_position.y = y;
        trajectory.heading_position.z = robot_pose.pose.position.z;
        trajectory.heading_yaw = yaw;
    }

    trajectory.final_position.x = x;
    trajectory.final_position.y = y;
    trajectory.final_position.z = robot_pose.pose.position.z;
    trajectory.final_yaw = yaw;

    return trajectory;
}

std::vector<std::pair<double, double>> DWA::calculate_dynamic_window(double current_linear, double current_angular){
    std::vector<std::pair<double, double>> velocities;

    double min_linear = -linear_max_vel_;
    double max_linear = linear_max_vel_;
    double min_angular = -angular_max_vel_;
    double max_angular = angular_max_vel_;

    const double linear_accel_limit = linear_acceleration_ * dt_;
    const double angular_accel_limit = angular_acceleration_ * dt_;

    min_linear = std::max(min_linear, current_linear - linear_accel_limit);
    max_linear = std::min(max_linear, current_linear + linear_accel_limit);
    min_angular = std::max(min_angular, current_angular - angular_accel_limit);
    max_angular = std::min(max_angular, current_angular + angular_accel_limit);

    for (double v = min_linear; v <= max_linear + 1e-6; v += linear_resolution_) {
        for (double w = min_angular; w <= max_angular + 1e-6; w += angular_resolution_) {
            velocities.emplace_back(v, w);
        }
    }

    if (velocities.empty()) {
        velocities.emplace_back(0.0, 0.0);
    }

    RCLCPP_DEBUG(this->get_logger(), "Generated %zu velocity candidates", velocities.size());
    return velocities;
}

double DWA::calculate_heading_cost(const Trajectory& trajectory, const geometry_msgs::msg::PoseStamped& robot_pose){
    const double target_x = current_target_pose_->x;
    const double target_y = current_target_pose_->y;

    const double dist_to_goal = std::hypot(target_x - trajectory.heading_position.x,
                                          target_y - trajectory.heading_position.y);
    const double current_dist = std::hypot(target_x - robot_pose.pose.position.x,
                                           target_y - robot_pose.pose.position.y);
    const double progress = current_dist - dist_to_goal;

    const double heading_angle = std::atan2(target_y - trajectory.heading_position.y,
                                            target_x - trajectory.heading_position.x);
    const double angle_diff = normalize_angle(heading_angle - trajectory.heading_yaw);

    double cost = dist_to_goal + heading_angle_weight_ * std::abs(angle_diff);

    if (progress > 0.0) {
        cost = std::max(0.0, cost - progress);
    } else {
        cost += std::abs(progress);
    }

    return cost;
}

double DWA::calculate_obstacle_cost(const Trajectory& trajectory){
    if (trajectory.collision) {
        return std::numeric_limits<double>::infinity();
    }

    if (!std::isfinite(trajectory.min_distance)) {
        return 0.0;
    }

    const double clearance = trajectory.min_distance - robot_radius_;
    if (clearance <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    return 1.0 / (clearance + obstacle_margin_);
}

double DWA::calculate_velocity_cost(double linear, double angular, const Trajectory& trajectory){
    static_cast<void>(trajectory);

    const double normalized_linear = std::clamp(std::abs(linear) / linear_max_vel_, 0.0, 1.0);
    const double normalized_angular = std::clamp(std::abs(angular) / angular_max_vel_, 0.0, 1.0);

    const double linear_limit = std::max(linear_acceleration_ * dt_, 1e-3);
    const double angular_limit = std::max(angular_acceleration_ * dt_, 1e-3);

    const double smoothness = (std::abs(linear - current_velocity_.linear.x) / linear_limit) +
                              (std::abs(angular - current_velocity_.angular.z) / angular_limit);

    double cost = (1.0 - normalized_linear) + 0.6 * normalized_angular + 0.3 * smoothness;

    if (linear < 0.0) {
        cost += 1.0;
    }

    return cost;
}

bool DWA::check_collision(const Trajectory& trajectory){
    return trajectory.collision;
}

geometry_msgs::msg::Twist DWA::calculate_optimal_velocity(const geometry_msgs::msg::PoseStamped& robot_pose){
    auto velocities = calculate_dynamic_window(current_velocity_.linear.x, current_velocity_.angular.z);
    auto obstacles = extract_obstacle_points(robot_pose);

    double best_cost = std::numeric_limits<double>::max();
    geometry_msgs::msg::Twist best_cmd;
    best_cmd.linear.x = 0.0;
    best_cmd.angular.z = 0.0;

    size_t feasible_candidates = 0;

    if (velocities.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No velocity candidates generated");
        return best_cmd;
    }

    for (const auto& vel : velocities) {
        const double linear = vel.first;
        const double angular = vel.second;

        const double abs_angular = std::abs(angular);
        // Enforce the minimum forward motion needed for the non-holonomic drivetrain
        if (abs_angular > 1e-6) {
            if (std::abs(linear) < min_linear_vel_for_turn_) {
                continue;
            }
            const double max_allowed_angular = std::abs(linear) / std::max(min_turning_radius_, 1e-3);
            if (abs_angular > max_allowed_angular) {
                continue;
            }
        }

        auto trajectory = simulate_trajectory(linear, angular, robot_pose, obstacles);
        if (check_collision(trajectory)) {
            continue;
        }

        double obstacle_cost = calculate_obstacle_cost(trajectory);
        if (!std::isfinite(obstacle_cost)) {
            continue;
        }

        double heading_cost = calculate_heading_cost(trajectory, robot_pose);
        double velocity_cost = calculate_velocity_cost(linear, angular, trajectory);

        const double total_cost = heading_gain_ * heading_cost +
                                  obstacle_gain_ * obstacle_cost +
                                  velocity_gain_ * velocity_cost;

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_cmd.linear.x = linear;
            best_cmd.angular.z = angular;
        }

        feasible_candidates++;
    }

    const size_t total_samples = velocities.size();
    if (feasible_candidates > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "DWA: %zu/%zu feasible candidates, selected v=%.2f w=%.2f (cost=%.3f)",
                             feasible_candidates, total_samples,
                             best_cmd.linear.x, best_cmd.angular.z, best_cost);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "All velocity candidates rejected by kinematic or collision checks");
    }

    if (best_cost == std::numeric_limits<double>::max()) {
        best_cmd.linear.x = 0.0;
        best_cmd.angular.z = 0.0;
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
