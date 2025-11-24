#ifndef LOCAL_PLANNER__DWA_NODE_HPP_
#define LOCAL_PLANNER__DWA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <memory>
#include <vector>
#include <limits>

namespace local_planner
{

struct DynamicWindow {
    double min_linear;
    double max_linear;
    double min_angular;
    double max_angular;
};

struct Trajectory {
    double linear_vel{0.0};
    double angular_vel{0.0};
    double cost{0.0};
    double min_distance{std::numeric_limits<double>::infinity()};
    geometry_msgs::msg::Point heading_position{};
    double heading_yaw{0.0};
    geometry_msgs::msg::Point final_position{};
    double final_yaw{0.0};
    bool collision{false};
    std::vector<geometry_msgs::msg::Point> path;
};

class DWA : public rclcpp::Node
{
public:
    DWA(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    DWA(const std::string& name_space, const rclcpp::NodeOptions& options);

private:
    void target_pose_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Target pose
    geometry_msgs::msg::Vector3::SharedPtr current_target_pose_;

    // Laser scan data
    sensor_msgs::msg::LaserScan::SharedPtr current_laser_scan_;

    // Current velocity (for dynamic window calculation)
    geometry_msgs::msg::Twist current_velocity_;

    // Parameters (const members initialized in constructor)
    const double linear_max_vel_;
    const double angular_max_vel_;
    const double control_frequency_;

    // DWA parameters
    const double dt_;
    const double predict_time_;
    const double linear_acceleration_;
    const double angular_acceleration_;
    const double linear_resolution_;
    const double angular_resolution_;
    const double robot_radius_;
    const double heading_gain_;
    const double obstacle_gain_;
    const double velocity_gain_;
    const double min_turning_radius_;
    const double min_linear_vel_for_turn_;
    const double goal_tolerance_;
    const double heading_angle_weight_;
    const double obstacle_margin_;
    const double obstacle_predict_time_;

    // Helper functions
    bool get_robot_pose(geometry_msgs::msg::PoseStamped& robot_pose);
    double normalize_angle(double angle);
    std::vector<geometry_msgs::msg::Point> extract_obstacle_points(const geometry_msgs::msg::PoseStamped& robot_pose) const;
    Trajectory simulate_trajectory(double linear, double angular, const geometry_msgs::msg::PoseStamped& robot_pose, const std::vector<geometry_msgs::msg::Point>& obstacles) const;

    // DWA functions
    std::vector<std::pair<double, double>> calculate_dynamic_window(double current_linear, double current_angular);
    double calculate_heading_cost(const Trajectory& trajectory, const geometry_msgs::msg::PoseStamped& robot_pose);
    double calculate_obstacle_cost(const Trajectory& trajectory);
    double calculate_velocity_cost(double linear, double angular, const Trajectory& trajectory);
    bool check_collision(const Trajectory& trajectory);
    geometry_msgs::msg::Twist calculate_optimal_velocity(const geometry_msgs::msg::PoseStamped& robot_pose);

};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__DWA_NODE_HPP_
