#ifndef LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <memory>
#include <cmath>

namespace local_planner
{

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    LocalPlanner(const std::string& name_space, const rclcpp::NodeOptions& options);

private:
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void control_timer_callback();

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Target pose
    geometry_msgs::msg::PoseStamped::SharedPtr current_target_pose_;

        // Parameters (const members initialized in constructor)
    const double linear_max_vel_;
    const double angular_max_vel_;
    const double linear_gain_;
    const double angular_gain_;
    const double goal_tolerance_;
    const double control_frequency_;
    geometry_msgs::msg::Twist calculate_cmd_vel();
    bool get_robot_pose(geometry_msgs::msg::PoseStamped& robot_pose);
    double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    double calculate_angle_difference(double target_yaw, double current_yaw);
    double normalize_angle(double angle);
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_
