#ifndef LOCAL_PLANNER__DWA_NODE_HPP_
#define LOCAL_PLANNER__DWA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <memory>

namespace local_planner
{

class DWA : public rclcpp::Node
{
public:
    DWA(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    DWA(const std::string& name_space, const rclcpp::NodeOptions& options);

private:
    void target_pose_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void control_timer_callback();

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Target pose
    geometry_msgs::msg::Vector3::SharedPtr current_target_pose_;

    // Parameters (const members initialized in constructor)
    const double linear_max_vel_;
    const double angular_max_vel_;
    const double control_frequency_;
    bool get_robot_pose(geometry_msgs::msg::PoseStamped& robot_pose);
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__DWA_NODE_HPP_
