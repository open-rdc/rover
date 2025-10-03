import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'ekf.yaml'
    )

    # EKFフィルターノードの作成
    ekf_filter_node_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[config_file_path],
        remappings=[("odometry/filtered", "odometry/local")]
    )
    ekf_filter_node_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[config_file_path],
        remappings=[("odometry/filtered", "odometry/global")]
    )

    launch_description = LaunchDescription()
    launch_description.add_entity(ekf_filter_node_odom)
    launch_description.add_entity(ekf_filter_node_map)

    return launch_description
