import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'laser_filters.yaml'
    )

    laser_filters_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[config_file_path],
        remappings=[('scan', 'low_scan')],
    )

    launch_description = LaunchDescription()
    launch_description.add_entity(laser_filters_node)
    return launch_description
