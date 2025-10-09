import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    planner_config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'planner.yaml'
    )

    chassis_config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'chassis.yaml'
    )

    global_planner_node = Node(
        package='planner',
        executable='global_planner_node.py',
        name='global_planner_node',
        parameters=[planner_config_file_path],
        output='screen'
    )

    local_planner_node = Node(
        package='planner',
        executable='local_planner_node',
        name='local_planner_node',
        parameters=[planner_config_file_path, chassis_config_file_path],
        output='screen'
    )

    launch_description = LaunchDescription()

    launch_description.add_entity(global_planner_node)
    launch_description.add_entity(local_planner_node)

    return launch_description
