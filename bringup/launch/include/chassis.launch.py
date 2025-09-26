import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'chassis.yaml'
    )
    # 手動制御ノードの作成
    execute_node = Node(
        package = 'bringup',
        executable = 'executor',
        parameters = [config_file_path],
        output='screen'
    )

    # joyノードの追加
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [config_file_path],
        output='screen'
    )
    # teleopノードの追加
    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        parameters = [config_file_path],
        remappings=[('cmd_vel', 'joy_vel')],
        output='screen'
    )

    launch_description = LaunchDescription()

    launch_description.add_entity(execute_node)
    launch_description.add_entity(joy_node)
    launch_description.add_entity(teleop_node)

    return launch_description
