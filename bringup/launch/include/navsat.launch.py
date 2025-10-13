import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'navsat.yaml'
    )

    # nmea_tcpclient_driverノードの作成
    nmea_serial_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_tcpclient_driver',
        parameters=[config_file_path],
        remappings=[('fix', 'gps/fix')],
        output='screen'
    )

    # navsat_transform_nodeの作成
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        parameters=[config_file_path],
        remappings=[
            ('/imu', '/imu/data'),
            ('/odometry/filtered', '/odometry/local')
        ],
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_entity(nmea_serial_driver_node)
    launch_description.add_entity(navsat_transform_node)
    return launch_description
