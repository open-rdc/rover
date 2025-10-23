import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_include_path = os.path.join(
        get_package_share_directory('bringup'),
        'launch', 'include'
    )

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/chassis.launch.py'])
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/description.launch.py'])
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/adi_imu.launch.py'])
    )
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/robot_localization.launch.py'])
    )
    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/navsat.launch.py'])
    )

    launch_description = LaunchDescription()

    launch_description.add_action(chassis_launch)
    launch_description.add_action(description_launch)
    launch_description.add_action(imu_launch)
    launch_description.add_action(robot_localization_launch)
    launch_description.add_action(navsat_launch)

    return launch_description
