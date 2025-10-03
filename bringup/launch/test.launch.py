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

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/description.launch.py'])
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_include_path, '/adi_imu.launch.py'])
    )

    launch_description = LaunchDescription()

    launch_description.add_entity(description_launch)
    launch_description.add_entity(imu_launch)

    return launch_description
