import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'adi_imu.yaml'
    )


    # adis_rcv_csv_nodeの作成
    adis_rcv_csv_node = Node(
        package='adi_imu_tr_driver_ros2',
        executable='adis_rcv_csv_node',
        output='screen',
        parameters=
        [{
            "device": "/dev/ttyIMU",
            "frame_id": "base_link",
            # "parent_id": "odom",
            "rate": 100.0,
            "mode": "Register"
        }], # ノードを指定してパラメタを書くと読み込まない
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=
        [{
            "use_mag": False,
            "publish_tf": False
        }],
    )

    launch_description = LaunchDescription()

    launch_description.add_action(adis_rcv_csv_node)
    launch_description.add_action(imu_filter_node)

    return launch_description
