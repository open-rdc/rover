import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # bringupパッケージのディレクトリを取得
    bringup_dir = get_package_share_directory("bringup")
    # 将来的にmapvizの設定ファイルを作成する場合のパス
    mapviz_config_file = os.path.join(bringup_dir, "config", "mapviz.mvc")

    return launch.LaunchDescription([
        # Mapvizノード - GPS可視化ツール
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            # 設定ファイルがある場合は以下のコメントアウトを解除
            parameters=[{"config": mapviz_config_file}]
        ),
        # WGS84とUTM間の変換を提供（Mapvizのタイルマップに必要）
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame": "odom"},
                {"local_xy_origin": "auto"},
                {"local_xy_navsatfix_topic": "/gps/fix"}
            ]
        )
    ])
