#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3
import tf2_ros
import tf2_geometry_msgs
import csv
import os
import math
from rclpy.parameter import Parameter

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')

        # パラメータの宣言
        self.declare_parameter('mode', 'waypoint_recorder')  # 'waypoint_recorder' or 'path_planner'
        self.declare_parameter('waypoint_file', 'waypoints.csv')
        self.declare_parameter('joy_button_index', 0)  # ジョイスティックのボタンインデックス
        self.declare_parameter('waypoint_threshold', 2.0)  # ウェイポイント到達判定の閾値[m]

        # パラメータの取得
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        waypoint_filename = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.joy_button_index = self.get_parameter('joy_button_index').get_parameter_value().integer_value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').get_parameter_value().double_value

        # ウェイポイントファイルのフルパスを構築（このパッケージのソースディレクトリのconfigから取得）
        try:
            # 現在のファイルからパッケージのルートディレクトリを取得
            current_file_path = os.path.abspath(__file__)
            # global_planner_node.py -> src/global_planner/global_planner_node.py -> src -> package_root
            package_root = os.path.dirname(os.path.dirname(os.path.dirname(current_file_path)))
            self.waypoint_file = os.path.join(package_root, 'config', waypoint_filename)
            self.get_logger().info(f"Using waypoint file: {self.waypoint_file}")
        except Exception as e:
            self.get_logger().warn(f"Could not determine package path, using relative path: {e}")
            self.waypoint_file = os.path.join('config', waypoint_filename)

        # TFバッファとリスナーの初期化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 状態変数
        self.previous_joy_button_state = 0
        self.waypoints = []
        self.current_waypoint_index = 0

        # モードに応じた初期化
        if self.mode == 'waypoint_recorder':
            self.setup_waypoint_recorder()
        elif self.mode == 'path_planner':
            self.setup_path_planner()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            return

        self.get_logger().info(f"Global Planner Node started in {self.mode} mode")

    def setup_waypoint_recorder(self):
        """ウェイポイント記録モードの初期化"""
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.get_logger().info("Waypoint recorder mode initialized")

    def setup_path_planner(self):
        """パスプランナーモードの初期化"""
        # ターゲットポーズのパブリッシャーを作成
        self.target_pose_publisher = self.create_publisher(
            Vector3,
            'target_pose',
            10
        )

        # ジョイスティックのサブスクリプションを作成
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback_planner,
            10
        )

        # パスプランナーの状態変数
        self.planner_started = False

        self.load_waypoints()
        self.timer = self.create_timer(0.1, self.planner_callback)  # 10Hz
        self.get_logger().info(f"Path planner mode initialized with {len(self.waypoints)} waypoints")
        self.get_logger().info("Press joy button to start path following")

    def joy_callback(self, msg):
        """ジョイスティックのコールバック（ウェイポイント記録用）"""
        if len(msg.buttons) <= self.joy_button_index:
            return

        current_button_state = msg.buttons[self.joy_button_index]

        # ボタンが押された瞬間を検出（立ち上がりエッジ）
        if current_button_state == 1 and self.previous_joy_button_state == 0:
            self.record_current_position()

        self.previous_joy_button_state = current_button_state

    def joy_callback_planner(self, msg):
        """ジョイスティックのコールバック（パスプランナー用）"""
        if len(msg.buttons) <= self.joy_button_index:
            return

        current_button_state = msg.buttons[self.joy_button_index]

        # ボタンが押された瞬間を検出（立ち上がりエッジ）
        if current_button_state == 1 and self.previous_joy_button_state == 0:
            if not self.planner_started:
                self.planner_started = True
                self.get_logger().info("Path following started!")
            else:
                # 既に開始している場合は停止
                self.planner_started = False
                self.current_waypoint_index = 0  # リセット
                self.get_logger().info("Path following stopped and reset!")

        self.previous_joy_button_state = current_button_state

    def record_current_position(self):
        """現在位置をウェイポイントとして記録"""
        try:
            # utm -> base_link の変換を取得
            transform = self.tf_buffer.lookup_transform(
                'utm', 'base_link', rclpy.time.Time()
            )

            # 位置を取得
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # CSVファイルに保存
            self.save_waypoint_to_csv(x, y)

            self.get_logger().info(f"Waypoint recorded: x={x:.2f}, y={y:.2f}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get transform: {e}")

    def save_waypoint_to_csv(self, x, y):
        """ウェイポイントをCSVファイルに保存"""
        # ディレクトリが存在しない場合は作成
        waypoint_dir = os.path.dirname(self.waypoint_file)
        if not os.path.exists(waypoint_dir):
            os.makedirs(waypoint_dir)
            self.get_logger().info(f"Created directory: {waypoint_dir}")

        file_exists = os.path.isfile(self.waypoint_file)

        with open(self.waypoint_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists:
                writer.writerow(['x', 'y'])  # ヘッダー
            writer.writerow([x, y])

    def load_waypoints(self):
        """CSVファイルからウェイポイントを読み込み"""
        if not os.path.isfile(self.waypoint_file):
            self.get_logger().error(f"Waypoint file {self.waypoint_file} not found")
            return

        self.waypoints = []
        try:
            with open(self.waypoint_file, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    waypoint = {
                        'x': float(row['x']),
                        'y': float(row['y'])
                    }
                    self.waypoints.append(waypoint)

            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}")

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")

    def planner_callback(self):
        """パスプランナーのメインループ"""
        if not self.waypoints:
            return

        # パスプランナーが開始されていない場合は何もしない
        if not self.planner_started:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            # 最後のウェイポイントに到達したら、最後のウェイポイントを継続してパブリッシュ
            if len(self.waypoints) > 0:
                last_waypoint = self.waypoints[-1]
                self.publish_target_pose(last_waypoint)
                self.get_logger().info("All waypoints reached. Holding at last waypoint.")
            return

        try:
            # 現在位置を取得
            transform = self.tf_buffer.lookup_transform(
                'utm', 'base_link', rclpy.time.Time()
            )

            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y

            # 現在のターゲットウェイポイント
            target_waypoint = self.waypoints[self.current_waypoint_index]

            # ターゲットポーズをパブリッシュ
            self.publish_target_pose(target_waypoint)

            # 距離を計算
            distance = math.sqrt(
                (current_x - target_waypoint['x'])**2 +
                (current_y - target_waypoint['y'])**2
            )

            # ウェイポイントに到達したかチェック
            if distance < self.waypoint_threshold:
                self.get_logger().info(
                    f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} "
                    f"(x={target_waypoint['x']:.2f}, y={target_waypoint['y']:.2f})"
                )
                self.current_waypoint_index += 1
            else:
                # 現在のターゲットまでの距離を定期的に表示（5秒おき）
                if hasattr(self, '_last_distance_log_time'):
                    if self.get_clock().now().nanoseconds - self._last_distance_log_time > 5e9:  # 5秒
                        self.get_logger().info(
                            f"Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}, "
                            f"distance: {distance:.2f}m"
                        )
                        self._last_distance_log_time = self.get_clock().now().nanoseconds
                else:
                    self._last_distance_log_time = self.get_clock().now().nanoseconds

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get transform: {e}")

    def publish_target_pose(self, waypoint):
        """ターゲットウェイポイントをVector3メッセージとしてパブリッシュ"""
        target_pose_msg = Vector3()

        # x, y座標を設定（zは0.0のまま）
        target_pose_msg.x = float(waypoint['x'])
        target_pose_msg.y = float(waypoint['y'])
        target_pose_msg.z = 0.0

        # パブリッシュ
        self.target_pose_publisher.publish(target_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    global_planner_node = GlobalPlannerNode()

    try:
        rclpy.spin(global_planner_node)
    except KeyboardInterrupt:
        pass

    global_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
