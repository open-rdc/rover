#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import QuaternionStamped
import socket
import threading
import re
import math
from tf_transformations import quaternion_from_euler

class NmeaTcpDriverNode(Node):
    def __init__(self):
        super().__init__('nmea_tcp_driver_node')

        # パラメータの宣言
        self.declare_parameter('port', 1111)
        self.declare_parameter('frame_id', 'gps')

        # パラメータの取得
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # パブリッシャーの作成
        self.navsatfix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.quaternion_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/heading', 10)

        # TCPサーバーの初期化
        self.server_socket = None
        self.client_socket = None
        self.is_running = False

        # TCPサーバーの開始
        self.start_tcp_server()

        self.get_logger().info(f'NMEA TCP Driver Node started on port {self.port}')

    def start_tcp_server(self):
        """TCPサーバーを開始する"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(1)
            self.is_running = True

            self.get_logger().info(f'TCP server listening on port {self.port}')

            # 別スレッドでクライアント接続を待機
            server_thread = threading.Thread(target=self.accept_connections)
            server_thread.daemon = True
            server_thread.start()

        except Exception as e:
            self.get_logger().error(f'Failed to start TCP server: {e}')

    def accept_connections(self):
        """クライアント接続を受け入れる"""
        while self.is_running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f'Client connected from {addr}')

                # クライアントからのデータを処理
                self.handle_client()

            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f'Error accepting connection: {e}')

    def handle_client(self):
        """クライアントからのデータを処理する"""
        buffer = ""

        try:
            while self.is_running and self.client_socket:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data
                lines = buffer.split('\n')
                buffer = lines[-1]  # 最後の不完全な行をバッファに保持

                for line in lines[:-1]:
                    line = line.strip()
                    if line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                        self.parse_rmc(line)

        except Exception as e:
            self.get_logger().error(f'Error handling client data: {e}')
        finally:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                self.get_logger().info('Client disconnected')

    def parse_rmc(self, rmc_sentence):
        """RMCセンテンスを解析してNavSatFixとQuaternionStampedを発行する"""
        try:
            # RMCセンテンスの形式: $GNRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir,mode,checksum
            parts = rmc_sentence.split(',')

            if len(parts) < 12:
                return

            # ステータスチェック（'A'=有効, 'V'=無効）
            status = parts[2]
            if status != 'A':
                return

            # 緯度の解析
            lat_str = parts[3]
            lat_dir = parts[4]
            if not lat_str or not lat_dir:
                return

            # 緯度をdddmm.mmmmm形式からdecimal degreesに変換
            lat_degrees = int(lat_str[:2])
            lat_minutes = float(lat_str[2:])
            latitude = lat_degrees + lat_minutes / 60.0
            if lat_dir == 'S':
                latitude = -latitude

            # 経度の解析
            lon_str = parts[5]
            lon_dir = parts[6]
            if not lon_str or not lon_dir:
                return

            # 経度をdddmm.mmmmm形式からdecimal degreesに変換
            lon_degrees = int(lon_str[:3])
            lon_minutes = float(lon_str[3:])
            longitude = lon_degrees + lon_minutes / 60.0
            if lon_dir == 'W':
                longitude = -longitude

            # コース（方位角）の解析
            course_str = parts[8]
            if not course_str:
                return
            course = float(course_str)

            # タイムスタンプの取得
            timestamp = self.get_clock().now().to_msg()

            # NavSatFixメッセージの作成と発行
            navsatfix_msg = NavSatFix()
            navsatfix_msg.header.stamp = timestamp
            navsatfix_msg.header.frame_id = self.frame_id
            navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
            navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
            navsatfix_msg.latitude = latitude
            navsatfix_msg.longitude = longitude
            navsatfix_msg.altitude = 0.0 # float('NaN')
            navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            # navsatfix_msg.position_covariance = [100.0] * 9

            self.navsatfix_pub.publish(navsatfix_msg)

            # QuaternionStampedメッセージの作成と発行
            quaternion_msg = QuaternionStamped()
            quaternion_msg.header.stamp = timestamp
            quaternion_msg.header.frame_id = self.frame_id

            # コース角（真北からの角度）をクォータニオンに変換
            # NMEAのコースは真北からの時計回り角度（degrees）
            yaw_radians = math.radians(course)
            quaternion = quaternion_from_euler(0, 0, yaw_radians)

            quaternion_msg.quaternion.x = quaternion[0]
            quaternion_msg.quaternion.y = quaternion[1]
            quaternion_msg.quaternion.z = quaternion[2]
            quaternion_msg.quaternion.w = quaternion[3]

            self.quaternion_pub.publish(quaternion_msg)

            # IMUメッセージの作成と発行（/headingと同じデータ）
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = self.frame_id

            # オリエンテーション（/headingと同じクォータニオン）
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            # # オリエンテーション共分散（Z軸（ヨー）以外は不明）
            # imu_msg.orientation_covariance[0] = -1  # X軸ロール - 不明
            # imu_msg.orientation_covariance[4] = -1  # Y軸ピッチ - 不明
            # imu_msg.orientation_covariance[8] = 0.1  # Z軸ヨー - 既知（適当な小さな値）

            # # 角速度と線形加速度は利用できないため0に設定し、共分散を-1に設定
            # imu_msg.angular_velocity_covariance[0] = -1
            # imu_msg.linear_acceleration_covariance[0] = -1

            self.imu_pub.publish(imu_msg)

            self.get_logger().info(f'Published: Lat={latitude:.6f}, Lon={longitude:.6f}, Course={course:.2f}°')

        except Exception as e:
            self.get_logger().error(f'Error parsing RMC sentence: {e}')

    def destroy_node(self):
        """ノードの終了処理"""
        self.is_running = False

        if self.client_socket:
            self.client_socket.close()

        if self.server_socket:
            self.server_socket.close()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = NmeaTcpDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
