#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import socket
import threading
# import re
import math

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
                    if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                        self.parse_gga(line)

        except Exception as e:
            self.get_logger().error(f'Error handling client data: {e}')
        finally:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                self.get_logger().info('Client disconnected')

    def parse_gga(self, gga_sentence):
        """GGAセンテンスを解析してNavSatFixを発行する"""
        try:
            # GGAセンテンスの形式: $GNGGA,time,lat,lat_dir,lon,lon_dir,quality,num_sats,hdop,alt,alt_units,geoid_height,geoid_units,dgps_time,dgps_id,checksum
            parts = gga_sentence.split(',')

            if len(parts) < 15:
                return

            # GPS品質インジケーターをチェック（0=無効, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float, 6=推測航法, etc.）
            quality = parts[6]
            if not quality or quality == '0':
                return

            # 緯度の解析
            lat_str = parts[2]
            lat_dir = parts[3]
            if not lat_str or not lat_dir:
                return

            # 緯度をddmm.mmmmm形式からdecimal degreesに変換
            lat_degrees = int(lat_str[:2])
            lat_minutes = float(lat_str[2:])
            latitude = lat_degrees + lat_minutes / 60.0
            if lat_dir == 'S':
                latitude = -latitude

            # 経度の解析
            lon_str = parts[4]
            lon_dir = parts[5]
            if not lon_str or not lon_dir:
                return

            # 経度をdddmm.mmmmm形式からdecimal degreesに変換
            lon_degrees = int(lon_str[:3])
            lon_minutes = float(lon_str[3:])
            longitude = lon_degrees + lon_minutes / 60.0
            if lon_dir == 'W':
                longitude = -longitude

            # 高度の解析
            alt_str = parts[9]
            altitude = 0.0
            if alt_str:
                altitude = float(alt_str)

            # 衛星数とHDOPの解析
            num_sats_str = parts[7]
            hdop_str = parts[8]
            num_sats = int(num_sats_str) if num_sats_str else 0
            hdop = float(hdop_str) if hdop_str else 99.99

            # タイムスタンプの取得
            timestamp = self.get_clock().now().to_msg()

            # NavSatFixメッセージの作成と発行
            navsatfix_msg = NavSatFix()
            navsatfix_msg.header.stamp = timestamp
            navsatfix_msg.header.frame_id = self.frame_id

            # GPS品質に基づいてステータスを設定
            if quality == '1':
                navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
            elif quality == '2':
                navsatfix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif quality in ['4', '5']:  # RTK
                navsatfix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                navsatfix_msg.status.status = NavSatStatus.STATUS_FIX

            navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
            navsatfix_msg.latitude = latitude
            navsatfix_msg.longitude = longitude
            navsatfix_msg.altitude = altitude
            navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # HDOPとGPS品質に基づいて位置共分散を設定
            # 基準誤差をGPS品質に応じて調整
            if quality == '4':  # RTK Fixed
                base_error = 0.1  # 10cm
            elif quality == '5':  # RTK Float
                base_error = 1.0  # 1m
            elif quality == '2':  # DGPS
                base_error = 2.0  # 2m
            elif quality == '1':  # GPS
                base_error = 3.0  # 3m
            else:  # その他
                base_error = 10.0  # 10m（精度不明時は保守的に）

            # HDOP値による精度劣化を考慮
            horizontal_variance = (hdop * base_error) ** 2
            vertical_variance = horizontal_variance * 4  # 高度は水平位置より精度が低い

            # # HDOPが大きすぎる場合（>5.0）は信頼度を大幅に下げる
            # if hdop > 5.0:
            #     horizontal_variance *= 4  # さらに4倍
            #     vertical_variance *= 4
            # elif hdop > 2.0:
            #     horizontal_variance *= 2  # 2倍
            #     vertical_variance *= 2

            navsatfix_msg.position_covariance = [
                horizontal_variance, 0.0, 0.0,
                0.0, horizontal_variance, 0.0,
                0.0, 0.0, vertical_variance
            ]

            self.navsatfix_pub.publish(navsatfix_msg)

            self.get_logger().info(f'Published: Lat={latitude:.6f}, Lon={longitude:.6f}, Alt={altitude:.2f}m, Sats={num_sats}, HDOP={hdop:.2f}, Quality={quality}')

        except Exception as e:
            self.get_logger().error(f'Error parsing GGA sentence: {e}')

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
