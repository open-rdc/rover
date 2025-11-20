#!/usr/bin/env python3
"""
ROS 2 node (rclpy) that subscribes to /low_scan (sensor_msgs/LaserScan)
and prints the closest valid range, its index, angle, and x,y coordinates
in the scan's frame.

Usage:
  - Source your ROS 2 and workspace setup scripts.
  - Run: python3 /home/kazuma/colcon_ws/src/scripts/print_closest_scan.py

This script is intentionally minimal so you can run it directly from the
workspace without packaging it as a ROS 2 component.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ClosestPointNode(Node):
    def __init__(self):
        super().__init__('closest_point_printer')
        # Subscribe to /low_scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            10,
        )
        # Prevent unused variable warning
        self.subscription
        self.get_logger().info('closest_point_printer started; subscribed to scan')

    def scan_callback(self, msg: LaserScan):
        # Convert to list for safe indexing
        ranges = list(msg.ranges)

        # Filter valid ranges: finite and within sensor's reported bounds
        valid = []  # list of tuples (index, range)
        for i, r in enumerate(ranges):
            if r is None:
                continue
            # math.isfinite handles inf and nan
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valid.append((i, float(r)))

        if not valid:
            self.get_logger().info('No valid ranges in incoming LaserScan')
            return

        # Find the minimum range among valid points
        idx, min_r = min(valid, key=lambda x: x[1])

        # Compute corresponding angle and Cartesian coordinates
        angle = msg.angle_min + idx * msg.angle_increment
        x = min_r * math.cos(angle)
        y = min_r * math.sin(angle)

        # Print a concise one-line summary
        self.get_logger().info(
            f"Closest range: {min_r:.3f} m  (index={idx}, angle={angle:.3f} rad) "
            f"-> x={x:.3f} m, y={y:.3f} m  frame={msg.header.frame_id}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ClosestPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
