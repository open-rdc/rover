#!/usr/bin/env python3
"""
Robot trajectory plotter from rosbag (Simple version)
Reads TF transforms from rosbag and visualizes robot trajectory.
Large XY trajectory display without arrows or speed analysis.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import math

class TrajectoryPlotter:
    def __init__(self):
        self.positions = []
        self.timestamps = []
        self.orientations = []
        
    def _quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to euler angle (yaw)"""
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw
    
    def read_rosbag(self, bag_path):
        """Read TF data from rosbag"""
        print(f"Loading rosbag: {bag_path}")
        
        # Handle rosbag2 file pattern
        db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
        if not db_files:
            print("Error: No .db3 files found")
            return False
        
        db_path = os.path.join(bag_path, db_files[0])
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Get both /tf and /tf_static topics
        cursor.execute("SELECT id, name, type FROM topics WHERE name = '/tf' OR name = '/tf_static'")
        topics = cursor.fetchall()
        
        if not topics:
            print("Error: /tf or /tf_static topic not found")
            return False
        
        print(f"Found topics: {[t[1] for t in topics]}")
        
        utm_to_odom = None
        
        # Process all topics
        for topic_info in topics:
            topic_id = topic_info[0]
            topic_name = topic_info[1]
            msg_type = get_message(topic_info[2])
            
            cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))
            messages = cursor.fetchall()
            
            print(f"Processing {len(messages)} messages from {topic_name}...")
            
            for timestamp, data in messages:
                try:
                    msg = deserialize_message(data, msg_type)
                    
                    for transform in msg.transforms:
                        # Save UTM->ODOM transform (from tf_static)
                        if (transform.header.frame_id == 'utm' and transform.child_frame_id == 'odom'):
                            utm_to_odom = transform
                            print(f"Got UTM->ODOM transform: x={transform.transform.translation.x:.2f}, y={transform.transform.translation.y:.2f}")
                        
                        # Process ODOM->base_link transforms (from tf)
                        elif (transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link'):
                            x = transform.transform.translation.x
                            y = transform.transform.translation.y
                            z = transform.transform.translation.z
                            
                            # Transform to UTM coordinates (if available)
                            if utm_to_odom:
                                utm_x = utm_to_odom.transform.translation.x
                                utm_y = utm_to_odom.transform.translation.y
                                utm_z = utm_to_odom.transform.translation.z
                                
                                final_x = utm_x + x
                                final_y = utm_y + y
                                final_z = utm_z + z
                            else:
                                final_x = x
                                final_y = y
                                final_z = z
                            
                            yaw = self._quaternion_to_euler(
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w
                            )
                            
                            self.positions.append([final_x, final_y, final_z])
                            self.timestamps.append(timestamp)
                            self.orientations.append(yaw)
                            
                except Exception as e:
                    continue
        
        conn.close()
        
        if not self.positions:
            print("Error: No position data found")
            return False
            
        coordinate_system = "UTM" if utm_to_odom else "ODOM"
        print(f"Got {len(self.positions)} position data points ({coordinate_system} coordinates)")
        return True
    
    def plot_trajectory(self, save_path=None):
        """Plot trajectory with large XY display only"""
        if not self.positions:
            print("Error: No data to plot")
            return
            
        positions = np.array(self.positions)
        x_coords = positions[:, 0]
        y_coords = positions[:, 1]
        z_coords = positions[:, 2]
        
        # Create figure with single large XY plot
        fig, ax = plt.subplots(1, 1, figsize=(16, 12))
        fig.suptitle('Robot Trajectory (UTM/ODOM Coordinates)', fontsize=20)
        
        # Large XY trajectory plot
        scatter = ax.scatter(x_coords, y_coords, c=range(len(x_coords)), 
                            cmap='viridis', s=25, alpha=0.8)
        ax.plot(x_coords, y_coords, 'b-', alpha=0.4, linewidth=2)
        
        # Mark start and goal with large markers
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=20, label='Start', 
                markeredgecolor='white', markeredgewidth=3)
        ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=20, label='Goal',
                markeredgecolor='white', markeredgewidth=3)
        
        ax.set_xlabel('X (m)', fontsize=18)
        ax.set_ylabel('Y (m)', fontsize=18)
        ax.set_title('XY Trajectory', fontsize=22, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=16)
        ax.set_aspect('equal')
        ax.tick_params(labelsize=14)
        
        # Enhanced colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Time Progress', fontsize=16)
        cbar.ax.tick_params(labelsize=14)
        
        plt.tight_layout()
        
        # Print basic statistics
        self.print_statistics(positions)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved: {save_path}")
        
        plt.show()
    
    def print_statistics(self, positions):
        """Print basic trajectory statistics"""
        print("\n" + "="*50)
        print("         TRAJECTORY STATISTICS")
        print("="*50)
        print(f"Total data points: {len(positions):,}")
        
        if len(positions) > 1:
            distances = np.sqrt(np.diff(positions[:, 0])**2 + np.diff(positions[:, 1])**2)
            total_distance = np.sum(distances)
            straight_distance = np.sqrt((positions[-1, 0] - positions[0, 0])**2 + 
                                      (positions[-1, 1] - positions[0, 1])**2)
            
            print(f"Total distance traveled: {total_distance:.2f} m")
            print(f"Straight-line distance: {straight_distance:.2f} m") 
            print(f"Path efficiency: {straight_distance/total_distance*100:.1f}%")
            
            # Coordinate ranges
            x_range = np.max(positions[:, 0]) - np.min(positions[:, 0])
            y_range = np.max(positions[:, 1]) - np.min(positions[:, 1])
            
            print(f"\nCoordinate ranges:")
            print(f"  X: {np.min(positions[:, 0]):.2f} to {np.max(positions[:, 0]):.2f} m (range: {x_range:.2f} m)")
            print(f"  Y: {np.min(positions[:, 1]):.2f} to {np.max(positions[:, 1]):.2f} m (range: {y_range:.2f} m)")
            
            print("="*50)

def main():
    parser = argparse.ArgumentParser(description='Plot robot trajectory (simple version)')
    parser.add_argument('rosbag_path', help='Path to rosbag file')
    parser.add_argument('--save', '-s', help='Save file name')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.rosbag_path):
        print(f"Rosbag not found: {args.rosbag_path}")
        sys.exit(1)
    
    plotter = TrajectoryPlotter()
    if not plotter.read_rosbag(args.rosbag_path):
        sys.exit(1)
    
    plotter.plot_trajectory(args.save)

if __name__ == '__main__':
    main()
