#!/usr/bin/env python3

"""
pc2_to_scan.py - A ROS 2 node that subscribes to a PointCloud2 topic and publishes a corresponding LaserScan topic.

This script subscribes to a PointCloud2 topic and converts the point cloud data to a LaserScan message, which it publishes to a new topic.

Author: Joshua Greineder
Date created: March 17, 2023
Last modified: March 19, 2023
Python Version: 3.8

Usage:
    1. Start a ROS 2 system.
    2. Run this node with the command: ros2 run Cam_Filter pc2_to_scan.py

ROS 2 Subscriptions:
    /point_cloud: PointCloud2 message with the unfiltered point cloud data.

ROS 2 Publishers:
    /laser_scan: LaserScan message with the filtered scan data.

ROS 2 Parameters:
    None.

Description of functionality:
The node subscribes to a PointCloud2 topic and extracts the point cloud data as a numpy array. It then converts the point cloud
data to a LaserScan message using the provided scan parameters (angle_min, angle_max, angle_increment, range_min, range_max, 
and time_increment). The range values of the LaserScan message are computed from the point cloud data by taking the Euclidean 
norm of the x, y, and z coordinates of each point. The resulting LaserScan message is published to a new topic.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2 as pc2
import numpy as np


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('point_cloud_to_laser_scan_node')

        # Set up subscription to PointCloud2 topic
        self.create_subscription(
            PointCloud2,
            '/combined_point_cloud',
            self.callback,
            10
        )

        # Set up publisher for LaserScan topic
        self.pub = self.create_publisher(
            LaserScan,
            '/laser_scan',
            10
        )

        # Set up scan parameters
        self.angle_min = -np.pi / 2
        self.angle_max = np.pi / 2
        self.angle_increment = np.pi / 180
        self.range_min = 0.0
        self.range_max = 5.0
        self.time_increment = 0.0

    def callback(self, msg):
        # Extract point cloud data as numpy array
        pc = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

        # Convert point cloud to laser scan
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.time_increment = self.time_increment
        scan.ranges = np.linalg.norm(pc[:, :3], axis=1)

        # Publish laser scan
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = PointCloudToLaserScanNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
