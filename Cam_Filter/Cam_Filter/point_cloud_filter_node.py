#!/usr/bin/env python3

"""
point_cloud_filter_node.py - A ROS 2 node that filters a point cloud from a camera by removing points with
white color and points at a certain height.

This script subscribes to the depth/color/points topic from a specified camera, filters the point cloud data,
and publishes the filtered point cloud to a new topic.

Author: Joshua Greineder
Date created: August 26, 2022
Last modified: March 19, 2023
Python Version: 3.8

Usage:
    1. Start a ROS 2 system.
    2. Run this node with the command: ros2 run Cam_Filter point_cloud_filter_node.py

ROS 2 Subscriptions:
    /{camera_name}/depth/color/points: PointCloud2 message with the unfiltered point cloud data.

ROS 2 Publishers:
    /filtered_point_cloud/{camera_name}: PointCloud2 message with the filtered point cloud data.

ROS 2 Parameters:
    {camera_name}/white_threshold (int): Minimum RGB value for a point to be considered not white (default 240).
    {camera_name}/height_threshold (float): Height at which to filter points (default 0.5).
    {camera_name}/height_tolerance (float): Tolerance for height filtering (default 0.05).

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
import numpy as np
import argparse


class PointCloudFilterNode(Node):
    def __init__(self, camera_name):
        super().__init__('point_cloud_filter_node_' + camera_name)
        self.camera_name = camera_name

        # Set up subscription to camera topic
        self.create_subscription(
            PointCloud2,
            '/' + self.camera_name + '/depth/color/points',
            self.callback,
            10
        )

        # Set up publisher for filtered point cloud
        self.pub = self.create_publisher(
            PointCloud2,
            '/filtered_point_cloud/' + self.camera_name,
            10
        )

        # Set up filter parameters
        self.white_threshold = 240
        self.height_threshold = 0.5
        self.height_tolerance = 0.05

    def callback(self, msg):
        # Extract point cloud data as numpy array
        pc = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

        # Filter white points
        white_mask = np.logical_and.reduce((pc[:, 0] >= self.white_threshold, pc[:, 1] >= self.white_threshold, pc[:, 2] >= self.white_threshold))
        white_points = pc[white_mask]

        # Filter points at half meter height
        height_mask = np.abs(pc[:, 2] - self.height_threshold) < self.height_tolerance
        height_points = pc[height_mask]

        # Combine filtered point clouds
        filtered_points = np.vstack((white_points, height_points))

        # Create new point cloud message with filtered points
        msg_out = pc2.create_cloud(
            header=msg.header,
            fields=[
                # Add XYZ coordinates
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # Add RGB color
                PointField('rgb', 12, PointField.FLOAT32, 1)
            ],
            points=filtered_points
        )

        # Convert numpy array to PointCloud2 message
        msg_out = pc2.numpy_to_pointcloud2(filtered_points, msg.header.stamp, msg.header.frame_id)

        # Publish filtered point cloud
        self.pub.publish(msg_out)


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Filter point cloud data from cameras.')
    parser.add_argument('--white-threshold', type=int, default=240, help='White threshold for filtering points.')
    parser.add_argument('--height-threshold', type=float, default=0.5, help='Height threshold for filtering points.')
    parser.add_argument('--height-tolerance', type=float, default=0.05, help='Height tolerance for filtering points.')
    args = parser.parse_args()

    rclpy.init()
    # Create nodes for each camera
    camera_names = ['camera1']
    nodes = [PointCloudFilterNode(camera_name, args.white_threshold, args.height_threshold, args.height_tolerance) for camera_name in camera_names]
    rclpy.spin(nodes)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
