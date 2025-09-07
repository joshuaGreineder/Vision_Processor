#!/usr/bin/env python3

"""
point_cloud_combiner.py - Combines filtered point clouds from multiple cameras into a single point cloud.

This node subscribes to filtered point cloud topics from multiple cameras and combines them into a single point cloud
message. The combined point cloud message is published to the '/combined_point_cloud' topic.

Author: Joshua Greineder
Date created: March 10, 2023
Last modified: March 19, 2023
Python Version: 3.8

Usage:
    $ ros2 run Cam_Filter point_cloud_combiner.py

ROS 2 topics:
    Subscribed:
        - '/filtered_point_cloud/{camera_name}': Filtered point cloud from {camera_name}
        - For each camera

    Published:
        - '/combined_point_cloud': Combined point cloud from all cameras

ROS 2 Parameters:
    None.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
import numpy as np


class PointCloudCombinerNode(Node):
    def __init__(self):
        super().__init__('point_cloud_combiner_node')

        # Set up subscriptions to filtered point cloud topics
        self.subs = []
        camera_names = ['camera1', 'camera2', 'camera3']
        for camera_name in camera_names:
            sub = self.create_subscription(
                PointCloud2,
                '/filtered_point_cloud/' + camera_name,
                self.callback,
                10
            )
            self.subs.append(sub)

        # Set up publisher for combined point cloud
        self.pub = self.create_publisher(
            PointCloud2,
            '/combined_point_cloud',
            10
        )

    def callback(self, msg):
        # Extract point cloud data as numpy array
        pc = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

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
            points=pc
        )

        # Convert numpy array to PointCloud2 message
        msg_out = pc2.numpy_to_pointcloud2(pc, msg.header.stamp, msg.header.frame_id)

        # Publish combined point cloud
        self.pub.publish(msg_out)


def main():
    rclpy.init()
    node = PointCloudCombinerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
