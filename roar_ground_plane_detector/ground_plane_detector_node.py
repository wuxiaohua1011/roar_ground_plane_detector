#!/usr/bin/env python3
from itertools import starmap
import rclpy
import rclpy.node
import sys
import cv2
import numpy as np
from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from ctypes import *  # convert float to uint32
from std_msgs.msg import Header
from .utils import *
import time


class GroundPlaneDetectorNode(rclpy.node.Node):
    """Given point cloud, this node will seperate ground plane and obstacle

    Args:
        rclpy (_type_): rclpy node
    """

    def __init__(self):
        super().__init__("ground_plane_detector")
        # declare routes and get their values
        self.declare_parameter("lidar_topic", "/carla/ego_vehicle/center_lidar")
        self.declare_parameter("distance_threshold", 0.1)
        self.declare_parameter("ransac_n", 3)
        self.declare_parameter("num_iterations", 1000)
        self.lidar_topic = (
            self.get_parameter("lidar_topic").get_parameter_value().string_value
        )
        # route definitions
        self.subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.callback,
            10,
        )
        self.subscription  # prevent unused variable warning
        self.ground_points_publisher = self.create_publisher(
            PointCloud2, f"{self.lidar_topic}/ground_points", 10
        )
        self.obstacle_points_publisher = self.create_publisher(
            PointCloud2, f"{self.lidar_topic}/obstacle_points", 10
        )

        self.distance_threshold = (
            self.get_parameter("distance_threshold").get_parameter_value().double_value
        )
        self.ransac_n = (
            self.get_parameter("ransac_n").get_parameter_value().integer_value
        )
        self.num_iterations = (
            self.get_parameter("num_iterations").get_parameter_value().integer_value
        )
        self.get_logger().info(f"distance_threshold: {self.distance_threshold}")
        self.get_logger().info(f"ransac_n: {self.ransac_n}")
        self.get_logger().info(f"num_iterations: {self.num_iterations}")
        self.get_logger().info(
            f"Publishing Ground Points at {self.ground_points_publisher.topic}"
        )
        self.get_logger().info(
            f"Publishing Obstacle Points at {self.obstacle_points_publisher.topic}"
        )

    def callback(self, lidar_msg: PointCloud2):
        pcd_as_numpy_array = pointcloud2_to_array(lidar_msg)[:, :3]  # ignore intensity
        o3d_pcd: o3d.geometry.PointCloud = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(pcd_as_numpy_array)
        )

        plane_model, inliers = o3d_pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.num_iterations,
        )

        ground_pcd = o3d_pcd.select_by_index(inliers)
        obstacle_pcd = o3d_pcd.select_by_index(inliers, invert=True)

        ground_pc2 = array_to_xyz_pointcloud2f(
            np.array(ground_pcd.points, dtype=np.float32),
            frame_id=lidar_msg.header.frame_id,
            stamp=lidar_msg.header.stamp,
        )
        obstacle_pc2 = array_to_xyz_pointcloud2f(
            np.array(obstacle_pcd.points, dtype=np.float32),
            frame_id=lidar_msg.header.frame_id,
            stamp=lidar_msg.header.stamp,
        )

        self.ground_points_publisher.publish(ground_pc2)
        self.obstacle_points_publisher.publish(obstacle_pc2)


def main(args=None):
    rclpy.init()
    node = GroundPlaneDetectorNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
