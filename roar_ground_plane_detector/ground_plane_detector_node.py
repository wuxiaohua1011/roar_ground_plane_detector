#!/usr/bin/env python3
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


def point_cloud(points, parent_frame, stamp):
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [
        PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate("xyz")
    ]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = Header(frame_id=parent_frame)
    header.stamp = stamp
    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data,
    )


class GroundPlaneDetectorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("ground_plane_detector")

        # declare routes and get their values
        self.declare_parameter("lidar_topic", "/carla/ego_vehicle/center_lidar")
        self.declare_parameter("should_show", False)
        self.declare_parameter("parent_frame", "ego_vehicle")
        self.declare_parameter("distance_threshold", 1.0)
        self.declare_parameter("ransac_n", 10)
        self.declare_parameter("num_iterations", 1000)

        self.distance_threshold = (
            self.get_parameter("distance_threshold").get_parameter_value().double_value
        )
        self.ransac_n = (
            self.get_parameter("ransac_n").get_parameter_value().integer_value
        )
        self.num_iterations = (
            self.get_parameter("num_iterations").get_parameter_value().integer_value
        )

        self.lidar_topic = (
            self.get_parameter("lidar_topic").get_parameter_value().string_value
        )
        self.should_show = (
            self.get_parameter("should_show").get_parameter_value().bool_value
        )
        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
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
            PointCloud2, "ground_points", 10
        )
        self.obstacle_points_publisher = self.create_publisher(
            PointCloud2, "obstacle_points", 10
        )

        # helper variables
        self.bridge = CvBridge()
        if self.should_show:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(width=500, height=500)
            self.pcd = o3d.geometry.PointCloud()
            self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
            self.points_added = False

    def callback(self, center_lidar_pcl_msg):
        pcd_as_numpy_array = np.array(list(read_points(center_lidar_pcl_msg)))[:, :3]
        self.o3d_pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(pcd_as_numpy_array)
        )
        pcd = self.o3d_pcd  # .voxel_down_sample(voxel_size=2)

        # # normal removal method
        # outlier_cloud.estimate_normals()
        # normals = np.array(outlier_cloud.normals)
        # normals_avgs = np.average(normals, axis=0)
        # coords = np.where(normals[:, 2] < normals_avgs[2])
        # points = np.array(outlier_cloud.points)
        # outlier_cloud.points = o3d.utility.Vector3dVector(points[coords])
        # print(outlier_cloud)
        pcd.paint_uniform_color([0, 0, 0])
        # find plane
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.num_iterations,
        )
        colors = np.array(pcd.colors)
        colors[inliers] = [1.0, 0.0, 0.0]
        pcd.colors = o3d.utility.Vector3dVector(colors)
        # inlier_cloud = outlier_cloud.select_by_index(inliers)
        # inlier_cloud.paint_uniform_color([1.0, 0, 0])
        # outlier_cloud = outlier_cloud.select_by_index(inliers, invert=True)

        # remove things that are further and taller than a certain threshold
        # points = np.array(outlier_cloud.points)
        # coords = np.where(
        #     (points[:, 0] > 1) & (points[:, 2] < 1.5)
        # )  # further, and taller
        # points = points[coords]
        # outlier_cloud.points = o3d.utility.Vector3dVector(points)

        # outlier_cloud, ind = outlier_cloud.remove_radius_outlier(nb_points=5, radius=5)

        ground_pcd = pcd.select_by_index(inliers)
        obstacle_pcd = pcd.select_by_index(inliers, invert=True)

        ground_pc2 = point_cloud(
            np.array(ground_pcd.points),
            parent_frame=self.parent_frame,
            stamp=self.get_clock().now().to_msg(),
        )
        obstacle_pc2 = point_cloud(
            np.array(obstacle_pcd.points),
            parent_frame=self.parent_frame,
            stamp=self.get_clock().now().to_msg(),
        )

        self.ground_points_publisher.publish(ground_pc2)
        self.obstacle_points_publisher.publish(obstacle_pc2)
        if self.should_show:
            self.non_blocking_pcd_visualization(
                pcd, should_show_axis=True, axis_size=10
            )

    def non_blocking_pcd_visualization(
        self,
        pcd: o3d.geometry.PointCloud,
        should_center=False,
        should_show_axis=False,
        axis_size: float = 1,
    ):
        """
        Real time point cloud visualization.
        Args:
            pcd: point cloud to be visualized
            should_center: true to always center the point cloud
            should_show_axis: true to show axis
            axis_size: adjust axis size
        Returns:
            None
        """
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        if should_center:
            points = points - np.mean(points, axis=0)

        if self.points_added is False:
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(points)
            self.pcd.colors = o3d.utility.Vector3dVector(colors)

            if should_show_axis:
                self.coordinate_frame = (
                    o3d.geometry.TriangleMesh.create_coordinate_frame(
                        size=axis_size, origin=np.mean(points, axis=0)
                    )
                )
                self.vis.add_geometry(self.coordinate_frame)
            self.vis.add_geometry(self.pcd)
            self.points_added = True
        else:
            # print(np.shape(np.vstack((np.asarray(self.pcd.points), points))))
            self.pcd.points = o3d.utility.Vector3dVector(points)
            self.pcd.colors = o3d.utility.Vector3dVector(colors)
            if should_show_axis:
                self.coordinate_frame = (
                    o3d.geometry.TriangleMesh.create_coordinate_frame(
                        size=axis_size, origin=np.mean(points, axis=0)
                    )
                )
                self.vis.update_geometry(self.coordinate_frame)
            self.vis.update_geometry(self.pcd)

        self.vis.poll_events()
        self.vis.update_renderer()


def main(args=None):
    rclpy.init()
    node = GroundPlaneDetectorNode()
    rclpy.spin(node)


## The code below is "ported" from
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later:
# https://github.com/ros2/common_interfaces
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


if __name__ == "__main__":
    main()
