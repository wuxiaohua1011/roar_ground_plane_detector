from asyncio import base_subprocess
from email.mime import base

from sqlalchemy import true
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path


def generate_launch_description():
    base_path = os.path.realpath(
        get_package_share_directory("roar_ground_plane_detector")
    )

    node = Node(
        package="roar_ground_plane_detector",
        executable="ground_plane_detector_node",
        name="ground_plane_detector_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"lidar_topic": "/carla/ego_vehicle/center_lidar"},
            {"should_show": True},
        ],
    )
    return LaunchDescription([node])
