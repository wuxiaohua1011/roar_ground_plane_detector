from asyncio import base_subprocess
from email.mime import base

from sqlalchemy import true
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
import launch


def generate_launch_description():
    base_path = os.path.realpath(
        get_package_share_directory("roar_ground_plane_detector")
    )

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="lidar_topic", default_value="/carla/ego_vehicle/center_lidar"
            ),
            launch.actions.DeclareLaunchArgument(
                name="should_show", default_value="False"
            ),
            launch.actions.DeclareLaunchArgument(
                name="parent_frame", default_value="ego_vehicle"
            ),
            launch.actions.DeclareLaunchArgument(
                name="distance_threshold", default_value="1.0"
            ),
            launch.actions.DeclareLaunchArgument(name="ransac_n", default_value="10"),
            launch.actions.DeclareLaunchArgument(
                name="num_iterations", default_value="1000"
            ),
            launch.actions.DeclareLaunchArgument(
                name="initial_voxel_size", default_value="0.5"
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_distance", default_value="100.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="display_axis_scale", default_value="10.0"
            ),
            Node(
                package="roar_ground_plane_detector",
                executable="ground_plane_detector_node",
                name="ground_plane_detector_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "lidar_topic": launch.substitutions.LaunchConfiguration(
                            "lidar_topic"
                        )
                    },
                    {
                        "should_show": launch.substitutions.LaunchConfiguration(
                            "should_show"
                        )
                    },
                    {
                        "parent_frame": launch.substitutions.LaunchConfiguration(
                            "parent_frame"
                        )
                    },
                    {
                        "distance_threshold": launch.substitutions.LaunchConfiguration(
                            "distance_threshold"
                        )
                    },
                    {
                        "num_iterations": launch.substitutions.LaunchConfiguration(
                            "num_iterations"
                        )
                    },
                    {
                        "initial_voxel_size": launch.substitutions.LaunchConfiguration(
                            "initial_voxel_size"
                        )
                    },
                    {
                        "max_distance": launch.substitutions.LaunchConfiguration(
                            "max_distance"
                        )
                    },
                    {
                        "display_axis_scale": launch.substitutions.LaunchConfiguration(
                            "display_axis_scale"
                        )
                    },
                ],
            ),
        ]
    )
