from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
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
                name="distance_threshold", default_value="0.1"
            ),
            launch.actions.DeclareLaunchArgument(name="ransac_n", default_value="3"),
            launch.actions.DeclareLaunchArgument(
                name="num_iterations", default_value="100"
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
                    {"ransac_n": launch.substitutions.LaunchConfiguration("ransac_n")},
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
                ],
            ),
        ]
    )
