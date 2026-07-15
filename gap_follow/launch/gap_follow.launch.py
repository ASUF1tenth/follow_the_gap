import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gap_follow")
    params_path = os.path.join(pkg_share, "config", "params.yaml")

    gap_follower = Node(
        package="gap_follow",
        executable="reactive_node",
        name="gap_follower",
        output="screen",
        parameters=[params_path],
    )

    return LaunchDescription([gap_follower])
