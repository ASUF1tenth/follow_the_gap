import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_controller(context, *args, **kwargs):
    controller_pkg = LaunchConfiguration("controller").perform(context)
    launch_file = LaunchConfiguration("controller_launch_file").perform(context)

    controller_share = get_package_share_directory(controller_pkg)
    controller_launch = os.path.join(controller_share, "launch", launch_file)

    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch))]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                default_value="gap_follow",
                description="Controller package name to launch",
            ),
            DeclareLaunchArgument(
                "controller_launch_file",
                default_value="follow_the_gap.launch.py",
                description="Launch file inside selected controller package",
            ),
            OpaqueFunction(function=launch_controller),
        ]
    )
