#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("soft_world")
    base_launch = os.path.join(
        package_share, "launch", "soft_3_working.launch.py"
    )
    controller = os.path.join(
        package_share, "scripts", "control_soft_3_circle_working.py"
    )
    radius = LaunchConfiguration("circle_radius_m")
    height = LaunchConfiguration("circle_height_m")
    period = LaunchConfiguration("circle_period_s")
    settle = LaunchConfiguration("settle_seconds")
    return LaunchDescription([
        DeclareLaunchArgument(
            "circle_radius_m", default_value="0.2125",
            description=(
                "Horizontal tip-circle radius in metres; the default is half "
                "the 0.425 m base-to-tip centerline length."
            ),
        ),
        DeclareLaunchArgument(
            "circle_height_m", default_value="0.40",
            description="Tip-circle world Z height in metres.",
        ),
        DeclareLaunchArgument(
            "circle_period_s", default_value="54.0",
            description=(
                "Seconds per complete revolution; 54 s keeps approximately "
                "the original tangential tip speed at the larger radius."
            ),
        ),
        DeclareLaunchArgument(
            "settle_seconds", default_value="6.0",
            description="Initial hold time before circular motion begins.",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
        ExecuteProcess(cmd=[
            "python3", controller, "--ros-args",
            "-p", ["circle_radius_m:=", radius],
            "-p", ["circle_height_m:=", height],
            "-p", ["circle_period_s:=", period],
            "-p", ["settle_seconds:=", settle],
        ], output="screen"),
    ])
