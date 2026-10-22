#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_share = get_package_share_directory("soft_world")
    script_path = os.path.join(
        package_share, "scripts", "soft_quadraped_grounded_working.py"
    )
    isaac_python = os.path.expanduser("~/isaacsim/python.sh")

    return LaunchDescription([
        ExecuteProcess(
            cmd=[isaac_python, script_path],
            output="screen",
        )
    ])
