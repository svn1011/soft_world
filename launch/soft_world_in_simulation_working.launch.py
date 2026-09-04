#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_share = get_package_share_directory("soft_world")

    script_path = os.path.join(
        package_share,
        "scripts",
        "load_soft_limb_in_simulation_working.py"
    )

    isaac_sim_path = os.path.expanduser(
        os.environ.get("ISAAC_SIM_PATH", "~/isaacsim")
    )
    isaac_python = os.path.join(isaac_sim_path, "python.sh")

    isaac_sim = ExecuteProcess(
        cmd=[
            isaac_python,
            script_path
        ],
        additional_env={
            "SOFT_LIMB_USD_NAME": "soft_limb_hanging_working.usd",
        },
        output="screen"
    )

    return LaunchDescription([
        isaac_sim
    ])
