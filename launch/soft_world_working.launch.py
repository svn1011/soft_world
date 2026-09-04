#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from soft_world.robot_description_working import build_robot_description


def generate_launch_description():

    package_share = get_package_share_directory("soft_world")

    script_path = os.path.join(
        package_share,
        "scripts",
        "load_soft_limb_working.py"
    )
    strain_plot_path = os.path.join(
        package_share,
        "scripts",
        "plot_strain_gauges_working.py",
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="soft_limb_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": build_robot_description(),
        }],
    )

    map_to_limb = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="soft_limb_map_transform",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "map", "soft_limb_disk_00",
        ],
    )

    # VS Code is installed as a Snap on this machine. Its integrated terminal
    # exports Core20 GTK/GIO paths that are ABI-incompatible with Ubuntu 22.04
    # RViz. Start RViz with only those injected paths removed.
    rviz = ExecuteProcess(
        cmd=[
            "/usr/bin/env",
            "-u", "SNAP",
            "-u", "SNAP_LIBRARY_PATH",
            "-u", "GTK_PATH",
            "-u", "GIO_MODULE_DIR",
            "XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop",
            "rviz2",
        ],
        output="screen",
    )

    strain_plots = ExecuteProcess(
        cmd=[
            "/usr/bin/env",
            "-u", "SNAP",
            "-u", "SNAP_LIBRARY_PATH",
            "-u", "GTK_PATH",
            "-u", "GIO_MODULE_DIR",
            "XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop",
            "MPLCONFIGDIR=/tmp/soft_world_matplotlib",
            "python3",
            strain_plot_path,
        ],
        output="screen",
    )

    return LaunchDescription([
        map_to_limb,
        robot_state_publisher,
        isaac_sim,
        rviz,
        strain_plots,
    ])
