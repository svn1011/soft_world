#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_share = get_package_share_directory("soft_world")
    isaac_sim_path = os.path.expanduser(
        os.environ.get("ISAAC_SIM_PATH", "~/isaacsim")
    )
    voltage_plot_path = os.path.join(
        package_share, "scripts", "plot_soft_3_working.py"
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                os.path.join(isaac_sim_path, "python.sh"),
                os.path.join(
                    package_share, "scripts",
                    "load_soft_3_working.py",
                ),
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "/usr/bin/env",
                "-u", "SNAP",
                "-u", "SNAP_LIBRARY_PATH",
                "-u", "GTK_PATH",
                "-u", "GIO_MODULE_DIR",
                "XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop",
                "MPLCONFIGDIR=/tmp/soft_3_working_matplotlib_voltage",
                "SOFT_SNAKE_PLOT_SIGNAL=bridge_voltage",
                "python3",
                voltage_plot_path,
            ],
            output="screen",
        ),
    ])
