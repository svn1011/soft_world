"""Load the shared configuration without importing scripts/soft_world.py.

That legacy filename shadows the installed package when a script is executed
directly, and importing it would incorrectly start a second Isaac application.
"""

import os
import sys

_original_path = sys.path[:]
_script_directory = os.path.dirname(os.path.abspath(__file__))
try:
    sys.path[:] = [path for path in sys.path
                   if os.path.abspath(path or os.getcwd()) != _script_directory]
    from soft_world.limb_configuration_working import (
        DISK_COUNT, JOINT_COUNT, TIP_DISK_INDEX, DISK_PATHS,
        interval_bend, validate_stage,
    )
finally:
    sys.path[:] = _original_path
