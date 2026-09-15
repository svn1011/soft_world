#!/usr/bin/env python3

"""Regenerate six-disc assets through Isaac Sim's fully initialized schemas."""

import os
from pathlib import Path
import subprocess


def main():
    """Generate both six-disc variants, preserving the twelve-disc backups."""
    isaac_directory = Path(
        os.environ.get("ISAAC_SIM_PATH", str(Path.home() / "isaacsim"))
    ).expanduser()
    isaac_python = isaac_directory / "python.sh"
    if not isaac_python.is_file():
        raise FileNotFoundError(
            f"Isaac Python not found: {isaac_python}; set ISAAC_SIM_PATH"
        )

    script_directory = Path(__file__).resolve().parent
    export_environment = os.environ.copy()
    export_environment["SOFT_LIMB_EXPORT_ONLY"] = "1"
    # Bare USD flattening lacks Isaac's registered metadata/schema context:
    # the earlier renderer-free exports passed structural checks but crashed
    # during World initialization. Use the native generators instead.
    for script_name in ("soft_limb_hang_working.py", "soft_limb_working.py"):
        subprocess.run(
            [str(isaac_python), str(script_directory / script_name)],
            env=export_environment,
            check=True,
        )


if __name__ == "__main__":
    main()
