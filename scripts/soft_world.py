#!/usr/bin/env python3

from isaacsim import SimulationApp

# Start Isaac Sim with GUI
simulation_app = SimulationApp({
    "headless": False
})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCylinder
import numpy as np


def main():

    # --------------------------------------------------
    # Create simulation world
    # --------------------------------------------------
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0
    )

    # --------------------------------------------------
    # Ground plane
    # --------------------------------------------------
    world.scene.add_default_ground_plane()

    # --------------------------------------------------
    # Soft robot disk
    #
    # Diameter  = 4 cm = 0.04 m
    # Radius    = 2 cm = 0.02 m
    # Thickness = 0.5 cm = 0.005 m
    # --------------------------------------------------

    disk = world.scene.add(
        DynamicCylinder(
            prim_path="/World/SoftDisk",
            name="soft_disk",

            position=np.array([
                0.0,
                0.0,
                0.02
            ]),

            radius=0.02,
            height=0.005,

            mass=0.01
        )
    )

    # --------------------------------------------------
    # Reset physics
    # --------------------------------------------------
    world.reset()

    print("--------------------------------")
    print("SOFT WORLD STARTED")
    print("--------------------------------")
    print("Disk diameter  : 4 cm")
    print("Disk radius    : 2 cm")
    print("Disk thickness : 0.5 cm")
    print("Prim path      : /World/SoftDisk")
    print("--------------------------------")

    # --------------------------------------------------
    # Simulation loop
    # --------------------------------------------------
    while simulation_app.is_running():

        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()