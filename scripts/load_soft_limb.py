#!/usr/bin/env python3

"""Load and simulate the saved dual-D6 soft-limb USD stage."""

import os

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx import get_physx_interface
from pxr import PhysxSchema, UsdGeom, UsdPhysics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_COUNT = 11
MAX_ABS_BEND_POSITION_DEGREES = 25.0
MAX_TARGET_SPEED_DEGREES_PER_SECOND = 45.0
MAX_ABS_TORQUE = 1_000_000.0
MAX_TORQUE_SLEW_RATE = 2_000_000.0
BEND_STIFFNESS = 1000000
BEND_DAMPING = 100
MAX_BEND_DEGREES = 25.0
SOFT_LIMB_USD_NAME = os.environ.get(
    "SOFT_LIMB_USD_NAME", "soft_limb_position_control.usd"
)
PHYSICS_FREQUENCY_HZ = 240.0
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 8


def joint_paths():
    """Yield the base joint followed by both joints in each later pair."""
    yield "/World/SoftLimb/Joints/Pair_00_Base_D6Joint"
    for index in range(1, JOINT_COUNT):
        yield f"/World/SoftLimb/Joints/Pair_{index:02d}_D6A"
        yield f"/World/SoftLimb/Joints/Pair_{index:02d}_D6B"


def configure_joint_drives(stage):
    """Enable X-Y bend drives and lock twist whenever the stage loads."""
    for joint_path in joint_paths():
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            raise RuntimeError(f"D6 joint not found: {joint_path}")

        # Projection is deprecated in current PhysX and an enabled value only
        # produces a warning. Override older generated assets that stored it.
        PhysxSchema.PhysxJointAPI.Apply(
            joint_prim
        ).CreateEnableProjectionAttr().Set(False)

        # A D6 axis is locked when its lower limit exceeds its upper limit.
        twist_limit = UsdPhysics.LimitAPI.Apply(joint_prim, "rotZ")
        twist_limit.CreateLowAttr().Set(1.0)
        twist_limit.CreateHighAttr().Set(-1.0)

        # The A/B joints each represent half an interval, so their rotational
        # stiffness and damping are doubled (K = EI / length).
        length_scale = 1.0 if "Pair_00_Base" in joint_path else 2.0
        for axis in ("rotX", "rotY"):
            rotation_limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
            rotation_limit.CreateLowAttr().Set(-MAX_BEND_DEGREES)
            rotation_limit.CreateHighAttr().Set(MAX_BEND_DEGREES)

            drive = UsdPhysics.DriveAPI.Apply(joint_prim, axis)
            drive.CreateTypeAttr().Set("force")
            drive.CreateTargetPositionAttr().Set(0.0)
            drive.GetStiffnessAttr().Set(
                length_scale * BEND_STIFFNESS
            )
            drive.GetDampingAttr().Set(length_scale * BEND_DAMPING)


def configure_rigid_body_solver(stage):
    """Harden every disk and virtual link against constraint stretching."""
    body_paths = [
        f"/World/SoftLimb/Disk_{index:02d}"
        for index in range(JOINT_COUNT + 1)
    ]
    body_paths.extend(
        f"/World/SoftLimb/VirtualLinks/Pair_{index:02d}_Link"
        for index in range(1, JOINT_COUNT)
    )

    for body_path in body_paths:
        body_prim = stage.GetPrimAtPath(body_path)
        if not body_prim.IsValid():
            raise RuntimeError(f"Rigid body not found: {body_path}")
        physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
        physx_body.CreateSolverPositionIterationCountAttr().Set(
            SOLVER_POSITION_ITERATIONS
        )
        physx_body.CreateSolverVelocityIterationCountAttr().Set(
            SOLVER_VELOCITY_ITERATIONS
        )
        physx_body.CreateSleepThresholdAttr().Set(0.0)


def normalize_disk_xform_ops(stage):
    """Order the composed disk orient op so PhysX can update it safely."""
    for index in range(JOINT_COUNT + 1):
        disk_prim = stage.GetPrimAtPath(
            f"/World/SoftLimb/Disk_{index:02d}"
        )
        if not disk_prim.IsValid():
            raise RuntimeError(f"Disk not found: {disk_prim.GetPath()}")

        xform = UsdGeom.Xformable(disk_prim)
        ordered_ops = xform.GetOrderedXformOps()
        if any(
            op.GetOpType() == UsdGeom.XformOp.TypeOrient
            for op in ordered_ops
        ):
            continue

        orient_attr = disk_prim.GetAttribute("xformOp:orient")
        if orient_attr.IsValid():
            xform.SetXformOpOrder(
                [*ordered_ops, UsdGeom.XformOp(orient_attr)]
            )


class SoftLimbCommandSubscriber(Node):
    """Receive phi bend magnitude/torque and theta direction."""

    def __init__(self, stage):
        super().__init__("soft_limb_command_subscriber")
        # Columns are bend magnitude and bend direction in degrees.
        self.base_command = np.zeros(2, dtype=np.float32)
        self.base_applied = np.zeros(2, dtype=np.float32)
        self.d6_a_commands = np.zeros((JOINT_COUNT - 1, 2), dtype=np.float32)
        self.d6_b_commands = np.zeros((JOINT_COUNT - 1, 2), dtype=np.float32)
        self.d6_a_applied = np.zeros((JOINT_COUNT - 1, 2), dtype=np.float32)
        self.d6_b_applied = np.zeros((JOINT_COUNT - 1, 2), dtype=np.float32)
        self.base_torque_command = np.zeros(1, dtype=np.float32)
        self.base_torque_applied = np.zeros(1, dtype=np.float32)
        self.d6_a_torque_commands = np.zeros(
            JOINT_COUNT - 1, dtype=np.float32
        )
        self.d6_b_torque_commands = np.zeros(
            JOINT_COUNT - 1, dtype=np.float32
        )
        self.d6_a_torque_applied = np.zeros(
            JOINT_COUNT - 1, dtype=np.float32
        )
        self.d6_b_torque_applied = np.zeros(
            JOINT_COUNT - 1, dtype=np.float32
        )
        self._drive_targets = []

        self._subscribe_joint(
            "/soft_limb/pair_00/base",
            self.base_command,
            self.base_applied,
            self.base_torque_command,
            "/World/SoftLimb/Joints/Pair_00_Base_D6Joint",
            stage,
        )
        for pair_index in range(1, JOINT_COUNT):
            command_index = pair_index - 1
            self._subscribe_joint(
                f"/soft_limb/pair_{pair_index:02d}/d6_a",
                self.d6_a_commands[command_index],
                self.d6_a_applied[command_index],
                self.d6_a_torque_commands[command_index:command_index + 1],
                f"/World/SoftLimb/Joints/Pair_{pair_index:02d}_D6A",
                stage,
            )
            self._subscribe_joint(
                f"/soft_limb/pair_{pair_index:02d}/d6_b",
                self.d6_b_commands[command_index],
                self.d6_b_applied[command_index],
                self.d6_b_torque_commands[command_index:command_index + 1],
                f"/World/SoftLimb/Joints/Pair_{pair_index:02d}_D6B",
                stage,
            )

    def _subscribe_joint(
        self, topic_root, command, applied, torque_command, joint_path, stage
    ):
        joint_prim = stage.GetPrimAtPath(joint_path)
        x_target = UsdPhysics.DriveAPI(
            joint_prim, "rotX"
        ).GetTargetPositionAttr()
        y_target = UsdPhysics.DriveAPI(
            joint_prim, "rotY"
        ).GetTargetPositionAttr()
        self._drive_targets.append(
            (command, applied, x_target, y_target)
        )
        self.create_subscription(
            Float64,
            f"{topic_root}/roll/position",
            lambda message, values=command, torque=torque_command:
                self._set_roll_position(
                    values, torque, message
                ),
            10,
        )
        self.create_subscription(
            Float64,
            f"{topic_root}/roll/torque",
            lambda message, values=command, torque=torque_command:
                self._set_roll_torque(
                    values, torque, message
                ),
            10,
        )
        self.create_subscription(
            Float64,
            f"{topic_root}/yaw/position",
            lambda message, values=command: self._set_yaw_position(
                values, message
            ),
            10,
        )

    @staticmethod
    def _set_roll_position(command, torque_command, message):
        command[0] = np.clip(
            message.data,
            -MAX_ABS_BEND_POSITION_DEGREES,
            MAX_ABS_BEND_POSITION_DEGREES,
        )
        torque_command[0] = 0.0

    @staticmethod
    def _set_roll_torque(command, torque_command, message):
        torque_command[0] = np.clip(
            message.data, -MAX_ABS_TORQUE, MAX_ABS_TORQUE
        )
        command[0] = 0.0

    @staticmethod
    def _set_yaw_position(command, message):
        command[1] = (message.data + 180.0) % 360.0 - 180.0

    def update_drive_targets(self, time_step):
        """Move drive targets toward commands at a bounded angular speed."""
        max_step = MAX_TARGET_SPEED_DEGREES_PER_SECOND * time_step
        for command, applied, x_target, y_target in self._drive_targets:
            applied[0] += np.clip(
                command[0] - applied[0], -max_step, max_step
            )
            direction_delta = (
                command[1] - applied[1] + 180.0
            ) % 360.0 - 180.0
            applied[1] += np.clip(
                direction_delta, -max_step, max_step
            )
            direction = np.deg2rad(applied[1])
            # Webster/Jones convention: theta=0 bends about +Y toward +X.
            x_target.Set(float(-applied[0] * np.sin(direction)))
            y_target.Set(float(applied[0] * np.cos(direction)))

        max_torque_step = MAX_TORQUE_SLEW_RATE * time_step
        torque_groups = (
            (self.base_torque_command, self.base_torque_applied),
            (self.d6_a_torque_commands, self.d6_a_torque_applied),
            (self.d6_b_torque_commands, self.d6_b_torque_applied),
        )
        for commands, applied in torque_groups:
            applied += np.clip(
                commands - applied, -max_torque_step, max_torque_step
            )

    @staticmethod
    def _world_bend_directions(orientations, directions_degrees):
        """Rotate local theta-directed X-Y axes into world coordinates."""
        orientations = np.asarray(orientations, dtype=np.float32)
        w = orientations[:, 0]
        x = orientations[:, 1]
        y = orientations[:, 2]
        z = orientations[:, 3]
        x_axes = np.column_stack(
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y + w * z),
                2.0 * (x * z - w * y),
            )
        )
        y_axes = np.column_stack(
            (
                2.0 * (x * y - w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z + w * x),
            )
        )
        directions = np.deg2rad(directions_degrees)
        return (
            -np.sin(directions)[:, None] * x_axes
            + np.cos(directions)[:, None] * y_axes
        ).astype(np.float32)

    def body_torques(self, disk_orientations, virtual_orientations):
        """Convert joint roll torques into equal/opposite body torques."""
        disk_torques = np.zeros((JOINT_COUNT + 1, 3), dtype=np.float32)
        virtual_torques = np.zeros((JOINT_COUNT - 1, 3), dtype=np.float32)
        base_axis = self._world_bend_directions(
            disk_orientations[:1], self.base_applied[1:2]
        )[0]
        d6_a_axes = self._world_bend_directions(
            disk_orientations[1:JOINT_COUNT], self.d6_a_applied[:, 1]
        )
        d6_b_axes = self._world_bend_directions(
            virtual_orientations, self.d6_b_applied[:, 1]
        )

        base_torque = self.base_torque_applied[0] * base_axis
        disk_torques[0] -= base_torque
        disk_torques[1] += base_torque

        for command_index in range(JOINT_COUNT - 1):
            pair_index = command_index + 1
            d6_a_torque = (
                self.d6_a_torque_applied[command_index]
                * d6_a_axes[command_index]
            )
            d6_b_torque = (
                self.d6_b_torque_applied[command_index]
                * d6_b_axes[command_index]
            )
            disk_torques[pair_index] -= d6_a_torque
            virtual_torques[command_index] += d6_a_torque
            virtual_torques[command_index] -= d6_b_torque
            disk_torques[pair_index + 1] += d6_b_torque

        return disk_torques, virtual_torques


def get_soft_limb_usd_path():
    """Find soft_limb.usd beside an installed or source-tree script."""
    script_directory = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(script_directory, "..", "usd", SOFT_LIMB_USD_NAME),
        os.path.join(
            script_directory, "..", "..", "..", "..", "..", "usd",
            SOFT_LIMB_USD_NAME
        ),
    )

    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate

    raise FileNotFoundError(
        f"Could not find usd/{SOFT_LIMB_USD_NAME} relative to "
        "load_soft_limb.py"
    )


def main():
    usd_path = get_soft_limb_usd_path()
    if not open_stage(usd_path):
        raise RuntimeError(f"Isaac Sim could not open {usd_path}")

    # Attach the World API to the physics scene already stored in the USD.
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_FREQUENCY_HZ,
        rendering_dt=1.0 / 60.0,
    )
    world.get_physics_context().set_physx_update_transformations_settings(
        update_to_usd=True,
        update_velocities_to_usd=True,
        output_velocities_local_space=False,
    )
    configure_joint_drives(world.stage)
    configure_rigid_body_solver(world.stage)
    normalize_disk_xform_ops(world.stage)
    disks = world.scene.add(
        RigidPrim(
            prim_paths_expr="/World/SoftLimb/Disk_.*",
            name="soft_limb_disks",
            reset_xform_properties=False,
        )
    )
    virtual_links = world.scene.add(
        RigidPrim(
            prim_paths_expr="/World/SoftLimb/VirtualLinks/Pair_.*_Link",
            name="soft_limb_virtual_links",
            reset_xform_properties=False,
        )
    )
    world.reset()
    # Keep the lightweight bodies responsive to live commands.
    disks.set_sleep_thresholds(np.zeros(JOINT_COUNT + 1, dtype=np.float32))
    virtual_links.set_sleep_thresholds(
        np.zeros(JOINT_COUNT - 1, dtype=np.float32)
    )
    world.play()

    started_rclpy = not rclpy.ok()
    if started_rclpy:
        rclpy.init()
    command_subscriber = SoftLimbCommandSubscriber(world.stage)
    physx_interface = get_physx_interface()

    set_camera_view(
        eye=np.array([0.45, 0.45, 0.22]),
        target=np.array([0.0, 0.0, 0.16]),
        camera_prim_path="/OmniverseKit_Persp",
    )

    print("--------------------------------")
    print("SAVED SOFT LIMB LOADED")
    print("--------------------------------")
    print(f"USD stage: {usd_path}")
    print("Phi position      : .../roll/position (bend degrees)")
    print("Phi torque        : .../roll/torque (N*m)")
    print("Theta direction   : .../yaw/position (direction degrees)")
    print("Command sets      : base plus pair_01..10/d6_a and d6_b")
    print("Position units    : degrees")
    print(
        f"Bend limit        : +/-{MAX_ABS_BEND_POSITION_DEGREES} degrees"
    )
    print(f"Torque limit      : +/-{MAX_ABS_TORQUE} N*m")
    print(
        f"Target speed limit: {MAX_TARGET_SPEED_DEGREES_PER_SECOND} deg/s"
    )
    print(f"Bend stiffness    : {BEND_STIFFNESS}")
    print(f"Bend damping      : {BEND_DAMPING}")
    print(f"Physics frequency : {PHYSICS_FREQUENCY_HZ} Hz")
    print(f"Solver pos iters  : {SOLVER_POSITION_ITERATIONS}")
    print(f"Solver vel iters  : {SOLVER_VELOCITY_ITERATIONS}")
    print("Joint projection  : disabled (deprecated)")
    print("Joint topology    : 1 base D6, then 2 serial D6 per pair")
    print("Physics to USD    : live updates enabled")
    print(f"X/Y bend limits   : +/-{MAX_BEND_DEGREES} degrees")
    print("Twist (rotZ)      : locked")
    print("--------------------------------")

    while simulation_app.is_running():
        rclpy.spin_once(command_subscriber, timeout_sec=0.0)
        if world.is_playing():
            command_subscriber.update_drive_targets(
                1.0 / PHYSICS_FREQUENCY_HZ
            )
            _, disk_orientations = disks.get_world_poses()
            _, virtual_orientations = virtual_links.get_world_poses()
            disk_torques, virtual_torques = command_subscriber.body_torques(
                disk_orientations, virtual_orientations
            )
            disks.apply_forces_and_torques_at_pos(
                torques=disk_torques,
                is_global=True,
            )
            virtual_links.apply_forces_and_torques_at_pos(
                torques=virtual_torques,
                is_global=True,
            )
            world.step(render=False)
            physx_interface.update_transformations(False, True, True)
        world.render()

    command_subscriber.destroy_node()
    if started_rclpy:
        rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
