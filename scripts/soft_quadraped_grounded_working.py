#!/usr/bin/env python3

"""Run the unchanged 12-disc quadruped with hybrid torque/theta control."""

import os

from isaacsim import SimulationApp


HEADLESS = os.environ.get("SOFT_QUADRUPED_HEADLESS", "0") == "1"
MAX_SIMULATION_STEPS = int(
    os.environ.get("SOFT_QUADRUPED_MAX_STEPS", "0")
)
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx import get_physx_interface
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_COUNT_PER_LIMB = 11
DISK_COUNT_PER_LIMB = 12

# Firm, heavily damped leg drives for stable ground contact.
MAX_ABS_POSITION_DEGREES = 25.0
MAX_TARGET_SPEED_DEGREES_PER_SECOND = 120.0
MAX_ABS_TORQUE = 1.0
# Reach the requested standing torque quickly, while retaining a short ramp
# that prevents a one-frame impulse when the feet establish contact.
MAX_TORQUE_SLEW_RATE = 10.0
MAX_TOTAL_TORQUE_MODE_BEND_DEGREES = 180.0
# High-pressure-like tuning: four times the preceding joint stiffness and
# twice its damping. Since damping ratio scales as c/sqrt(k), this retains
# approximately the preceding ratio while doubling natural frequency.
ROLL_STIFFNESS = 20_000_000
ROLL_DAMPING = 42_000
YAW_STIFFNESS = 30_000_000
YAW_DAMPING = 51_000
MAX_BEND_DEGREES = 25.0

# ROS name -> USD prim name. Topics follow the standalone limb topology:
# /<limb>/pair_NN/<base|d6_a|d6_b>/<roll|yaw>/position
LIMB_NAMES = {
    "body": "Body",
    "front_left": "FrontLeft",
    "front_right": "FrontRight",
    "rear_left": "RearLeft",
    "rear_right": "RearRight",
}

ROBOT_PATH = "/World/SoftQuadruped"
ATTACHMENTS_PATH = f"{ROBOT_PATH}/Attachments"
SOURCE_LIMB_PATH = Sdf.Path("/World/SoftLimb")

BODY_MIDPOINT = 0.1675
BODY_END_X = 0.1375
BLUE_END_OFFSET = 0.03
INITIAL_HEIGHT = 1.0
FLOOR_STATIC_FRICTION = 0.9
FLOOR_DYNAMIC_FRICTION = 0.9
FLOOR_RESTITUTION = 0.0
# 120 Hz is sufficient for the stiff, damped joint model and halves the GUI
# physics workload relative to the former 240 Hz configuration.  This keeps
# displayed motion much closer to real time on the same machine.
PHYSICS_FREQUENCY_HZ = 120.0
RENDER_FREQUENCY_HZ = 60.0
RENDER_EVERY_N_PHYSICS_STEPS = max(
    1, int(round(PHYSICS_FREQUENCY_HZ / RENDER_FREQUENCY_HZ))
)
SOLVER_POSITION_ITERATIONS = 24
SOLVER_VELOCITY_ITERATIONS = 6
BODY_LINEAR_DAMPING = 5.0
BODY_ANGULAR_DAMPING = 25.0
LEFT_LEG_AXIAL_ROTATION_DEGREES = 180.0
# The right legs receive a second 180-degree turn relative to the previous
# all-leg-180 configuration. A net 360 degrees is orientation-equivalent to
# zero, but recording 360 makes the requested additional turn explicit.
RIGHT_LEG_AXIAL_ROTATION_DEGREES = 360.0
BEND_FEEDBACK_PROPORTIONAL_GAIN = 1.2
BEND_FEEDBACK_INTEGRAL_GAIN_PER_SECOND = 4.0
MAX_BEND_COMPENSATION_DEGREES = 90.0
MAX_COMPENSATED_TOTAL_BEND_DEGREES = 250.0
BEND_MEASUREMENT_FILTER_SECONDS = 0.015


def quaternion_multiply(left, right):
    """Multiply scalar-first quaternions."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ], dtype=np.float64)


def relative_rotation_vector(reference, measured):
    """Return the local rotation vector from reference to measured."""
    reference = np.asarray(reference, dtype=np.float64)
    measured = np.asarray(measured, dtype=np.float64)
    reference /= np.linalg.norm(reference)
    measured /= np.linalg.norm(measured)
    relative = quaternion_multiply(
        reference * np.array([1.0, -1.0, -1.0, -1.0]), measured
    )
    relative /= np.linalg.norm(relative)
    if relative[0] < 0.0:
        relative = -relative
    vector = relative[1:]
    vector_norm = np.linalg.norm(vector)
    if vector_norm < 1.0e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(vector_norm, relative[0])
    return angle * vector / vector_norm


def measured_signed_roll_bend_degrees(disk_view):
    """Accumulate local disc-to-disc roll bend for one complete limb."""
    _, orientations = disk_view.get_world_poses()
    orientations = np.asarray(orientations, dtype=np.float64)
    bend_radians = 0.0
    for previous, current in zip(orientations[:-1], orientations[1:]):
        bend_radians += relative_rotation_vector(previous, current)[0]
    return float(np.rad2deg(bend_radians))


class QuadrupedHybridSubscriber(Node):
    """Apply compensated phi-torque and held theta targets to five limbs."""

    def __init__(self, stage, limb_paths, disk_views):
        super().__init__("soft_quadruped_hybrid_subscriber")
        self._drive_targets = {name: [] for name in limb_paths}
        self._disk_views = disk_views
        self._filtered_bends = {name: 0.0 for name in limb_paths}
        self._integral_compensation = {name: 0.0 for name in limb_paths}

        for limb_name, limb_path in limb_paths.items():
            self._subscribe_joint(
                f"/{limb_name}/pair_00/base",
                f"{limb_path}/Joints/Pair_00_Base_D6Joint",
                stage,
                limb_name,
                1.0 / JOINT_COUNT_PER_LIMB,
            )
            for pair_index in range(1, JOINT_COUNT_PER_LIMB):
                joint_suffixes = (
                    ("D6A", "d6_a"),
                    ("D6B", "d6_b"),
                )
                for suffix, topic_suffix in joint_suffixes:
                    self._subscribe_joint(
                        f"/{limb_name}/pair_{pair_index:02d}/{topic_suffix}",
                        f"{limb_path}/Joints/Pair_{pair_index:02d}_{suffix}",
                        stage,
                        limb_name,
                        1.0 / (2.0 * JOINT_COUNT_PER_LIMB),
                    )

    def _subscribe_joint(
        self, topic_root, joint_path, stage, limb_name, bend_share
    ):
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            raise RuntimeError(f"Missing D6 joint: {joint_path}")

        command = np.zeros(2, dtype=np.float32)
        applied = np.zeros(2, dtype=np.float32)
        roll_target = UsdPhysics.DriveAPI(
            joint_prim, "rotX"
        ).GetTargetPositionAttr()
        yaw_target = UsdPhysics.DriveAPI(
            joint_prim, "rotZ"
        ).GetTargetPositionAttr()
        self._drive_targets[limb_name].append(
            (command, applied, roll_target, yaw_target, bend_share)
        )

        self.create_subscription(
            Float64,
            f"{topic_root}/roll/torque",
            lambda message, values=command: self._set_torque(values, message),
            10,
        )
        self.create_subscription(
            Float64,
            f"{topic_root}/yaw/position",
            lambda message, values=command: self._set_theta(values, message),
            10,
        )

    @staticmethod
    def _set_torque(command, message):
        command[0] = np.clip(
            message.data,
            -MAX_ABS_TORQUE,
            MAX_ABS_TORQUE,
        )

    @staticmethod
    def _set_theta(command, message):
        command[1] = np.clip(
            message.data,
            -MAX_ABS_POSITION_DEGREES,
            MAX_ABS_POSITION_DEGREES,
        )

    def update_drive_targets(self, time_step):
        """Track commanded bend and compensate measured load-induced error."""
        torque_step = MAX_TORQUE_SLEW_RATE * time_step
        position_step = MAX_TARGET_SPEED_DEGREES_PER_SECOND * time_step
        filter_alpha = time_step / (
            BEND_MEASUREMENT_FILTER_SECONDS + time_step
        )
        for limb_name, drive_targets in self._drive_targets.items():
            for command, applied, _, yaw_target, _ in drive_targets:
                applied[0] += np.clip(
                    command[0] - applied[0], -torque_step, torque_step
                )
                applied[1] += np.clip(
                    command[1] - applied[1], -position_step, position_step
                )
                yaw_target.Set(float(applied[1]))

            requested_total_bend = (
                drive_targets[0][1][0]
                / MAX_ABS_TORQUE
                * MAX_TOTAL_TORQUE_MODE_BEND_DEGREES
            )
            if limb_name == "body":
                compensated_total_bend = requested_total_bend
            else:
                measured_bend = measured_signed_roll_bend_degrees(
                    self._disk_views[limb_name]
                )
                self._filtered_bends[limb_name] += filter_alpha * (
                    measured_bend - self._filtered_bends[limb_name]
                )
                bend_error = (
                    requested_total_bend - self._filtered_bends[limb_name]
                )
                self._integral_compensation[limb_name] = np.clip(
                    self._integral_compensation[limb_name]
                    + BEND_FEEDBACK_INTEGRAL_GAIN_PER_SECOND
                    * bend_error
                    * time_step,
                    -MAX_BEND_COMPENSATION_DEGREES,
                    MAX_BEND_COMPENSATION_DEGREES,
                )
                compensation = np.clip(
                    BEND_FEEDBACK_PROPORTIONAL_GAIN * bend_error
                    + self._integral_compensation[limb_name],
                    -MAX_BEND_COMPENSATION_DEGREES,
                    MAX_BEND_COMPENSATION_DEGREES,
                )
                compensated_total_bend = np.clip(
                    requested_total_bend + compensation,
                    -MAX_COMPENSATED_TOTAL_BEND_DEGREES,
                    MAX_COMPENSATED_TOTAL_BEND_DEGREES,
                )

            for _, _, roll_target, _, bend_share in drive_targets:
                roll_target.Set(float(compensated_total_bend * bend_share))


def get_soft_limb_usd_path():
    """Find the position-controlled limb in source/installed layouts."""
    script_directory = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(
            script_directory,
            "..",
            "usd",
            "soft_limb_position_control.usd",
        ),
        os.path.join(
            script_directory, "..", "..", "..", "..", "..", "usd",
            "soft_limb_position_control.usd"
        ),
    )
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(
        "Could not find usd/soft_limb_position_control.usd"
    )


def add_limb(
    stage,
    usd_path,
    name,
    translation,
    rotation=(0.0, 0.0, 0.0),
    axial_rotation=0.0,
):
    """Reference and position one complete saved soft limb."""
    limb_path = f"{ROBOT_PATH}/{name}"
    limb_prim = stage.OverridePrim(limb_path)
    limb_prim.GetReferences().AddReference(usd_path, SOURCE_LIMB_PATH)

    xform = UsdGeom.Xformable(limb_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    if axial_rotation:
        # Authored after the mounting rotation so this operation is applied
        # in the limb's local frame, about its longitudinal +Z axis.
        xform.AddRotateZOp(opSuffix="axial").Set(float(axial_rotation))
    return limb_path


def configure_limb(
    stage, limb_path, keep_foot_fixed, lock_bending=False
):
    """Configure limb drives, base mobility, and optional rigidity."""
    joint_paths = [
        f"{limb_path}/Joints/Pair_00_Base_D6Joint"
    ]
    for index in range(1, JOINT_COUNT_PER_LIMB):
        joint_paths.extend(
            (
                f"{limb_path}/Joints/Pair_{index:02d}_D6A",
                f"{limb_path}/Joints/Pair_{index:02d}_D6B",
            )
        )

    for joint_path in joint_paths:
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            raise RuntimeError(f"Missing D6 joint: {joint_path}")

        PhysxSchema.PhysxJointAPI.Apply(
            joint_prim
        ).CreateEnableProjectionAttr().Set(False)
        pitch_limit = UsdPhysics.LimitAPI.Apply(joint_prim, "rotY")
        pitch_limit.CreateLowAttr().Set(1.0)
        pitch_limit.CreateHighAttr().Set(-1.0)

        for axis in ("rotX", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
            if lock_bending:
                # A lower limit above the upper limit locks a D6 axis.
                limit.CreateLowAttr().Set(1.0)
                limit.CreateHighAttr().Set(-1.0)
            else:
                limit.CreateLowAttr().Set(-MAX_BEND_DEGREES)
                limit.CreateHighAttr().Set(MAX_BEND_DEGREES)
            drive = UsdPhysics.DriveAPI.Apply(joint_prim, axis)
            drive.CreateTypeAttr().Set("force")
            drive.CreateTargetPositionAttr().Set(0.0)
            if axis == "rotX":
                drive.CreateStiffnessAttr().Set(ROLL_STIFFNESS)
                drive.CreateDampingAttr().Set(ROLL_DAMPING)
            else:
                drive.CreateStiffnessAttr().Set(YAW_STIFFNESS)
                drive.CreateDampingAttr().Set(YAW_DAMPING)

    for index in range(DISK_COUNT_PER_LIMB):
        disk_prim = stage.GetPrimAtPath(f"{limb_path}/Disk_{index:02d}")
        if not disk_prim.IsValid():
            raise RuntimeError(f"Missing disk in {limb_path}: {index}")
        physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(disk_prim)
        physx_body.CreateSolverPositionIterationCountAttr().Set(
            SOLVER_POSITION_ITERATIONS
        )
        physx_body.CreateSolverVelocityIterationCountAttr().Set(
            SOLVER_VELOCITY_ITERATIONS
        )
        physx_body.CreateSleepThresholdAttr().Set(0.0)
        physx_body.CreateLinearDampingAttr().Set(BODY_LINEAR_DAMPING)
        physx_body.CreateAngularDampingAttr().Set(BODY_ANGULAR_DAMPING)

    for index in range(1, JOINT_COUNT_PER_LIMB):
        link_prim = stage.GetPrimAtPath(
            f"{limb_path}/VirtualLinks/Pair_{index:02d}_Link"
        )
        if not link_prim.IsValid():
            raise RuntimeError(f"Missing virtual link in {limb_path}: {index}")
        physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
        physx_body.CreateSolverPositionIterationCountAttr().Set(
            SOLVER_POSITION_ITERATIONS
        )
        physx_body.CreateSolverVelocityIterationCountAttr().Set(
            SOLVER_VELOCITY_ITERATIONS
        )
        physx_body.CreateSleepThresholdAttr().Set(0.0)
        physx_body.CreateLinearDampingAttr().Set(BODY_LINEAR_DAMPING)
        physx_body.CreateAngularDampingAttr().Set(BODY_ANGULAR_DAMPING)

    base_prim = stage.GetPrimAtPath(f"{limb_path}/Disk_00")
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(base_prim)
    rigid_body.CreateKinematicEnabledAttr().Set(keep_foot_fixed)


def add_fixed_attachment(stage, name, body_0_path, body_1_path):
    """Fix two rigid disks while preserving their authored relative pose."""
    body_0 = stage.GetPrimAtPath(body_0_path)
    body_1 = stage.GetPrimAtPath(body_1_path)
    if not body_0.IsValid() or not body_1.IsValid():
        raise RuntimeError(f"Cannot attach {body_0_path} to {body_1_path}")

    xform_cache = UsdGeom.XformCache()
    body_0_pose = xform_cache.GetLocalToWorldTransform(body_0)
    body_1_pose = xform_cache.GetLocalToWorldTransform(body_1)
    relative_pose = (body_1_pose * body_0_pose.GetInverse()).RemoveScaleShear()

    joint = UsdPhysics.FixedJoint.Define(
        stage, f"{ATTACHMENTS_PATH}/{name}"
    )
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body_0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body_1_path)])
    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(relative_pose.ExtractTranslation())
    )
    joint.CreateLocalRot0Attr().Set(
        Gf.Quatf(relative_pose.ExtractRotationQuat())
    )
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateCollisionEnabledAttr().Set(False)


def main():
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_FREQUENCY_HZ,
        rendering_dt=1.0 / RENDER_FREQUENCY_HZ,
    )
    world.scene.add_default_ground_plane(
        static_friction=FLOOR_STATIC_FRICTION,
        dynamic_friction=FLOOR_DYNAMIC_FRICTION,
        restitution=FLOOR_RESTITUTION,
    )
    world.get_physics_context().set_physx_update_transformations_settings(
        update_to_usd=True,
        update_velocities_to_usd=True,
        output_velocities_local_space=False,
    )

    stage = world.stage
    UsdGeom.Xform.Define(stage, ROBOT_PATH)
    UsdGeom.Scope.Define(stage, ATTACHMENTS_PATH)
    limb_usd = get_soft_limb_usd_path()

    body_path = add_limb(
        stage,
        limb_usd,
        "Body",
        (-BODY_MIDPOINT, 0.0, INITIAL_HEIGHT),
        (0.0, 90.0, 0.0),
    )

    # Each leg lies along Y. Disk_00 is the blue end and is positioned at the
    # body, while the rest of the limb extends outward to form a top-view H.
    leg_specs = (
        (
            "FrontLeft",
            BODY_END_X,
            -BLUE_END_OFFSET,
            (-90.0, 0.0, 0.0),
            LEFT_LEG_AXIAL_ROTATION_DEGREES,
        ),
        (
            "FrontRight",
            BODY_END_X,
            BLUE_END_OFFSET,
            (90.0, 0.0, 0.0),
            RIGHT_LEG_AXIAL_ROTATION_DEGREES,
        ),
        (
            "RearLeft",
            -BODY_END_X,
            -BLUE_END_OFFSET,
            (-90.0, 0.0, 0.0),
            LEFT_LEG_AXIAL_ROTATION_DEGREES,
        ),
        (
            "RearRight",
            -BODY_END_X,
            BLUE_END_OFFSET,
            (90.0, 0.0, 0.0),
            RIGHT_LEG_AXIAL_ROTATION_DEGREES,
        ),
    )
    leg_paths = {}
    for name, x_position, y_position, rotation, axial_rotation in leg_specs:
        leg_paths[name] = add_limb(
            stage,
            limb_usd,
            name,
            (x_position, y_position, INITIAL_HEIGHT),
            rotation,
            axial_rotation,
        )

    # Leave every limb dynamic so the robot is not suspended in world space.
    configure_limb(
        stage,
        body_path,
        keep_foot_fixed=False,
        lock_bending=True,
    )
    for leg_path in leg_paths.values():
        configure_limb(stage, leg_path, keep_foot_fixed=False)

    # Two fixed leg attachments at each end disk of the body limb.
    for side_name in ("FrontLeft", "FrontRight"):
        add_fixed_attachment(
            stage,
            f"{side_name}Attachment",
            f"{body_path}/Disk_11",
            f"{leg_paths[side_name]}/Disk_00",
        )
    for side_name in ("RearLeft", "RearRight"):
        add_fixed_attachment(
            stage,
            f"{side_name}Attachment",
            f"{body_path}/Disk_00",
            f"{leg_paths[side_name]}/Disk_00",
        )

    limb_paths = {
        ros_name: f"{ROBOT_PATH}/{usd_name}"
        for ros_name, usd_name in LIMB_NAMES.items()
    }
    disk_views = {}
    virtual_link_views = []
    for ros_name, usd_name in LIMB_NAMES.items():
        disk_views[ros_name] = world.scene.add(
                RigidPrim(
                    prim_paths_expr=f"{ROBOT_PATH}/{usd_name}/Disk_.*",
                    name=f"{ros_name}_disks",
                    reset_xform_properties=False,
                )
        )
        virtual_link_views.append(
            world.scene.add(
                RigidPrim(
                    prim_paths_expr=(
                        f"{ROBOT_PATH}/{usd_name}/VirtualLinks/Pair_.*_Link"
                    ),
                    name=f"{ros_name}_virtual_links",
                    reset_xform_properties=False,
                )
            )
        )

    world.reset()
    zero_sleep_thresholds = np.zeros(
        DISK_COUNT_PER_LIMB, dtype=np.float32
    )
    for disk_view in disk_views.values():
        disk_view.set_sleep_thresholds(zero_sleep_thresholds)
    zero_virtual_sleep_thresholds = np.zeros(
        JOINT_COUNT_PER_LIMB - 1, dtype=np.float32
    )
    for virtual_link_view in virtual_link_views:
        virtual_link_view.set_sleep_thresholds(
            zero_virtual_sleep_thresholds
        )
    world.play()

    started_rclpy = not rclpy.ok()
    if started_rclpy:
        rclpy.init()
    hybrid_subscriber = QuadrupedHybridSubscriber(
        world.stage, limb_paths, disk_views
    )
    physx_interface = get_physx_interface()

    set_camera_view(
        eye=np.array([1.3, 1.3, 1.5]),
        target=np.array([0.0, 0.0, 0.75]),
        camera_prim_path="/OmniverseKit_Persp",
    )

    print("--------------------------------")
    print("SOFT QUADRUPED STARTED")
    print("--------------------------------")
    print("Soft limbs        : 5")
    print("Body limbs        : 1 crossbar along X")
    print("Body bending      : locked (rigid/straight)")
    print("Leg limbs         : 4 coplanar limbs along Y")
    print("Top-view shape    : H")
    print("Body attachments  : blue Disk_00 leg ends")
    print("Fixed attachments : 4 (two per body end)")
    print(f"Initial height    : {INITIAL_HEIGHT} m (all limbs dynamic)")
    print(f"Limb USD          : {limb_usd}")
    print(f"Roll stiffness    : {ROLL_STIFFNESS}")
    print(f"Roll damping      : {ROLL_DAMPING}")
    print(f"Yaw stiffness     : {YAW_STIFFNESS}")
    print(f"Yaw damping       : {YAW_DAMPING}")
    print(f"Joint bend limits : +/-{MAX_BEND_DEGREES} degrees")
    print(f"Floor static mu   : {FLOOR_STATIC_FRICTION}")
    print(f"Floor dynamic mu  : {FLOOR_DYNAMIC_FRICTION}")
    print(f"Floor restitution : {FLOOR_RESTITUTION}")
    print(f"Physics frequency : {PHYSICS_FREQUENCY_HZ} Hz")
    print(f"Solver pos iters  : {SOLVER_POSITION_ITERATIONS}")
    print(f"Solver vel iters  : {SOLVER_VELOCITY_ITERATIONS}")
    print(f"Body linear damp  : {BODY_LINEAR_DAMPING}")
    print(f"Body angular damp : {BODY_ANGULAR_DAMPING}")
    print(
        "Leg axial rotation: "
        f"left={LEFT_LEG_AXIAL_ROTATION_DEGREES}, "
        f"right={RIGHT_LEG_AXIAL_ROTATION_DEGREES} degrees"
    )
    print("Control mode      : compensated phi torque + theta position")
    print(f"Torque limit      : +/-{MAX_ABS_TORQUE} N*m")
    print(
        "Bend feedback     : "
        f"Kp={BEND_FEEDBACK_PROPORTIONAL_GAIN}, "
        f"Ki={BEND_FEEDBACK_INTEGRAL_GAIN_PER_SECOND}/s"
    )
    print(
        f"Torque bend map   : {MAX_ABS_TORQUE} N*m -> "
        f"{MAX_TOTAL_TORQUE_MODE_BEND_DEGREES} degrees total"
    )
    print(
        "Hybrid topics     : /<limb>/pair_NN/<joint>/roll/torque and "
        ".../yaw/position"
    )
    print("Limb names        : " + ", ".join(LIMB_NAMES))
    print("--------------------------------")

    simulation_steps = 0
    while simulation_app.is_running():
        rclpy.spin_once(hybrid_subscriber, timeout_sec=0.0)
        if world.is_playing():
            hybrid_subscriber.update_drive_targets(
                1.0 / PHYSICS_FREQUENCY_HZ
            )
            world.step(render=False)
            physx_interface.update_transformations(False, True, True)
            simulation_steps += 1
        if (
            not HEADLESS
            and simulation_steps % RENDER_EVERY_N_PHYSICS_STEPS == 0
        ):
            world.render()
        if MAX_SIMULATION_STEPS and simulation_steps >= MAX_SIMULATION_STEPS:
            break

    if MAX_SIMULATION_STEPS:
        for limb_name in ("front_left", "front_right", "rear_left", "rear_right"):
            bend = measured_signed_roll_bend_degrees(disk_views[limb_name])
            print(f"FINAL_BEND {limb_name}={bend:.6f} degrees")

    hybrid_subscriber.destroy_node()
    if started_rclpy:
        rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
