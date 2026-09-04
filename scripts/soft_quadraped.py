#!/usr/bin/env python3

"""Build a quadruped from five position-controlled dual-D6 limbs."""

import os

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

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

# Position-drive values from load_soft_limb.py.
MAX_ABS_POSITION_DEGREES = 25.0
MAX_TARGET_SPEED_DEGREES_PER_SECOND = 45.0
ROLL_STIFFNESS = 1_000_000
ROLL_DAMPING = 1
YAW_STIFFNESS = 2_000_000
YAW_DAMPING = 200
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
HANG_HEIGHT = 1.0
FLOOR_STATIC_FRICTION = 0.9
FLOOR_DYNAMIC_FRICTION = 0.9
FLOOR_RESTITUTION = 0.0
PHYSICS_FREQUENCY_HZ = 240.0
RENDER_FREQUENCY_HZ = 60.0
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 8


class QuadrupedPositionSubscriber(Node):
    """Apply held roll/yaw position targets to all five limbs."""

    def __init__(self, stage, limb_paths):
        super().__init__("soft_quadruped_position_subscriber")
        self._drive_targets = []

        for limb_name, limb_path in limb_paths.items():
            self._subscribe_joint(
                f"/{limb_name}/pair_00/base",
                f"{limb_path}/Joints/Pair_00_Base_D6Joint",
                stage,
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
                    )

    def _subscribe_joint(self, topic_root, joint_path, stage):
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
        self._drive_targets.append(
            (command, applied, roll_target, yaw_target)
        )

        self.create_subscription(
            Float64,
            f"{topic_root}/roll/position",
            lambda message, values=command: self._set_position(
                values, 0, message
            ),
            10,
        )
        self.create_subscription(
            Float64,
            f"{topic_root}/yaw/position",
            lambda message, values=command: self._set_position(
                values, 1, message
            ),
            10,
        )

    @staticmethod
    def _set_position(command, axis_index, message):
        command[axis_index] = np.clip(
            message.data,
            -MAX_ABS_POSITION_DEGREES,
            MAX_ABS_POSITION_DEGREES,
        )

    def update_drive_targets(self, time_step):
        """Move drive targets toward commands at a bounded angular speed."""
        max_step = MAX_TARGET_SPEED_DEGREES_PER_SECOND * time_step
        for command, applied, roll_target, yaw_target in self._drive_targets:
            applied += np.clip(command - applied, -max_step, max_step)
            roll_target.Set(float(applied[0]))
            yaw_target.Set(float(applied[1]))


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


def add_limb(stage, usd_path, name, translation, rotation=(0.0, 0.0, 0.0)):
    """Reference and position one complete saved soft limb."""
    limb_path = f"{ROBOT_PATH}/{name}"
    limb_prim = stage.OverridePrim(limb_path)
    limb_prim.GetReferences().AddReference(usd_path, SOURCE_LIMB_PATH)

    xform = UsdGeom.Xformable(limb_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    return limb_path


def configure_limb(stage, limb_path, keep_foot_fixed):
    """Apply the tuned spring values and configure base mobility."""
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
        (-BODY_MIDPOINT, 0.0, HANG_HEIGHT),
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
        ),
        (
            "FrontRight",
            BODY_END_X,
            BLUE_END_OFFSET,
            (90.0, 0.0, 0.0),
        ),
        (
            "RearLeft",
            -BODY_END_X,
            -BLUE_END_OFFSET,
            (-90.0, 0.0, 0.0),
        ),
        (
            "RearRight",
            -BODY_END_X,
            BLUE_END_OFFSET,
            (90.0, 0.0, 0.0),
        ),
    )
    leg_paths = {}
    for name, x_position, y_position, rotation in leg_specs:
        leg_paths[name] = add_limb(
            stage,
            limb_usd,
            name,
            (x_position, y_position, HANG_HEIGHT),
            rotation,
        )

    # Hang the complete robot from the blue base disk of the body limb.
    configure_limb(stage, body_path, keep_foot_fixed=True)
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
    disk_views = []
    virtual_link_views = []
    for ros_name, usd_name in LIMB_NAMES.items():
        disk_views.append(
            world.scene.add(
                RigidPrim(
                    prim_paths_expr=f"{ROBOT_PATH}/{usd_name}/Disk_.*",
                    name=f"{ros_name}_disks",
                    reset_xform_properties=False,
                )
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
    for disk_view in disk_views:
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
    position_subscriber = QuadrupedPositionSubscriber(
        world.stage, limb_paths
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
    print("Leg limbs         : 4 coplanar limbs along Y")
    print("Top-view shape    : H")
    print("Body attachments  : blue Disk_00 leg ends")
    print("Fixed attachments : 4 (two per body end)")
    print(f"Hanging base      : {body_path}/Disk_00 at z={HANG_HEIGHT} m")
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
    print("Control mode      : held roll/yaw position")
    print(
        "Position topics   : /<limb>/pair_NN/<joint>/<axis>/position"
    )
    print("Limb names        : " + ", ".join(LIMB_NAMES))
    print("--------------------------------")

    while simulation_app.is_running():
        rclpy.spin_once(position_subscriber, timeout_sec=0.0)
        if world.is_playing():
            position_subscriber.update_drive_targets(
                1.0 / PHYSICS_FREQUENCY_HZ
            )
            world.step(render=False)
            physx_interface.update_transformations(False, True, True)
        world.render()

    position_subscriber.destroy_node()
    if started_rclpy:
        rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
