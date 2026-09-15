#!/usr/bin/env python3

"""Build a soft limb suspended downward from an elevated fixed base."""

import os

from isaacsim import SimulationApp


EXPORT_ONLY = os.environ.get("SOFT_LIMB_EXPORT_ONLY", "0") == "1"
simulation_app = SimulationApp({"headless": EXPORT_ONLY})

from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade
import numpy as np


DISK_COUNT = 6
DISK_THICKNESS = 0.005  # 0.5 cm
# Five 55 mm intervals preserve the old 11 x 25 mm centerline length.
DISK_GAP = 0.05  # 5 cm clear space between disk faces
DISK_PITCH = DISK_THICKNESS + DISK_GAP
BASE_HEIGHT = 0.40

# Position-drive parameters shared by the standalone limb and quadruped.
BEND_STIFFNESS = 1_000_000
BEND_DAMPING = 100
MAX_BEND_DEGREES = 25.0
VIRTUAL_LINK_OFFSET = 0.0005  # 0.5 mm between the two serial joints
VIRTUAL_LINK_MASS = 0.01
VIRTUAL_LINK_INERTIA = 1.0e-6
PHYSICS_FREQUENCY_HZ = 240.0
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 8

ASSEMBLY_PATH = "/World/SoftLimb"
JOINTS_PATH = f"{ASSEMBLY_PATH}/Joints"
VIRTUAL_LINKS_PATH = f"{ASSEMBLY_PATH}/VirtualLinks"
MATERIALS_PATH = f"{ASSEMBLY_PATH}/Materials"
SOURCE_PRIM_PATH = Sdf.Path("/World/SoftDisk")
OUTPUT_USD_NAME = "soft_limb_hanging_working.usd"


def get_disk_usd_path():
    """Find soft_disk.usd in both source and installed ROS layouts."""
    script_directory = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(script_directory, "..", "usd", "soft_disk.usd"),
        os.path.join(
            script_directory, "..", "..", "..", "usd", "soft_disk.usd"
        ),
    )

    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate

    raise FileNotFoundError(
        "Could not find usd/soft_disk.usd relative to soft_limb.py"
    )


def get_output_usd_path():
    """Return the new dual-D6 USD path without replacing the old asset."""
    script_directory = os.path.dirname(os.path.abspath(__file__))
    source_usd_directory = os.path.abspath(
        os.path.join(script_directory, "..", "usd")
    )
    if os.path.isdir(source_usd_directory):
        return os.path.join(source_usd_directory, OUTPUT_USD_NAME)
    return os.path.abspath(
        os.path.join(script_directory, "..", OUTPUT_USD_NAME)
    )


def create_color_material(stage, name, color):
    """Create a simple opaque USD Preview Surface material."""
    material = UsdShade.Material.Define(stage, f"{MATERIALS_PATH}/{name}")
    shader = UsdShade.Shader.Define(
        stage, f"{MATERIALS_PATH}/{name}/PreviewSurface"
    )
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def add_disk(stage, disk_usd_path, index, base_material, end_material):
    """Reference one disk, extending downward from the fixed base."""
    disk_path = f"{ASSEMBLY_PATH}/Disk_{index:02d}"
    # Do not author a local Xform type here.  The referenced source prim is a
    # Cylinder; a stronger Xform type would hide its renderable geometry while
    # leaving its physics properties composed on the destination prim.
    disk_prim = stage.OverridePrim(disk_path)
    disk_prim.GetReferences().AddReference(disk_usd_path, SOURCE_PRIM_PATH)

    # Override the position authored in the source asset for this copy.
    xform = UsdGeom.Xformable(disk_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, BASE_HEIGHT - index * DISK_PITCH)
    )
    # The referenced cylinder already declares a double-precision orient op.
    # Keep it in the local op order so PhysX does not try to re-author the
    # dangling property from an untyped quaternion tuple during write-back.
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0))

    # Hold the elevated base disk while every later disk hangs below it.
    if index == 0:
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(disk_prim)
        rigid_body.CreateKinematicEnabledAttr().Set(True)
        UsdShade.MaterialBindingAPI.Apply(disk_prim).Bind(base_material)
    elif index == DISK_COUNT - 1:
        UsdShade.MaterialBindingAPI.Apply(disk_prim).Bind(end_material)

    return Sdf.Path(disk_path)


def set_joint_limit(joint_prim, axis, low, high):
    """Configure one translational or rotational D6 axis limit."""
    limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
    limit.CreateLowAttr().Set(low)
    limit.CreateHighAttr().Set(high)


def configure_d6_axes(joint_prim, length_scale):
    """Lock translation/twist and enable X-Y vector bending."""
    for axis in ("transX", "transY", "transZ", "rotZ"):
        set_joint_limit(joint_prim, axis, 1.0, -1.0)

    for axis in ("rotX", "rotY"):
        set_joint_limit(
            joint_prim, axis, -MAX_BEND_DEGREES, MAX_BEND_DEGREES
        )
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateTargetPositionAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(length_scale * BEND_STIFFNESS)
        drive.CreateDampingAttr().Set(length_scale * BEND_DAMPING)


def add_d6_joint(
    stage,
    name,
    body_0,
    body_1,
    local_pos_0,
    local_pos_1,
    length_scale,
):
    """Add one roll/yaw D6 joint using explicit local anchor positions."""
    joint_path = f"{JOINTS_PATH}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([body_0])
    joint.CreateBody1Rel().SetTargets([body_1])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos_0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos_1))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr().Set(False)
    configure_d6_axes(joint.GetPrim(), length_scale)
    return Sdf.Path(joint_path)


def add_virtual_link(stage, pair_index):
    """Create a hidden rigid link between two serial D6 joints."""
    link_path = f"{VIRTUAL_LINKS_PATH}/Pair_{pair_index:02d}_Link"
    link = UsdGeom.Cube.Define(stage, link_path)
    link.CreateSizeAttr().Set(1.0)
    link.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    xform = UsdGeom.Xformable(link.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(
            0.0,
            0.0,
            BASE_HEIGHT - (pair_index + 0.5) * DISK_PITCH,
        )
    )
    xform.AddScaleOp().Set(
        Gf.Vec3f(0.001, 0.001, VIRTUAL_LINK_OFFSET)
    )

    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(link.GetPrim())
    mass.CreateMassAttr().Set(VIRTUAL_LINK_MASS)
    mass.CreateDiagonalInertiaAttr().Set(
        Gf.Vec3f(
            VIRTUAL_LINK_INERTIA,
            VIRTUAL_LINK_INERTIA,
            VIRTUAL_LINK_INERTIA,
        )
    )
    mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))
    return Sdf.Path(link_path)


def add_joint_pair(stage, pair_index, upper_disk, lower_disk):
    """Connect each upper disk to the next disk below it."""
    if pair_index == 0:
        add_d6_joint(
            stage,
            "Pair_00_Base_D6Joint",
            upper_disk,
            lower_disk,
            (0.0, 0.0, -DISK_PITCH * 0.5),
            (0.0, 0.0, DISK_PITCH * 0.5),
            1.0,
        )
        return

    virtual_link = add_virtual_link(stage, pair_index)
    disk_anchor = (DISK_PITCH - VIRTUAL_LINK_OFFSET) * 0.5
    link_anchor = VIRTUAL_LINK_OFFSET * 0.5

    add_d6_joint(
        stage,
        f"Pair_{pair_index:02d}_D6A",
        upper_disk,
        virtual_link,
        (0.0, 0.0, -disk_anchor),
        (0.0, 0.0, link_anchor),
        2.0,
    )
    add_d6_joint(
        stage,
        f"Pair_{pair_index:02d}_D6B",
        virtual_link,
        lower_disk,
        (0.0, 0.0, -link_anchor),
        (0.0, 0.0, disk_anchor),
        2.0,
    )


def main():
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_FREQUENCY_HZ,
        rendering_dt=1.0 / 60.0,
    )
    world.scene.add_default_ground_plane()

    stage = world.stage
    UsdGeom.Xform.Define(stage, ASSEMBLY_PATH)
    UsdGeom.Scope.Define(stage, JOINTS_PATH)
    UsdGeom.Scope.Define(stage, VIRTUAL_LINKS_PATH)
    UsdGeom.Scope.Define(stage, MATERIALS_PATH)

    base_material = create_color_material(
        stage, "BaseBlue", Gf.Vec3f(0.02, 0.15, 0.8)
    )
    end_material = create_color_material(
        stage, "EndRed", Gf.Vec3f(0.8, 0.03, 0.03)
    )

    disk_usd_path = get_disk_usd_path()
    disk_paths = [
        add_disk(
            stage,
            disk_usd_path,
            index,
            base_material,
            end_material,
        )
        for index in range(DISK_COUNT)
    ]

    for pair_index in range(DISK_COUNT - 1):
        add_joint_pair(
            stage,
            pair_index,
            disk_paths[pair_index],
            disk_paths[pair_index + 1],
        )

    output_usd_path = get_output_usd_path()
    if not stage.Export(output_usd_path):
        raise RuntimeError(f"Could not export {output_usd_path}")

    if EXPORT_ONLY:
        print(f"Exported dual-D6 soft limb: {output_usd_path}")
        simulation_app.close()
        return

    world.reset()

    # The limb is only a few centimetres wide, so use a close initial view.
    set_camera_view(
        eye=np.array([0.45, 0.45, 0.28]),
        target=np.array(
            [0.0, 0.0, BASE_HEIGHT - 0.5 * (DISK_COUNT - 1) * DISK_PITCH]
        ),
        camera_prim_path="/OmniverseKit_Persp",
    )

    print("--------------------------------")
    print("HANGING SOFT LIMB STARTED")
    print("--------------------------------")
    print(f"Disk asset       : {disk_usd_path}")
    print(f"Source prim      : {SOURCE_PRIM_PATH}")
    print(f"Number of disks  : {DISK_COUNT}")
    print(f"Clear disk gap   : {DISK_GAP * 100:.1f} cm")
    print("Joint type       : spring D6")
    print("Base joint count : 1")
    print("Other pair joints: 2 serial D6 joints")
    print(f"Virtual offset   : {VIRTUAL_LINK_OFFSET * 1000:.1f} mm")
    print(f"Total joints     : {1 + 2 * (DISK_COUNT - 2)}")
    print(f"Exported USD     : {output_usd_path}")
    print(f"Bend stiffness   : {BEND_STIFFNESS}")
    print(f"Bend damping     : {BEND_DAMPING}")
    print(f"Maximum bend     : {MAX_BEND_DEGREES:.1f} degrees")
    print(f"Base disk        : anchored at z={BASE_HEIGHT:.3f} m")
    print("Limb direction   : downward from Disk_00")
    print("--------------------------------")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
