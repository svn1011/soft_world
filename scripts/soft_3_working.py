#!/usr/bin/env python3

"""Generate three verified six-disc modules in one hanging serial snake."""

import os

from isaacsim import SimulationApp


EXPORT_ONLY = os.environ.get("SOFT_SNAKE_EXPORT_ONLY", "0") == "1"
simulation_app = SimulationApp({"headless": EXPORT_ONLY})

from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade


MODULE_COUNT = 3
DISKS_PER_MODULE = 6
INTERVALS_PER_MODULE = DISKS_PER_MODULE - 1
DISK_COUNT = MODULE_COUNT * DISKS_PER_MODULE
DISK_THICKNESS = 0.005
DISK_GAP = 0.02
DISK_PITCH = DISK_THICKNESS + DISK_GAP
BASE_HEIGHT = 0.75
DISK_MASS = 0.01
BEND_STIFFNESS = 6400
BEND_DAMPING = 240
MAX_INTERVAL_BEND_DEGREES = 80.0
PHYSICS_FREQUENCY_HZ = 240.0
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 8

ASSEMBLY_PATH = "/World/SoftSnake"
JOINTS_PATH = f"{ASSEMBLY_PATH}/Joints"
MATERIALS_PATH = f"{ASSEMBLY_PATH}/Materials"
SOURCE_PRIM_PATH = Sdf.Path("/World/SoftDisk")
OUTPUT_USD_NAME = "soft_3_working.usd"


def get_disk_usd_path():
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
    raise FileNotFoundError("Could not find usd/soft_disk.usd")


def get_output_usd_path():
    script_directory = os.path.dirname(os.path.abspath(__file__))
    source_directory = os.path.abspath(os.path.join(script_directory, "..", "usd"))
    if os.path.isdir(source_directory):
        return os.path.join(source_directory, OUTPUT_USD_NAME)
    return os.path.abspath(os.path.join(script_directory, "..", OUTPUT_USD_NAME))


def create_color_material(stage, name, color):
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


def disk_path(index):
    return Sdf.Path(f"{ASSEMBLY_PATH}/Disk_{index:02d}")


def module_interval_joint_path(module_index, interval_index):
    return (
        f"{JOINTS_PATH}/Module_{module_index + 1:02d}/"
        f"Interval_{interval_index + 1:02d}_D6Joint"
    )


def add_disk(stage, disk_usd_path, index, materials):
    prim = stage.OverridePrim(str(disk_path(index)))
    prim.GetReferences().AddReference(disk_usd_path, SOURCE_PRIM_PATH)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, BASE_HEIGHT - index * DISK_PITCH)
    )
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0))
    if index == 0:
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(True)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials["base"])
    elif index in (5, 11, 17):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            materials[f"tip_{index // DISKS_PER_MODULE + 1}"]
        )
    return disk_path(index)


def set_limit(joint_prim, axis, low, high):
    limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
    limit.CreateLowAttr().Set(low)
    limit.CreateHighAttr().Set(high)


def add_bend_joint(stage, module_index, interval_index, upper, lower):
    joint = UsdPhysics.Joint.Define(
        stage, module_interval_joint_path(module_index, interval_index)
    )
    joint.CreateBody0Rel().SetTargets([upper])
    joint.CreateBody1Rel().SetTargets([lower])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.5 * DISK_PITCH))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.5 * DISK_PITCH))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateCollisionEnabledAttr().Set(False)
    for axis in ("transX", "transY", "transZ", "rotZ"):
        set_limit(joint.GetPrim(), axis, 1.0, -1.0)
    for axis in ("rotX", "rotY"):
        set_limit(
            joint.GetPrim(), axis,
            -MAX_INTERVAL_BEND_DEGREES, MAX_INTERVAL_BEND_DEGREES,
        )
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("acceleration")
        drive.CreateTargetPositionAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(BEND_STIFFNESS)
        drive.CreateDampingAttr().Set(BEND_DAMPING)
    PhysxSchema.PhysxJointAPI.Apply(
        joint.GetPrim()
    ).CreateEnableProjectionAttr().Set(False)


def add_module_connector(stage, connector_index, upper, lower):
    joint = UsdPhysics.FixedJoint.Define(
        stage, f"{JOINTS_PATH}/Connector_{connector_index:02d}_FixedJoint"
    )
    joint.CreateBody0Rel().SetTargets([upper])
    joint.CreateBody1Rel().SetTargets([lower])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.5 * DISK_PITCH))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.5 * DISK_PITCH))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateCollisionEnabledAttr().Set(False)
    PhysxSchema.PhysxJointAPI.Apply(
        joint.GetPrim()
    ).CreateEnableProjectionAttr().Set(False)


def configure_body(prim):
    body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    body.CreateSolverPositionIterationCountAttr().Set(
        SOLVER_POSITION_ITERATIONS
    )
    body.CreateSolverVelocityIterationCountAttr().Set(
        SOLVER_VELOCITY_ITERATIONS
    )
    body.CreateSleepThresholdAttr().Set(0.0)
    body.CreateLinearDampingAttr().Set(2.0)
    body.CreateAngularDampingAttr().Set(10.0)


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
    UsdGeom.Scope.Define(stage, MATERIALS_PATH)
    for module_index in range(MODULE_COUNT):
        UsdGeom.Scope.Define(
            stage, f"{JOINTS_PATH}/Module_{module_index + 1:02d}"
        )

    materials = {
        "base": create_color_material(
            stage, "BaseBlue", Gf.Vec3f(0.02, 0.15, 0.8)
        ),
        "tip_1": create_color_material(
            stage, "Module1Red", Gf.Vec3f(0.8, 0.03, 0.03)
        ),
        "tip_2": create_color_material(
            stage, "Module2Gold", Gf.Vec3f(0.9, 0.55, 0.02)
        ),
        "tip_3": create_color_material(
            stage, "Module3Green", Gf.Vec3f(0.03, 0.65, 0.2)
        ),
    }
    disk_usd_path = get_disk_usd_path()
    disks = [
        add_disk(stage, disk_usd_path, index, materials)
        for index in range(DISK_COUNT)
    ]
    for index in range(DISK_COUNT):
        configure_body(stage.GetPrimAtPath(disks[index]))

    for module_index in range(MODULE_COUNT):
        start = module_index * DISKS_PER_MODULE
        for interval_index in range(INTERVALS_PER_MODULE):
            upper_index = start + interval_index
            add_bend_joint(
                stage, module_index, interval_index,
                disks[upper_index], disks[upper_index + 1],
            )
        if module_index < MODULE_COUNT - 1:
            add_module_connector(
                stage, module_index + 1,
                disks[start + DISKS_PER_MODULE - 1],
                disks[start + DISKS_PER_MODULE],
            )

    output_path = get_output_usd_path()
    if not stage.Export(output_path):
        raise RuntimeError(f"Could not export {output_path}")
    if EXPORT_ONLY:
        print(f"Exported three-module hanging snake: {output_path}")
        simulation_app.close()
        return

    world.reset()
    set_camera_view(
        eye=np.array([0.9, 0.9, 0.54]),
        target=np.array([0.0, 0.0, 0.54]),
        camera_prim_path="/OmniverseKit_Persp",
    )
    print("THREE-MODULE HANGING SOFT SNAKE STARTED")
    print(f"Disks: {DISK_COUNT}; actuated intervals: {MODULE_COUNT * INTERVALS_PER_MODULE}")
    print("Sensors: 18 total (six per module; added by the working loader)")
    print(f"Exported USD: {output_path}")
    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
