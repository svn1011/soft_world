#!/usr/bin/env python3

"""Update authored six-disc USD geometry without flattening its schemas."""

from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdPhysics


PITCH = 0.025
LINK_OFFSET = 0.0005
ROOT = "/World/SoftLimb"


def move(stage, path, position):
    """Set the existing translation operation."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing prim: {path}")
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*position))
            return
    raise RuntimeError(f"Missing translation: {path}")


def update(path, direction):
    """Reanchor an existing native asset, preserving all other authored data."""
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"Cannot open {path}")
    base = stage.GetPrimAtPath(f"{ROOT}/Disk_00")
    origin = next(op.Get() for op in UsdGeom.Xformable(base).GetOrderedXformOps()
                  if op.GetOpType() == UsdGeom.XformOp.TypeTranslate)
    for index in range(6):
        move(stage, f"{ROOT}/Disk_{index:02d}",
             (origin[0], origin[1], origin[2] + direction * index * PITCH))
    for index in range(5):
        if index == 0:
            anchors = [("Pair_00_Base_D6Joint",
                        direction * PITCH / 2, -direction * PITCH / 2, 1)]
        else:
            move(stage, f"{ROOT}/VirtualLinks/Pair_{index:02d}_Link",
                 (origin[0], origin[1],
                  origin[2] + direction * (index + 0.5) * PITCH))
            disk = (PITCH - LINK_OFFSET) / 2
            link = LINK_OFFSET / 2
            anchors = [(f"Pair_{index:02d}_D6A", direction * disk,
                        -direction * link, 2),
                       (f"Pair_{index:02d}_D6B", direction * link,
                        -direction * disk, 2)]
        for name, anchor0, anchor1, scale in anchors:
            prim = stage.GetPrimAtPath(f"{ROOT}/Joints/{name}")
            joint = UsdPhysics.Joint(prim)
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(0, 0, anchor0))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, anchor1))
            for axis in ("rotX", "rotY"):
                limit = UsdPhysics.LimitAPI.Apply(prim, axis)
                limit.CreateLowAttr().Set(-40.0)
                limit.CreateHighAttr().Set(40.0)
                drive = UsdPhysics.DriveAPI.Apply(prim, axis)
                drive.CreateTypeAttr().Set("acceleration")
                drive.CreateTargetPositionAttr().Set(0.0)
                drive.CreateStiffnessAttr().Set(scale * 6400)
                drive.CreateDampingAttr().Set(scale * 240)
    # Save the authored root, not a flattened composition: this retains the
    # valid native asset's renderer and Isaac-specific metadata unchanged.
    if not stage.GetRootLayer().Save():
        raise RuntimeError(f"Cannot save {path}")
    print(f"Updated {path}: 25 mm pitch, 40 degree joint limit", flush=True)


def main():
    """Update both six-disc variants without touching the twelve-disc backups."""
    directory = Path(__file__).resolve().parent.parent / "usd"
    update(directory / "soft_limb_hanging_6_disc_working.usd", -1)
    update(directory / "soft_limb_position_control_6_disc_working.usd", 1)


if __name__ == "__main__":
    main()
