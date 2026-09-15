"""One topology definition for the active six-disc working limb."""

DISK_COUNT = 6
JOINT_COUNT = DISK_COUNT - 1
TIP_DISK_INDEX = DISK_COUNT - 1
DISK_PATHS = tuple(f"/World/SoftLimb/Disk_{index:02d}" for index in range(DISK_COUNT))


def interval_bend(total_bend):
    """Distribute base-to-final-disc bend over actual disc intervals."""
    return float(total_bend) / JOINT_COUNT


def validate_stage(stage):
    """Refuse a stale twelve-disc asset instead of treating disc six as intermediate."""
    root = stage.GetPrimAtPath("/World/SoftLimb")
    actual = {str(prim.GetPath()) for prim in root.GetChildren()
              if prim.GetName().startswith("Disk_")} if root else set()
    if actual != set(DISK_PATHS):
        raise RuntimeError(
            f"Working limb requires exactly {DISK_COUNT} discs with tip {DISK_PATHS[-1]}; "
            f"loaded {sorted(actual)}. Load the six-disc working USD.")
