#!/usr/bin/env python3

"""Measure bounded, controller-independent bend tracking in headless Isaac."""

import argparse
import os


def main():
    """Exercise the same position drives and compensation as the loader."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phi", type=float, default=180.0)
    parser.add_argument("--theta", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--legacy-controller", action="store_true",
                        help="Verify compatibility with the original 12-disc controller")
    args = parser.parse_args()
    os.environ["SOFT_LIMB_HEADLESS"] = "1"
    import load_soft_limb_working as limb

    np = limb.np
    node = None
    controller = None
    try:
        usd_directory = os.path.join(os.path.dirname(__file__), "..", "usd")
        usd_path = os.path.abspath(os.path.join(
            usd_directory, "soft_limb_hanging_6_disc_working.usd"
        ))
        if not limb.open_stage(usd_path):
            raise RuntimeError(f"Cannot open {usd_path}")
        from omni.usd import get_context
        limb.validate_stage(get_context().get_stage())
        world = limb.World(stage_units_in_meters=1.0,
                           physics_dt=1.0 / limb.PHYSICS_FREQUENCY_HZ,
                           rendering_dt=1.0 / 60.0)
        limb.configure_joint_drives(world.stage)
        limb.configure_rigid_body_solver(world.stage)
        limb.normalize_disk_xform_ops(world.stage)
        disks = world.scene.add(limb.RigidPrim(
            prim_paths_expr=list(limb.DISK_PATHS),
            name="verify_disks", reset_xform_properties=False))
        links = world.scene.add(limb.RigidPrim(
            prim_paths_expr="/World/SoftLimb/VirtualLinks/Pair_.*_Link",
            name="verify_links", reset_xform_properties=False))
        world.reset()
        disks.set_sleep_thresholds(np.zeros(limb.JOINT_COUNT + 1))
        links.set_sleep_thresholds(np.zeros(limb.JOINT_COUNT - 1))
        limb.rclpy.init()
        node = limb.SoftLimbCommandSubscriber(world.stage)
        if args.legacy_controller:
            from control_soft_limb import SoftLimbPositionPublisher
        else:
            from control_soft_limb_working import SoftLimbPositionPublisher
        from rclpy.parameter import Parameter
        import time
        controller = SoftLimbPositionPublisher()
        controller.set_parameters([Parameter("phi_position", value=args.phi),
                                   Parameter("theta_position", value=args.theta)])
        # Exercise real publisher/subscriber delivery and command distribution.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            controller.publish_positions()
            limb.rclpy.spin_once(node, timeout_sec=0.01)
            expected = args.phi / limb.JOINT_COUNT
            if (np.isclose(node.base_command[0], expected)
                    and np.allclose(node.d6_a_commands[:, 0], expected / 2)
                    and np.allclose(node.d6_b_commands[:, 0], expected / 2)
                    and np.isclose(node.base_command[1], args.theta)
                    and np.allclose(node.d6_a_commands[:, 1], args.theta)
                    and np.allclose(node.d6_b_commands[:, 1], args.theta)):
                break
        else:
            raise RuntimeError("Working controller did not deliver all six-disc commands")
        print(f"VERIFY controller: base={node.base_command[0]} deg, "
              f"split joints={node.d6_a_commands[:, 0].tolist()} deg; "
              f"tip={limb.DISK_PATHS[-1]}", flush=True)
        disk_forces = np.zeros((limb.JOINT_COUNT + 1, 3), dtype=np.float32)
        link_forces = np.zeros((limb.JOINT_COUNT - 1, 3), dtype=np.float32)
        disk_forces[:, 2] = limb.DISK_MASS * limb.GRAVITY_ACCELERATION
        link_forces[:, 2] = limb.VIRTUAL_LINK_MASS * limb.GRAVITY_ACCELERATION
        interface = limb.get_physx_interface()
        world.play()
        measurements = []
        steps = int(args.seconds * limb.PHYSICS_FREQUENCY_HZ)
        for index in range(steps):
            node.update_drive_targets(1.0 / limb.PHYSICS_FREQUENCY_HZ)
            disks.apply_forces_and_torques_at_pos(forces=disk_forces,
                                                is_global=True)
            links.apply_forces_and_torques_at_pos(forces=link_forces,
                                                is_global=True)
            world.step(render=False)
            interface.update_transformations(False, True, True)
            if index % 24 == 0:
                world.render()
            if index >= steps - int(limb.PHYSICS_FREQUENCY_HZ):
                _, orientations = disks.get_world_poses()
                rotation = limb.quaternion_rotation_vector(
                    limb.relative_quaternion(orientations[0], orientations[-1]))
                measurements.append(float(np.rad2deg(np.hypot(*rotation[:2]))))
        _, orientations = disks.get_world_poses()
        tip = limb.relative_quaternion(orientations[0], orientations[-1])
        half_angle = np.deg2rad(args.phi) * 0.5
        direction = np.deg2rad(args.theta)
        target = np.array([np.cos(half_angle),
                           -np.sin(direction) * np.sin(half_angle),
                           np.cos(direction) * np.sin(half_angle), 0.0])
        error = float(np.rad2deg(np.linalg.norm(limb.quaternion_rotation_vector(
            limb.relative_quaternion(target, tip)))))
        intervals = [float(np.rad2deg(np.linalg.norm(
            limb.quaternion_rotation_vector(limb.relative_quaternion(a, b)))))
            for a, b in zip(orientations[:-1], orientations[1:])]
        print(f"VERIFY phi={args.phi} theta={args.theta}: "
              f"tip_orientation_error={error:.4f} deg; "
              f"last_second_phi_range={min(measurements):.4f}.."
              f"{max(measurements):.4f} deg; intervals={intervals}", flush=True)
        passed = error <= 2.0 and max(measurements) - min(measurements) <= 1.0
        print("VERIFY RESULT: " + ("PASS" if passed else "FAIL"), flush=True)
        if not passed:
            raise RuntimeError("Bend tracking/stability tolerance failed")
    finally:
        if controller is not None:
            controller.destroy_node()
        if node is not None:
            node.destroy_node()
        if limb.rclpy.ok():
            limb.rclpy.shutdown()
        limb.simulation_app.close()


if __name__ == "__main__":
    main()
