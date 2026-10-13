#!/usr/bin/env python3

"""Verify independent three-module snake commands and all 18 sensors."""

import argparse
import os


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--torques", nargs=3, type=float, default=[0.5, 0.75, 1.0])
    parser.add_argument("--thetas", nargs=3, type=float, default=[0.0, 90.0, -90.0])
    parser.add_argument("--seconds", type=float, default=12.0)
    args = parser.parse_args()
    os.environ["SOFT_SNAKE_HEADLESS"] = "1"

    import load_soft_3_working as snake

    np = snake.np
    node = None
    controller = None
    try:
        if not snake.open_stage(snake.get_usd_path()):
            raise RuntimeError("Could not open the three-module snake USD")
        from omni.usd import get_context
        stage = get_context().get_stage()
        snake.validate_stage(stage)
        snake.configure_stage(stage)
        world = snake.World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / snake.PHYSICS_FREQUENCY_HZ,
            rendering_dt=1.0 / 60.0,
        )
        disks = world.scene.add(snake.RigidPrim(
            prim_paths_expr=list(snake.DISK_PATHS),
            name="verify_soft_3_working_disks",
            reset_xform_properties=False,
        ))
        world.reset()
        disks.set_sleep_thresholds(np.zeros(snake.DISK_COUNT))
        snake.rclpy.init()
        node = snake.Soft3WorkingCommandSubscriber(stage)
        from control_soft_3_working import Soft3WorkingController
        from rclpy.parameter import Parameter
        controller = Soft3WorkingController()
        parameters = []
        expected_torques = []
        expected_thetas = []
        for module_number, (torque, theta) in enumerate(
            zip(args.torques, args.thetas), start=1
        ):
            expected_torque = float(np.clip(
                torque, -snake.MAX_ABS_TORQUE, snake.MAX_ABS_TORQUE
            ))
            expected_theta = (theta + 180.0) % 360.0 - 180.0
            expected_torques.append(expected_torque)
            expected_thetas.append(expected_theta)
            parameters.extend([
                Parameter(
                    f"module_{module_number}_phi_torque", value=torque
                ),
                Parameter(
                    f"module_{module_number}_theta_position", value=theta
                ),
            ])
        controller.set_parameters(parameters)

        import time
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            controller.publish_commands()
            snake.rclpy.spin_once(node, timeout_sec=0.01)
            if (
                np.allclose(node.torque_commands, expected_torques)
                and np.allclose(node.theta_commands, expected_thetas)
            ):
                break
        else:
            raise RuntimeError("All three command pairs were not delivered")

        node.update_drive_targets(1.0 / snake.PHYSICS_FREQUENCY_HZ)
        first_targets = []
        for module_index in range(snake.MODULE_COUNT):
            module_targets = []
            for interval_index in range(snake.INTERVALS_PER_MODULE):
                prim = stage.GetPrimAtPath(
                    snake.module_joint_path(module_index, interval_index)
                )
                module_targets.append([
                    snake.UsdPhysics.DriveAPI(
                        prim, "rotX"
                    ).GetTargetPositionAttr().Get(),
                    snake.UsdPhysics.DriveAPI(
                        prim, "rotY"
                    ).GetTargetPositionAttr().Get(),
                ])
            if not np.allclose(
                module_targets,
                np.repeat([module_targets[0]], snake.INTERVALS_PER_MODULE, axis=0),
                atol=1.0e-6,
            ):
                raise RuntimeError(
                    f"Module {module_index + 1} intervals are not synchronized: "
                    f"{module_targets}"
                )
            first_targets.append(module_targets[0])
        print(f"VERIFY first-step module targets: {first_targets}", flush=True)

        forces = np.zeros((snake.DISK_COUNT, 3), dtype=np.float32)
        forces[:, 2] = snake.DISK_MASS * snake.GRAVITY_ACCELERATION
        torques = np.zeros((snake.DISK_COUNT, 3), dtype=np.float32)
        interface = snake.get_physx_interface()
        world.play()
        history = [[] for _ in range(snake.MODULE_COUNT)]
        steps = int(args.seconds * snake.PHYSICS_FREQUENCY_HZ)
        for step in range(steps):
            node.update_drive_targets(1.0 / snake.PHYSICS_FREQUENCY_HZ)
            disks.apply_forces_and_torques_at_pos(
                forces=forces, torques=torques, is_global=True
            )
            world.step(render=False)
            interface.update_transformations(False, True, True)
            if step % 24 == 0:
                world.render()
            if step >= steps - int(snake.PHYSICS_FREQUENCY_HZ):
                _, orientations = disks.get_world_poses()
                for module_index in range(snake.MODULE_COUNT):
                    start = snake.module_start_disk(module_index)
                    bend = snake.accumulated_bend_vector(
                        orientations[start:start + snake.DISKS_PER_MODULE]
                    )
                    history[module_index].append(
                        float(np.rad2deg(np.linalg.norm(bend)))
                    )

        positions, orientations = disks.get_world_poses()
        node.publish_measurements(positions, orientations)
        sensor_count = sum(
            len(publishers["gauges"])
            for publishers in node.sensor_publishers.values()
        )
        if sensor_count != 18:
            raise RuntimeError(f"Expected 18 gauges, found {sensor_count}")

        passed = True
        reports = []
        for module_index in range(snake.MODULE_COUNT):
            start = snake.module_start_disk(module_index)
            module_orientations = orientations[
                start:start + snake.DISKS_PER_MODULE
            ]
            bend_x, bend_y = snake.accumulated_bend_vector(module_orientations)
            measured_phi = float(np.rad2deg(np.hypot(bend_x, bend_y)))
            measured_theta = float(np.rad2deg(np.arctan2(-bend_x, bend_y)))
            expected_phi = (
                abs(expected_torques[module_index])
                / snake.MAX_ABS_TORQUE
                * snake.MAX_TOTAL_MODULE_BEND_DEGREES
            )
            intervals = [
                float(np.rad2deg(np.linalg.norm(
                    snake.quaternion_rotation_vector(
                        snake.relative_quaternion(previous, current)
                    )
                )))
                for previous, current in zip(
                    module_orientations[:-1], module_orientations[1:]
                )
            ]
            tracking_error = abs(measured_phi - expected_phi)
            stability_range = max(history[module_index]) - min(history[module_index])
            interval_spread = max(intervals) - min(intervals)
            module_passed = (
                tracking_error <= 2.0
                and stability_range <= 1.0
                and interval_spread <= 1.0
            )
            passed = passed and module_passed
            reports.append({
                "module": module_index + 1,
                "expected_phi": expected_phi,
                "measured_phi": measured_phi,
                "measured_theta": measured_theta,
                "intervals": intervals,
                "tracking_error": tracking_error,
                "stability_range": stability_range,
                "pass": module_passed,
            })

        connector_bends = []
        for upper_index in (5, 11):
            connector_bends.append(float(np.rad2deg(np.linalg.norm(
                snake.quaternion_rotation_vector(
                    snake.relative_quaternion(
                        orientations[upper_index], orientations[upper_index + 1]
                    )
                )
            ))))
        if max(connector_bends) > 0.1:
            passed = False
        print(f"VERIFY modules: {reports}", flush=True)
        print(f"VERIFY fixed connector bends: {connector_bends}", flush=True)
        print(f"VERIFY sensor gauge publishers: {sensor_count}", flush=True)
        print("VERIFY RESULT: " + ("PASS" if passed else "FAIL"), flush=True)
        if not passed:
            raise RuntimeError("Three-module snake regression failed")
    finally:
        if controller is not None:
            controller.destroy_node()
        if node is not None:
            node.destroy_node()
        if snake.rclpy.ok():
            snake.rclpy.shutdown()
        snake.simulation_app.close()


if __name__ == "__main__":
    main()
