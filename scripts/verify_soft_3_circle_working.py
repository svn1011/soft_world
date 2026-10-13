#!/usr/bin/env python3

"""Headless verification of the minimum-energy horizontal circle controller."""

import os
import time


def main():
    os.environ["SOFT_SNAKE_HEADLESS"] = "1"
    import load_soft_3_working as snake

    np = snake.np
    command_node = None
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
            name="verify_soft_3_circle_disks",
            reset_xform_properties=False,
        ))
        world.reset()
        disks.set_sleep_thresholds(np.zeros(snake.DISK_COUNT))
        snake.rclpy.init()
        command_node = snake.Soft3WorkingCommandSubscriber(stage)
        from control_soft_3_circle_working import MinimumEnergyCircleController
        controller = MinimumEnergyCircleController()

        forces = np.zeros((snake.DISK_COUNT, 3), dtype=np.float32)
        forces[:, 2] = snake.DISK_MASS * snake.GRAVITY_ACCELERATION
        torques = np.zeros((snake.DISK_COUNT, 3), dtype=np.float32)
        interface = snake.get_physx_interface()
        world.play()
        dt = 1.0 / snake.PHYSICS_FREQUENCY_HZ
        duration = controller.settle_seconds + controller.period
        steps = int(duration * snake.PHYSICS_FREQUENCY_HZ)
        positions_history = []
        targets_history = []
        start_clock = time.monotonic()
        for step in range(steps + 1):
            simulated_time = step * dt
            if step % 12 == 0:
                controller.started = time.monotonic() - simulated_time
                controller.publish_trajectory()
                snake.rclpy.spin_once(command_node, timeout_sec=0.0)
            command_node.update_drive_targets(dt)
            disks.apply_forces_and_torques_at_pos(
                forces=forces, torques=torques, is_global=True
            )
            world.step(render=False)
            interface.update_transformations(False, True, True)
            if step % 24 == 0:
                positions, orientations = disks.get_world_poses()
                command_node.publish_measurements(positions, orientations)
                snake.rclpy.spin_once(controller, timeout_sec=0.0)
                if simulated_time >= controller.settle_seconds:
                    angle = controller.trajectory_angle(simulated_time)
                    target = np.array([
                        controller.base[0] + controller.radius * np.cos(angle),
                        controller.base[1] + controller.radius * np.sin(angle),
                        controller.height,
                    ])
                    positions_history.append(np.asarray(positions[-1]))
                    targets_history.append(target)

        points = np.asarray(positions_history)
        targets = np.asarray(targets_history)
        radial = np.linalg.norm(points[:, :2] - controller.base[:2], axis=1)
        errors = np.linalg.norm(points - targets, axis=1)
        report = {
            "samples": len(points),
            "mean_radius_m": float(np.mean(radial)),
            "radius_peak_to_peak_m": float(np.ptp(radial)),
            "height_mean_m": float(np.mean(points[:, 2])),
            "height_peak_to_peak_m": float(np.ptp(points[:, 2])),
            "rms_target_error_m": float(np.sqrt(np.mean(errors ** 2))),
            "max_abs_torque_Nm": float(np.max(np.abs(controller.torques))),
            "energy_proxy": float(controller.solution["energy_proxy"]),
            "wall_seconds": time.monotonic() - start_clock,
        }
        print(f"CIRCLE VERIFY: {report}", flush=True)
        passed = (
            len(points) >= 100
            and report["height_peak_to_peak_m"] <= 0.015
            and report["radius_peak_to_peak_m"] <= 0.025
            and report["rms_target_error_m"] <= 0.050
            and report["max_abs_torque_Nm"] <= 1.0
        )
        print("CIRCLE VERIFY RESULT: " + ("PASS" if passed else "FAIL"), flush=True)
        if not passed:
            raise RuntimeError("Minimum-energy circular trajectory regression failed")
    finally:
        if controller is not None:
            controller.destroy_node()
        if command_node is not None:
            command_node.destroy_node()
        if snake.rclpy.ok():
            snake.rclpy.shutdown()
        snake.simulation_app.close()


if __name__ == "__main__":
    main()
