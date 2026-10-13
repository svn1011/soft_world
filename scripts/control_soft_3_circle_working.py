#!/usr/bin/env python3

"""Move the three-module snake tip on a minimum-energy horizontal circle."""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from minimum_energy_circle_working import (
    DEFAULT_CIRCLE_RADIUS_M,
    circle_target,
    solve_minimum_energy_posture,
)


MODULE_COUNT = 3
PUBLISH_RATE_HZ = 20.0


def wrap_degrees(value):
    return (float(value) + 180.0) % 360.0 - 180.0


class MinimumEnergyCircleController(Node):
    def __init__(self):
        super().__init__("soft_3_minimum_energy_circle_controller")
        self.declare_parameter("circle_radius_m", DEFAULT_CIRCLE_RADIUS_M)
        self.declare_parameter("circle_height_m", 0.40)
        self.declare_parameter("circle_period_s", 54.0)
        self.declare_parameter("settle_seconds", 6.0)
        self.declare_parameter("start_angle_deg", 0.0)
        self.declare_parameter("clockwise", False)
        self.declare_parameter("base_x_m", 0.0)
        self.declare_parameter("base_y_m", 0.0)
        self.declare_parameter("base_z_m", 0.75)
        self.declare_parameter("module_energy_weights", [1.0, 1.0, 1.0])

        self.radius = float(self.get_parameter("circle_radius_m").value)
        self.height = float(self.get_parameter("circle_height_m").value)
        self.period = float(self.get_parameter("circle_period_s").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.start_angle = math.radians(float(
            self.get_parameter("start_angle_deg").value
        ))
        self.clockwise = bool(self.get_parameter("clockwise").value)
        self.base = np.array([
            float(self.get_parameter("base_x_m").value),
            float(self.get_parameter("base_y_m").value),
            float(self.get_parameter("base_z_m").value),
        ])
        weights = np.asarray(
            self.get_parameter("module_energy_weights").value,
            dtype=np.float64,
        )
        if self.period <= 0.0:
            raise ValueError("circle_period_s must be positive")
        if self.settle_seconds < 0.0:
            raise ValueError("settle_seconds cannot be negative")
        vertical_drop = self.base[2] - self.height
        self.solution = solve_minimum_energy_posture(
            self.radius, vertical_drop, weights
        )
        self.torques = self.solution["torques_Nm"]
        self.actual_tip = None
        self.started = time.monotonic()
        self.last_log = 0.0

        self.command_publishers = []
        for module in range(1, MODULE_COUNT + 1):
            root = f"/soft_snake/module_{module}"
            self.command_publishers.append((
                self.create_publisher(Float64, f"{root}/phi_torque", 10),
                self.create_publisher(Float64, f"{root}/theta_position", 10),
            ))
        self.target_publisher = self.create_publisher(
            Float64MultiArray, "/soft_snake/circle/target_tip_position", 10
        )
        self.energy_publisher = self.create_publisher(
            Float64, "/soft_snake/circle/normalized_command_energy", 10
        )
        self.error_publisher = self.create_publisher(
            Float64, "/soft_snake/circle/tip_position_error", 10
        )
        self.create_subscription(
            Float64MultiArray, "/soft_snake/tip/world_position",
            self._tip_callback, 10,
        )
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_trajectory)
        bends_deg = np.degrees(self.solution["module_bends_rad"])
        self.get_logger().info(
            "Minimum-energy circle solved: radius=%.3f m, height=%.3f m, "
            "module bends=%s deg, torques=%s N*m, energy proxy=%.6f"
            % (
                self.radius, self.height,
                np.array2string(bends_deg, precision=3),
                np.array2string(self.torques, precision=4),
                self.solution["energy_proxy"],
            )
        )

    def _tip_callback(self, message):
        if len(message.data) >= 3:
            self.actual_tip = np.asarray(message.data[:3], dtype=np.float64)

    def trajectory_angle(self, elapsed):
        moving_time = max(0.0, elapsed - self.settle_seconds)
        direction = -1.0 if self.clockwise else 1.0
        return self.start_angle + direction * 2.0 * math.pi * moving_time / self.period

    def publish_trajectory(self):
        elapsed = time.monotonic() - self.started
        angle = self.trajectory_angle(elapsed)
        # theta=angle+180 degrees maps the loader's bend axes toward +radial.
        theta_degrees = wrap_degrees(math.degrees(angle) + 180.0)
        for torque, (torque_publisher, theta_publisher) in zip(
            self.torques, self.command_publishers
        ):
            torque_publisher.publish(Float64(data=float(torque)))
            theta_publisher.publish(Float64(data=theta_degrees))

        target = circle_target(self.base, self.radius, self.height, angle)
        target_message = Float64MultiArray()
        target_message.data = target.tolist()
        self.target_publisher.publish(target_message)
        self.energy_publisher.publish(Float64(
            data=float(self.solution["energy_proxy"])
        ))
        if self.actual_tip is not None:
            error = float(np.linalg.norm(target - self.actual_tip))
            self.error_publisher.publish(Float64(data=error))
            if elapsed - self.last_log >= 2.0:
                self.get_logger().info(
                    f"target={np.round(target, 4).tolist()} m, "
                    f"tip={np.round(self.actual_tip, 4).tolist()} m, "
                    f"error={1000.0 * error:.1f} mm"
                )
                self.last_log = elapsed

    def publish_zero(self):
        for torque_publisher, theta_publisher in self.command_publishers:
            torque_publisher.publish(Float64(data=0.0))
            theta_publisher.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MinimumEnergyCircleController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            for _ in range(5):
                node.publish_zero()
                rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
