#!/usr/bin/env python3

"""Publish phi bend torque and theta direction to the soft limb."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from limb_configuration_working import JOINT_COUNT


PUBLISH_RATE_HZ = 20.0
MAX_RECOMMENDED_ABS_TORQUE = 1.0

# Phi is the torque-command input in N*m; theta is direction in degrees.
PHI_TORQUE = 0.0
THETA_POSITION_DEGREES = 0.0


class SoftLimbHybridPublisher(Node):
    """Publish phi bend torque and theta bend direction."""

    def __init__(self):
        super().__init__("soft_limb_hybrid_controller")
        self._command_publishers = {}
        self._last_command = None
        self._last_clipped_torque = None

        self.declare_parameter("phi_torque", PHI_TORQUE)
        self.declare_parameter(
            "theta_position", THETA_POSITION_DEGREES
        )

        self._add_joint_publishers("/soft_limb/pair_00/base")
        for pair_index in range(1, JOINT_COUNT):
            self._add_joint_publishers(
                f"/soft_limb/pair_{pair_index:02d}/d6_a"
            )
            self._add_joint_publishers(
                f"/soft_limb/pair_{pair_index:02d}/d6_b"
            )

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_commands)
        self.get_logger().info(
            "Publishing phi bend torque and theta bend direction"
        )

    def _add_joint_publishers(self, topic_root):
        self._command_publishers[(topic_root, "roll")] = (
            self.create_publisher(Float64, f"{topic_root}/roll/torque", 10)
        )
        self._command_publishers[(topic_root, "yaw")] = (
            self.create_publisher(Float64, f"{topic_root}/yaw/position", 10)
        )

    def _publish_joint(self, topic_root, phi_torque, theta_position):
        roll_message = Float64()
        roll_message.data = phi_torque
        self._command_publishers[(topic_root, "roll")].publish(roll_message)

        yaw_message = Float64()
        yaw_message.data = theta_position
        self._command_publishers[(topic_root, "yaw")].publish(yaw_message)

    def publish_commands(self):
        """Publish the current hybrid command to every D6 joint."""
        requested_torque = float(self.get_parameter("phi_torque").value)
        phi_torque = max(
            -MAX_RECOMMENDED_ABS_TORQUE,
            min(MAX_RECOMMENDED_ABS_TORQUE, requested_torque),
        )
        if requested_torque != phi_torque:
            if requested_torque != self._last_clipped_torque:
                self.get_logger().warning(
                    f"Requested torque {requested_torque:.6f} N*m exceeds "
                    f"the working range; using {phi_torque:.6f} N*m. "
                    "Commands above the useful range only force joint stops."
                )
            self._last_clipped_torque = requested_torque
        else:
            self._last_clipped_torque = None
        theta_position = float(
            self.get_parameter("theta_position").value
        )

        command = (phi_torque, theta_position)
        if command != self._last_command:
            self.get_logger().info(
                f"Phi/roll={phi_torque:.3f} N*m, "
                f"theta/direction={theta_position:.3f} degrees"
            )
            self._last_command = command

        self._publish_joint(
            "/soft_limb/pair_00/base", phi_torque, theta_position
        )
        for pair_index in range(1, JOINT_COUNT):
            self._publish_joint(
                f"/soft_limb/pair_{pair_index:02d}/d6_a",
                phi_torque,
                theta_position,
            )
            self._publish_joint(
                f"/soft_limb/pair_{pair_index:02d}/d6_b",
                phi_torque,
                theta_position,
            )


def main(args=None):
    rclpy.init(args=args)
    node = SoftLimbHybridPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
