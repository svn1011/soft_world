#!/usr/bin/env python3

"""Publish bend magnitude and direction to the dual-D6 soft limb."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_COUNT = 5
PUBLISH_RATE_HZ = 20.0
PHI_POSITION_DEGREES = 0.0
THETA_POSITION_DEGREES = 0.0

j
class SoftLimbPositionPublisher(Node):
    """Publish phi bend magnitude and theta direction."""

    def __init__(self):
        super().__init__("soft_limb_position_controller")
        self._position_publishers = {}
        self._last_command = None

        self.declare_parameter("phi_position", PHI_POSITION_DEGREES)
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

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_positions)
        self.get_logger().info(
            "Publishing phi bend position and theta bend direction"
        )

    def _add_joint_publishers(self, topic_root):
        self._position_publishers[(topic_root, "roll")] = (
            self.create_publisher(Float64, f"{topic_root}/roll/position", 10)
        )
        self._position_publishers[(topic_root, "yaw")] = (
            self.create_publisher(Float64, f"{topic_root}/yaw/position", 10)
        )

    def _publish_joint(self, topic_root, phi_position, theta_position):
        roll_message = Float64()
        roll_message.data = phi_position
        self._position_publishers[(topic_root, "roll")].publish(roll_message)

        yaw_message = Float64()
        yaw_message.data = theta_position
        self._position_publishers[(topic_root, "yaw")].publish(yaw_message)

    def publish_positions(self):
        """Distribute one total constant-curvature arc command."""
        phi_position = float(self.get_parameter("phi_position").value)
        theta_position = float(
            self.get_parameter("theta_position").value
        )

        command = (phi_position, theta_position)
        if command != self._last_command:
            self.get_logger().info(
                f"Phi/roll={phi_position:.3f} degrees, "
                f"theta/direction={theta_position:.3f} degrees"
            )
            self._last_command = command

        interval_bend = phi_position / JOINT_COUNT
        self._publish_joint(
            "/soft_limb/pair_00/base", interval_bend, theta_position
        )
        for pair_index in range(1, JOINT_COUNT):
            self._publish_joint(
                f"/soft_limb/pair_{pair_index:02d}/d6_a",
                0.5 * interval_bend,
                theta_position,
            )
            self._publish_joint(
                f"/soft_limb/pair_{pair_index:02d}/d6_b",
                0.5 * interval_bend,
                theta_position,
            )


def main(args=None):
    rclpy.init(args=args)
    node = SoftLimbPositionPublisher()
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
