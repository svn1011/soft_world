#!/usr/bin/env python3

"""Publish phi torque and theta position to the 12-disc quadruped."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_COUNT_PER_LIMB = 11
PUBLISH_RATE_HZ = 20.0

MAX_ABS_TORQUE = 1.0

# Phi is the torque-admittance command in N*m. Theta remains a position in
# degrees. Neutral defaults prevent motion until commands are deliberately set.
LIMB_COMMANDS = {
    "body": {"phi_torque": 0.0, "theta": 0.0},
    # Left legs have a net 180-degree axial rotation and right legs have a net
    # 360-degree rotation. The resulting local bend axes let one positive
    # torque sign bend all four legs downward in world space.
    "front_left": {"phi_torque": 0.50, "theta": 0.0},
    "front_right": {"phi_torque": 0.50, "theta": 0.0},
    "rear_left": {"phi_torque": 0.50, "theta": 0.0},
    "rear_right": {"phi_torque": 0.50, "theta": 0.0},
}


def joint_topic_roots(limb_name):
    """Yield topic root and yaw sign for one dual-D6 limb."""
    yield f"/{limb_name}/pair_00/base", 1.0
    for pair_index in range(1, JOINT_COUNT_PER_LIMB):
        yield f"/{limb_name}/pair_{pair_index:02d}/d6_a", 1.0
        yield f"/{limb_name}/pair_{pair_index:02d}/d6_b", -1.0


class QuadrupedHybridPublisher(Node):
    """Publish bend torque and held theta for all five 12-disc limbs."""

    def __init__(self):
        super().__init__("soft_quadruped_hybrid_controller")
        self._command_publishers = {}

        for limb_name, defaults in LIMB_COMMANDS.items():
            self.declare_parameter(
                f"{limb_name}.phi_torque", defaults["phi_torque"]
            )
            self.declare_parameter(
                f"{limb_name}.theta", defaults["theta"]
            )
            for topic_root, _ in joint_topic_roots(limb_name):
                self._command_publishers[(topic_root, "torque")] = (
                    self.create_publisher(
                        Float64, f"{topic_root}/roll/torque", 10
                    )
                )
                self._command_publishers[(topic_root, "theta")] = (
                    self.create_publisher(
                        Float64, f"{topic_root}/yaw/position", 10
                    )
                )

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_commands)
        self.get_logger().info(
            "Publishing 12-disc quadruped phi torque/theta position at "
            f"{PUBLISH_RATE_HZ:.1f} Hz"
        )

    def publish_commands(self):
        """Publish each limb's clamped torque and signed theta target."""
        for limb_name in LIMB_COMMANDS:
            requested_torque = float(
                self.get_parameter(f"{limb_name}.phi_torque").value
            )
            phi_torque = max(
                -MAX_ABS_TORQUE, min(MAX_ABS_TORQUE, requested_torque)
            )
            theta = float(self.get_parameter(f"{limb_name}.theta").value)
            for topic_root, yaw_sign in joint_topic_roots(limb_name):
                torque_message = Float64(data=phi_torque)
                self._command_publishers[(topic_root, "torque")].publish(
                    torque_message
                )

                theta_message = Float64(data=yaw_sign * theta)
                self._command_publishers[(topic_root, "theta")].publish(
                    theta_message
                )


def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedHybridPublisher()
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
