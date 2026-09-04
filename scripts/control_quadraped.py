#!/usr/bin/env python3

"""Continuously publish held phi/theta positions to five soft limbs."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


JOINT_COUNT_PER_LIMB = 11
PUBLISH_RATE_HZ = 20.0

# Position targets are in degrees and are applied to every section of a limb.
# Keep each value within the +/-25 degree joint limit.
LIMB_POSITIONS = {
    "body": {"phi": 5.0, "theta": 0.0},
    "front_left": {"phi": -5.0, "theta": 0.0},
    "front_right": {"phi": 5.0, "theta": 0.0},
    "rear_left": {"phi": -5.0, "theta": 0.0},
    "rear_right": {"phi": 5.0, "theta": 0.0},
}


def joint_topic_roots(limb_name):
    """Yield topic root and yaw sign for one dual-D6 limb."""
    yield f"/{limb_name}/pair_00/base", 1.0
    for pair_index in range(1, JOINT_COUNT_PER_LIMB):
        yield f"/{limb_name}/pair_{pair_index:02d}/d6_a", 1.0
        yield f"/{limb_name}/pair_{pair_index:02d}/d6_b", -1.0


class QuadrupedPositionPublisher(Node):
    """Publish roll/yaw drive targets for all five dual-D6 limbs."""

    def __init__(self):
        super().__init__("soft_quadruped_position_controller")
        self._position_publishers = {}

        for limb_name in LIMB_POSITIONS:
            for topic_root, _ in joint_topic_roots(limb_name):
                for axis in ("roll", "yaw"):
                    self._position_publishers[(topic_root, axis)] = (
                        self.create_publisher(
                            Float64,
                            f"{topic_root}/{axis}/position",
                            10,
                        )
                    )

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_positions)
        self.get_logger().info(
            "Publishing quadruped roll/yaw position targets at "
            f"{PUBLISH_RATE_HZ:.1f} Hz"
        )

    def publish_positions(self):
        """Publish each limb's phi and signed theta targets."""
        for limb_name, commands in LIMB_POSITIONS.items():
            for topic_root, yaw_sign in joint_topic_roots(limb_name):
                roll_message = Float64()
                roll_message.data = float(commands["phi"])
                self._position_publishers[(topic_root, "roll")].publish(
                    roll_message
                )

                yaw_message = Float64()
                yaw_message.data = yaw_sign * float(commands["theta"])
                self._position_publishers[(topic_root, "yaw")].publish(
                    yaw_message
                )


def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedPositionPublisher()
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
