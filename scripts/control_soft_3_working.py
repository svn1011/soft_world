#!/usr/bin/env python3

"""Publish three independent phi-torque and theta commands to the soft snake."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


MODULE_COUNT = 3
PUBLISH_RATE_HZ = 20.0
MAX_ABS_TORQUE = 1.0


class Soft3WorkingController(Node):
    def __init__(self):
        super().__init__("soft_3_working_controller")
        # Do not shadow rclpy.Node._publishers; rclpy owns that internal list.
        self._module_publishers = []
        self._last_commands = [None] * MODULE_COUNT
        self._last_clipped = [None] * MODULE_COUNT
        for module_number in range(1, MODULE_COUNT + 1):
            self.declare_parameter(f"module_{module_number}_phi_torque", 0.0)
            self.declare_parameter(f"module_{module_number}_theta_position", 0.0)
            root = f"/soft_snake/module_{module_number}"
            self._module_publishers.append((
                self.create_publisher(Float64, f"{root}/phi_torque", 10),
                self.create_publisher(Float64, f"{root}/theta_position", 10),
            ))
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_commands)
        self.get_logger().info(
            "Publishing three independent snake module torque/direction pairs"
        )

    def publish_commands(self):
        for module_index, (torque_publisher, theta_publisher) in enumerate(
            self._module_publishers
        ):
            module_number = module_index + 1
            requested = float(self.get_parameter(
                f"module_{module_number}_phi_torque"
            ).value)
            torque = max(-MAX_ABS_TORQUE, min(MAX_ABS_TORQUE, requested))
            if requested != torque and requested != self._last_clipped[module_index]:
                self.get_logger().warning(
                    f"Module {module_number}: requested {requested:.3f} N*m; "
                    f"using the {torque:.3f} N*m working limit"
                )
                self._last_clipped[module_index] = requested
            elif requested == torque:
                self._last_clipped[module_index] = None
            theta = float(self.get_parameter(
                f"module_{module_number}_theta_position"
            ).value)
            theta = (theta + 180.0) % 360.0 - 180.0
            command = (torque, theta)
            if command != self._last_commands[module_index]:
                self.get_logger().info(
                    f"Module {module_number}: phi torque={torque:.3f} N*m, "
                    f"theta={theta:.3f} degrees"
                )
                self._last_commands[module_index] = command
            torque_publisher.publish(Float64(data=torque))
            theta_publisher.publish(Float64(data=theta))


def main(args=None):
    rclpy.init(args=args)
    node = Soft3WorkingController()
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
