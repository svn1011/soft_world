#!/usr/bin/env python3

"""Display six live strain-gauge histories from the working soft limb."""

from collections import deque
import time

import matplotlib


matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


HISTORY_SECONDS = 20.0
MAX_SAMPLES = 4000
PLOT_REFRESH_HZ = 20.0
GAUGES = (
    ("one_third", 1, "One-third: gauge 1 (0°)", "tab:red"),
    ("one_third", 2, "One-third: gauge 2 (120°)", "tab:green"),
    ("one_third", 3, "One-third: gauge 3 (240°)", "tab:blue"),
    ("two_thirds", 1, "Two-thirds: gauge 1 (180°)", "tab:orange"),
    ("two_thirds", 2, "Two-thirds: gauge 2 (300°)", "tab:purple"),
    ("two_thirds", 3, "Two-thirds: gauge 3 (60°)", "tab:cyan"),
)


class StrainGaugePlotter(Node):
    """Collect the six individual microstrain topics for live plotting."""

    def __init__(self):
        super().__init__("soft_limb_strain_plotter")
        self.start_time = time.monotonic()
        self.samples = {
            (station, gauge): (
                deque(maxlen=MAX_SAMPLES), deque(maxlen=MAX_SAMPLES)
            )
            for station, gauge, _, _ in GAUGES
        }

        for station, gauge, _, _ in GAUGES:
            topic = (
                f"/soft_limb/sensors/{station}/gauge_{gauge}/strain"
            )
            self.create_subscription(
                Float64,
                topic,
                lambda message, key=(station, gauge): self.record(
                    key, message
                ),
                10,
            )
        self.get_logger().info("Plotting all six strain gauges")

    def record(self, key, message):
        """Append one time-stamped strain sample."""
        times, values = self.samples[key]
        times.append(time.monotonic() - self.start_time)
        values.append(message.data)


def main(args=None):
    rclpy.init(args=args)
    node = StrainGaugePlotter()

    figure, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
    figure.canvas.manager.set_window_title("Soft Limb Strain Gauges")
    figure.suptitle("Live strain-gauge readings")
    lines = {}

    for axis, (station, gauge, title, color) in zip(axes.flat, GAUGES):
        line, = axis.plot([], [], color=color, linewidth=1.5)
        lines[(station, gauge)] = line
        axis.set_title(title)
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Strain (µε)")
        axis.grid(True, alpha=0.3)
        axis.set_xlim(0.0, HISTORY_SECONDS)
        axis.set_ylim(-1.0, 1.0)

    figure.tight_layout()
    plt.show(block=False)
    refresh_period = 1.0 / PLOT_REFRESH_HZ
    last_refresh = 0.0

    try:
        while rclpy.ok() and plt.fignum_exists(figure.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now - last_refresh < refresh_period:
                figure.canvas.flush_events()
                continue
            last_refresh = now

            latest_time = now - node.start_time
            left_time = max(0.0, latest_time - HISTORY_SECONDS)
            right_time = max(HISTORY_SECONDS, latest_time)
            for axis, (station, gauge, _, _) in zip(axes.flat, GAUGES):
                times, values = node.samples[(station, gauge)]
                line = lines[(station, gauge)]
                line.set_data(times, values)
                axis.set_xlim(left_time, right_time)
                if values:
                    minimum = min(values)
                    maximum = max(values)
                    padding = max(1.0, 0.1 * (maximum - minimum))
                    axis.set_ylim(minimum - padding, maximum + padding)

            figure.canvas.draw_idle()
            figure.canvas.flush_events()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(figure)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
