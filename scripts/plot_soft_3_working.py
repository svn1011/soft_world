#!/usr/bin/env python3

"""Plot all 18 strain or bridge-voltage channels from the soft snake."""

from collections import deque
import math
import os
import time

import matplotlib


matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


SIGNAL = os.environ.get("SOFT_SNAKE_PLOT_SIGNAL", "strain")
SIGNAL_LABELS = {
    "strain": ("Strain (µε)", "Soft Snake: 18 Strain Gauges"),
    "bridge_voltage": ("Bridge output (V)", "Soft Snake: 18 Bridge Voltages"),
}
HISTORY_SECONDS = 20.0
MAX_SAMPLES = 4000
DISPLAY_BOUNDARY_VOLTS = 1.25
RINGS = (
    ("one_third", (0, 120, 240)),
    ("two_thirds", (180, 300, 60)),
)


class Soft3WorkingSensorPlotter(Node):
    def __init__(self):
        if SIGNAL not in SIGNAL_LABELS:
            raise ValueError(f"Unsupported signal: {SIGNAL}")
        super().__init__(f"soft_3_working_18_{SIGNAL}_plotter")
        self.start_time = time.monotonic()
        self.samples = {}
        self.warned_nonfinite = False
        for module in range(1, 4):
            for station, angles in RINGS:
                for gauge, angle in enumerate(angles, start=1):
                    key = (module, station, gauge, angle)
                    self.samples[key] = (
                        deque(maxlen=MAX_SAMPLES), deque(maxlen=MAX_SAMPLES)
                    )
                    topic = (
                        f"/soft_snake/module_{module}/sensors/{station}/"
                        f"gauge_{gauge}/{SIGNAL}"
                    )
                    self.create_subscription(
                        Float64, topic,
                        lambda message, sample_key=key: self.record(
                            sample_key, message
                        ),
                        10,
                    )

    def record(self, key, message):
        times, values = self.samples[key]
        value = message.data
        if SIGNAL == "bridge_voltage" and not math.isfinite(value):
            value = DISPLAY_BOUNDARY_VOLTS
            if not self.warned_nonfinite:
                self.get_logger().warning(
                    "Displaying non-finite bridge output at the +1.25 V "
                    "invalid-model boundary"
                )
                self.warned_nonfinite = True
        times.append(time.monotonic() - self.start_time)
        values.append(value)


def main(args=None):
    rclpy.init(args=args)
    node = Soft3WorkingSensorPlotter()
    ylabel, title = SIGNAL_LABELS[SIGNAL]
    figure, axes = plt.subplots(6, 3, figsize=(14, 13), sharex=True)
    figure.canvas.manager.set_window_title(title)
    figure.suptitle(title)
    lines = {}
    keys = list(node.samples)
    colors = ("tab:red", "tab:green", "tab:blue")
    for axis, key in zip(axes.flat, keys):
        module, station, gauge, angle = key
        line, = axis.plot([], [], color=colors[gauge - 1], linewidth=1.2)
        lines[key] = line
        ring = "1/3" if station == "one_third" else "2/3"
        axis.set_title(f"M{module} • {ring} • G{gauge} ({angle}°)", fontsize=9)
        axis.set_ylabel(ylabel, fontsize=8)
        axis.grid(True, alpha=0.25)
    for axis in axes[-1]:
        axis.set_xlabel("Time (s)")
    figure.tight_layout()
    plt.show(block=False)

    try:
        while rclpy.ok() and plt.fignum_exists(figure.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            latest = time.monotonic() - node.start_time
            left = max(0.0, latest - HISTORY_SECONDS)
            right = max(HISTORY_SECONDS, latest)
            for axis, key in zip(axes.flat, keys):
                times, values = node.samples[key]
                lines[key].set_data(times, values)
                axis.set_xlim(left, right)
                if values:
                    low, high = min(values), max(values)
                    minimum_padding = 0.02 if SIGNAL == "bridge_voltage" else 1.0
                    padding = max(minimum_padding, 0.1 * (high - low))
                    axis.set_ylim(low - padding, high + padding)
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
