#!/usr/bin/env python3

"""Run repeatable soft_3_working voltage sweeps and save an XLSX dataset."""

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from xlsx_writer_working import write_xlsx


MODULE_COUNT = 3
COMMAND_RATE_HZ = 20.0
SENSOR_STATIONS = (
    ("one_third", "1/3", (0, 120, 240)),
    ("two_thirds", "2/3", (180, 300, 60)),
)
SENSORS = tuple(
    (module, station, station_label, gauge, angle)
    for module in range(1, MODULE_COUNT + 1)
    for station, station_label, angles in SENSOR_STATIONS
    for gauge, angle in enumerate(angles, start=1)
)


def sensor_id(sensor):
    module, _, station_label, gauge, angle = sensor
    return f"M{module}_{station_label}_G{gauge}_{angle}deg"


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Collect all 18 soft_3_working bridge voltages into Excel."
    )
    parser.add_argument(
        "--torques", nargs="+", type=float,
        default=[0.25, 0.50, 0.75, 1.00],
        help="Torque commands in N*m (default: 0.25 0.50 0.75 1.00).",
    )
    parser.add_argument(
        "--thetas", nargs="+", type=float,
        default=[-90.0, -45.0, 0.0, 45.0, 90.0],
        help="Bending directions in degrees.",
    )
    parser.add_argument(
        "--sweep-mode", choices=("one_at_a_time", "all_same"),
        default="one_at_a_time",
        help="Isolate each module or command all modules together.",
    )
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--record-seconds", type=float, default=1.0)
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument("--sensor-timeout", type=float, default=60.0)
    parser.add_argument(
        "--output", type=Path,
        help="Output .xlsx path. Default: research_data_working/timestamped file.",
    )
    parser.add_argument(
        "--allow-competing-controller", action="store_true",
        help="Allow another soft_3_working controller to publish concurrently.",
    )
    return parser.parse_args()


class VoltageExperimentNode(Node):
    def __init__(self):
        super().__init__("soft_3_working_voltage_collector")
        self.command = [(0.0, 0.0)] * MODULE_COUNT
        self.voltages = {}
        self.validity = {}
        self.arrival_times = {}
        self.measured = {
            module: {"phi": None, "theta": None}
            for module in range(1, MODULE_COUNT + 1)
        }
        self.command_publishers = []
        for module in range(1, MODULE_COUNT + 1):
            root = f"/soft_snake/module_{module}"
            self.command_publishers.append((
                self.create_publisher(Float64, f"{root}/phi_torque", 10),
                self.create_publisher(Float64, f"{root}/theta_position", 10),
            ))
            self.create_subscription(
                Float64, f"{root}/measured_phi",
                lambda message, number=module: self._measured(number, "phi", message),
                10,
            )
            self.create_subscription(
                Float64, f"{root}/measured_theta",
                lambda message, number=module: self._measured(number, "theta", message),
                10,
            )
        for sensor in SENSORS:
            module, station, _, gauge, _ = sensor
            root = f"/soft_snake/module_{module}/sensors/{station}/gauge_{gauge}"
            self.create_subscription(
                Float64, f"{root}/bridge_voltage",
                lambda message, key=sensor: self._voltage(key, message), 10,
            )
            self.create_subscription(
                Bool, f"{root}/bridge_valid",
                lambda message, key=sensor: self._valid(key, message), 10,
            )
        self.create_timer(1.0 / COMMAND_RATE_HZ, self.publish_commands)

    def _voltage(self, key, message):
        self.voltages[key] = float(message.data)
        self.arrival_times[key] = time.monotonic()

    def _valid(self, key, message):
        self.validity[key] = bool(message.data)

    def _measured(self, module, field, message):
        self.measured[module][field] = float(message.data)

    def set_commands(self, commands):
        self.command = list(commands)
        self.publish_commands()

    def publish_commands(self):
        for (torque, theta), (torque_pub, theta_pub) in zip(
            self.command, self.command_publishers
        ):
            torque_pub.publish(Float64(data=float(torque)))
            theta_pub.publish(Float64(data=float(theta)))

    def sensors_ready(self, maximum_age=0.5):
        now = time.monotonic()
        return all(
            sensor in self.voltages
            and sensor in self.validity
            and now - self.arrival_times.get(sensor, 1.0e9) <= maximum_age
            for sensor in SENSORS
        )


def spin_for(node, seconds):
    deadline = time.monotonic() + max(0.0, seconds)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))


def build_cases(args):
    cases = [
        {"label": "baseline", "commands": [(0.0, 0.0)] * MODULE_COUNT}
    ]
    if args.sweep_mode == "one_at_a_time":
        for module in range(1, MODULE_COUNT + 1):
            for torque in args.torques:
                for theta in args.thetas:
                    commands = [(0.0, 0.0)] * MODULE_COUNT
                    commands[module - 1] = (torque, theta)
                    cases.append({
                        "label": f"module_{module}", "commands": commands,
                    })
    else:
        for torque in args.torques:
            for theta in args.thetas:
                cases.append({
                    "label": "all_modules",
                    "commands": [(torque, theta)] * MODULE_COUNT,
                })
    return cases


def sample_header():
    header = ["recorded_at_utc", "elapsed_s", "case_id", "repetition", "case_label"]
    for module in range(1, MODULE_COUNT + 1):
        header.extend([
            f"module_{module}_torque_Nm", f"module_{module}_theta_deg",
            f"module_{module}_measured_phi_deg", f"module_{module}_measured_theta_deg",
        ])
    header.extend(f"{sensor_id(sensor)}_V" for sensor in SENSORS)
    header.extend(f"{sensor_id(sensor)}_valid" for sensor in SENSORS)
    return header


def make_sample_row(node, started, case_id, repetition, case):
    row = [
        datetime.now(timezone.utc).isoformat(), time.monotonic() - started,
        case_id, repetition, case["label"],
    ]
    for module, (torque, theta) in enumerate(case["commands"], start=1):
        row.extend([
            torque, theta, node.measured[module]["phi"],
            node.measured[module]["theta"],
        ])
    row.extend(node.voltages.get(sensor) for sensor in SENSORS)
    row.extend(node.validity.get(sensor) for sensor in SENSORS)
    return row


def summarize(samples, header):
    index = {name: position for position, name in enumerate(header)}
    grouped = defaultdict(list)
    for row in samples:
        grouped[(row[index["case_id"]], row[index["repetition"]])].append(row)

    case_rows = [[
        "case_id", "repetition", "case_label", "sample_count",
        "module_1_torque_Nm", "module_1_theta_deg", "module_1_mean_phi_deg", "module_1_mean_theta_deg",
        "module_2_torque_Nm", "module_2_theta_deg", "module_2_mean_phi_deg", "module_2_mean_theta_deg",
        "module_3_torque_Nm", "module_3_theta_deg", "module_3_mean_phi_deg", "module_3_mean_theta_deg",
    ]]
    sensor_rows = [[
        "case_id", "repetition", "case_label", "sensor_id", "module",
        "station", "gauge", "angle_deg", "sample_count", "mean_voltage_V",
        "stdev_voltage_V", "minimum_voltage_V", "maximum_voltage_V", "valid_fraction",
    ]]
    for (case_id, repetition), rows in sorted(grouped.items()):
        first = rows[0]
        case_row = [case_id, repetition, first[index["case_label"]], len(rows)]
        for module in range(1, MODULE_COUNT + 1):
            phi_values = [
                row[index[f"module_{module}_measured_phi_deg"]] for row in rows
                if row[index[f"module_{module}_measured_phi_deg"]] is not None
            ]
            theta_values = [
                row[index[f"module_{module}_measured_theta_deg"]] for row in rows
                if row[index[f"module_{module}_measured_theta_deg"]] is not None
            ]
            case_row.extend([
                first[index[f"module_{module}_torque_Nm"]],
                first[index[f"module_{module}_theta_deg"]],
                statistics.fmean(phi_values) if phi_values else None,
                statistics.fmean(theta_values) if theta_values else None,
            ])
        case_rows.append(case_row)

        for sensor in SENSORS:
            voltage_column = index[f"{sensor_id(sensor)}_V"]
            valid_column = index[f"{sensor_id(sensor)}_valid"]
            values = [
                row[voltage_column] for row in rows
                if row[voltage_column] is not None and math.isfinite(row[voltage_column])
            ]
            valid_values = [bool(row[valid_column]) for row in rows if row[valid_column] is not None]
            module, _, station_label, gauge, angle = sensor
            sensor_rows.append([
                case_id, repetition, first[index["case_label"]], sensor_id(sensor),
                module, station_label, gauge, angle, len(values),
                statistics.fmean(values) if values else None,
                statistics.stdev(values) if len(values) > 1 else 0.0 if values else None,
                min(values) if values else None, max(values) if values else None,
                statistics.fmean(valid_values) if valid_values else None,
            ])
    return case_rows, sensor_rows


def save_workbook(path, args, cases, samples, started_at):
    header = sample_header()
    case_rows, sensor_rows = summarize(samples, header)
    estimated_seconds = len(cases) * args.repetitions * (
        args.settle_seconds + args.record_seconds
    )
    samples_per_case = max(1, round(args.record_seconds * args.sample_rate))
    metadata = [
        ["key", "value"],
        ["robot", "soft_3_working"],
        ["created_utc", datetime.now(timezone.utc).isoformat()],
        ["experiment_started_utc", started_at],
        ["sweep_mode", args.sweep_mode],
        ["torques_Nm", ", ".join(map(str, args.torques))],
        ["thetas_deg", ", ".join(map(str, args.thetas))],
        ["repetitions", args.repetitions],
        ["settle_seconds", args.settle_seconds],
        ["record_seconds", args.record_seconds],
        ["sample_rate_Hz", args.sample_rate],
        ["planned_cases", len(cases) * args.repetitions],
        ["planned_duration_seconds", estimated_seconds],
        ["planned_samples_per_case", samples_per_case],
        ["planned_total_samples", len(cases) * args.repetitions * samples_per_case],
        ["recorded_samples", len(samples)],
        ["sensor_count", len(SENSORS)],
        ["bridge_model", "350 ohm quarter bridge, GF=2.0, Vexc=2.5 V"],
        ["interpretation", "Pose-derived simulated voltage; not hardware calibrated"],
    ]
    sensor_map = [["sensor_id", "module", "station", "gauge", "angle_deg"]]
    for sensor in SENSORS:
        module, _, station_label, gauge, angle = sensor
        sensor_map.append([sensor_id(sensor), module, station_label, gauge, angle])
    return write_xlsx(path, {
        "Metadata": metadata,
        "Sensor_Map": sensor_map,
        "Cases": case_rows,
        "Samples": [header] + samples,
        "Sensor_Summary": sensor_rows,
    })


def main():
    args = parse_arguments()
    finite_values = (
        list(args.torques) + list(args.thetas)
        + [args.settle_seconds, args.record_seconds, args.sample_rate,
           args.sensor_timeout]
    )
    if not all(math.isfinite(value) for value in finite_values):
        raise ValueError("All numeric experiment settings must be finite")
    if args.repetitions < 1 or args.sample_rate <= 0.0:
        raise ValueError("repetitions and sample-rate must be positive")
    if args.settle_seconds < 0.0 or args.record_seconds <= 0.0:
        raise ValueError("settle-seconds must be nonnegative and record-seconds positive")
    if args.sensor_timeout <= 0.0:
        raise ValueError("sensor-timeout must be positive")
    if any(abs(torque) > 1.0 for torque in args.torques):
        raise ValueError("Torque commands must stay within +/-1.0 N*m")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path(
        f"research_data_working/soft_3_working_voltage_dataset_{timestamp}.xlsx"
    )
    if output.suffix.lower() != ".xlsx":
        output = output.with_suffix(".xlsx")

    cases = build_cases(args)
    total_cases = len(cases) * args.repetitions
    samples_per_case = max(1, round(args.record_seconds * args.sample_rate))
    planned_samples = total_cases * samples_per_case
    planned_seconds = total_cases * (args.settle_seconds + args.record_seconds)
    print(
        f"Cases: {total_cases}; samples/case: {samples_per_case}; "
        f"planned samples: {planned_samples}; "
        f"planned duration: {planned_seconds / 60.0:.1f} min"
    )
    print(f"Excel output: {output.resolve()}")

    rclpy.init()
    node = VoltageExperimentNode()
    started = time.monotonic()
    started_at = datetime.now(timezone.utc).isoformat()
    samples = []
    completed = 0
    try:
        spin_for(node, 0.5)
        competing = [
            name for name in node.get_node_names()
            if name == "soft_3_working_controller"
        ]
        if competing and not args.allow_competing_controller:
            raise RuntimeError(
                "Stop control_soft_3_working.py before collecting data; it would "
                "publish competing commands. Use --allow-competing-controller only "
                "for a deliberate experiment."
            )

        print("Waiting for all 18 voltage and validity channels...")
        deadline = time.monotonic() + args.sensor_timeout
        while rclpy.ok() and time.monotonic() < deadline and not node.sensors_ready():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.sensors_ready():
            missing = [sensor_id(sensor) for sensor in SENSORS if sensor not in node.voltages]
            raise RuntimeError("Sensor timeout; missing: " + ", ".join(missing))
        print("All 18 sensors ready.")

        case_id = 0
        for repetition in range(1, args.repetitions + 1):
            for case in cases:
                case_id += 1
                node.set_commands(case["commands"])
                print(
                    f"[{case_id}/{total_cases}] rep={repetition} "
                    f"{case['label']} commands={case['commands']}"
                )
                spin_for(node, args.settle_seconds)
                recording_started = time.monotonic()
                period = 1.0 / args.sample_rate
                for sample_number in range(samples_per_case):
                    sample_time = recording_started + sample_number * period
                    while rclpy.ok() and time.monotonic() < sample_time:
                        remaining = sample_time - time.monotonic()
                        rclpy.spin_once(node, timeout_sec=min(0.01, remaining))
                    if not rclpy.ok():
                        break
                    rclpy.spin_once(node, timeout_sec=0.0)
                    if node.sensors_ready():
                        samples.append(make_sample_row(
                            node, started, case_id, repetition, case
                        ))
                completed += 1
    except KeyboardInterrupt:
        print("Collection interrupted; saving the completed samples.")
    finally:
        node.set_commands([(0.0, 0.0)] * MODULE_COUNT)
        spin_for(node, 0.25)
        destination = save_workbook(output, args, cases, samples, started_at)
        print(
            f"Saved {len(samples)} samples from {completed}/{total_cases} completed "
            f"cases to {destination.resolve()}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
