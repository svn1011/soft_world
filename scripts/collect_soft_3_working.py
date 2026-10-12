#!/usr/bin/env python3

"""Run repeatable soft_3_working voltage sweeps and save an XLSX dataset."""

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
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


def display_sensor_id(sensor):
    """Return the human-readable identifier used by Final Data.xlsx."""
    return sensor_id(sensor).replace("_", " ")


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
    parser.add_argument(
        "--samples-per-case", type=int, default=1,
        help="Target-gated snapshots per case (default: exactly one).",
    )
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument("--sensor-timeout", type=float, default=60.0)
    parser.add_argument(
        "--torque-tolerance-nm", type=float, default=0.005,
        help="Maximum applied-versus-commanded torque error before recording.",
    )
    parser.add_argument(
        "--theta-tolerance-deg", type=float, default=2.0,
        help="Maximum circular bending-direction error before recording.",
    )
    parser.add_argument(
        "--target-hold-seconds", type=float, default=0.5,
        help="Time that all commanded modules must remain within tolerance.",
    )
    parser.add_argument(
        "--target-timeout", type=float, default=15.0,
        help="Maximum wait for each case to reach and hold its target.",
    )
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
        self.applied = {
            module: {"torque": None, "theta": None}
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
            self.create_subscription(
                Float64, f"{root}/applied_phi_torque",
                lambda message, number=module: self._applied(number, "torque", message),
                10,
            )
            self.create_subscription(
                Float64, f"{root}/applied_theta",
                lambda message, number=module: self._applied(number, "theta", message),
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

    def _applied(self, module, field, message):
        self.applied[module][field] = float(message.data)

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


def wrap_degrees(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def target_state(node, commands, torque_tolerance, theta_tolerance):
    """Return whether the simulator has applied every commanded torque and theta."""
    modules = []
    reached = True
    for module, (torque, theta_command) in enumerate(commands, start=1):
        applied_torque = node.applied[module]["torque"]
        applied_theta = node.applied[module]["theta"]
        torque_error = (
            abs(applied_torque - torque)
            if applied_torque is not None and math.isfinite(applied_torque)
            else math.inf
        )
        theta_error = (
            abs(wrap_degrees(applied_theta - theta_command))
            if applied_theta is not None and math.isfinite(applied_theta)
            else math.inf
        )
        module_reached = (
            torque_error <= torque_tolerance and theta_error <= theta_tolerance
        )
        reached = reached and module_reached
        modules.append({
            "module": module,
            "torque_command": float(torque),
            "theta_command": float(theta_command),
            "applied_torque": applied_torque,
            "applied_theta": applied_theta,
            "torque_error": torque_error,
            "theta_error": theta_error,
            "reached": module_reached,
        })
    return reached, modules


def wait_for_target(node, commands, args, timeout=None):
    """Wait for all modules to remain at the commanded target for the hold time."""
    started = time.monotonic()
    deadline = started + (args.target_timeout if timeout is None else timeout)
    held_since = None
    state = []
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        reached, state = target_state(
            node, commands, args.torque_tolerance_nm, args.theta_tolerance_deg
        )
        now = time.monotonic()
        if reached:
            if held_since is None:
                held_since = now
            if now - held_since >= args.target_hold_seconds:
                return True, now - started, state
        else:
            held_since = None
    target_state_value = target_state(
        node, commands, args.torque_tolerance_nm, args.theta_tolerance_deg
    )[1]
    return False, time.monotonic() - started, target_state_value


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
            f"module_{module}_applied_torque_Nm",
            f"module_{module}_applied_theta_deg",
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
            torque, theta, node.applied[module]["torque"],
            node.applied[module]["theta"], node.measured[module]["phi"],
            node.measured[module]["theta"],
        ])
    row.extend(node.voltages.get(sensor) for sensor in SENSORS)
    row.extend(node.validity.get(sensor) for sensor in SENSORS)
    return row


def make_sensor_data_rows(samples, header):
    """Expand samples into the exact Final Sensor Data long-table layout."""
    index = {name: position for position, name in enumerate(header)}
    rows = [[
        "case number", "module number", "section number", "sensor number",
        "torque command Nm", "theta command deg", "voltage V",
        "applied torque Nm", "applied theta deg", "commanded phi deg",
        "phi angle deg", "sensor angle deg", "sensor id",
    ]]
    for sample in samples:
        for sensor in SENSORS:
            module, _, section_number, gauge, sensor_angle = sensor
            torque = sample[index[f"module_{module}_torque_Nm"]]
            measured_phi = sample[index[f"module_{module}_measured_phi_deg"]]
            theta_command = sample[index[f"module_{module}_theta_deg"]]
            applied_torque = sample[index[f"module_{module}_applied_torque_Nm"]]
            applied_theta = sample[index[f"module_{module}_applied_theta_deg"]]
            rows.append([
                sample[index["case_id"]], module, section_number, f"G{gauge}",
                torque, theta_command,
                sample[index[f"{sensor_id(sensor)}_V"]],
                applied_torque, applied_theta,
                max(-180.0, min(180.0, 180.0 * torque)), measured_phi,
                sensor_angle, display_sensor_id(sensor),
            ])
    return rows


def make_final_sample_rows(samples, header):
    """Select and rename fields to match the Final Data.xlsx Samples sheet."""
    index = {name: position for position, name in enumerate(header)}
    output_header = ["case id", "case label"]
    for module in range(1, MODULE_COUNT + 1):
        output_header.extend([
            f"module {module} torque Nm", f"module {module} theta deg",
            f"module {module} applied torque Nm",
            f"module {module} applied theta deg",
            f"module {module} measured phi deg",
            f"module {module} measured theta deg",
        ])
    output_header.extend(f"{display_sensor_id(sensor)} V" for sensor in SENSORS)

    rows = [output_header]
    for sample in samples:
        row = [
            sample[index["case_id"]],
            str(sample[index["case_label"]]).replace("_", " "),
        ]
        for module in range(1, MODULE_COUNT + 1):
            row.extend([
                sample[index[f"module_{module}_torque_Nm"]],
                sample[index[f"module_{module}_theta_deg"]],
                sample[index[f"module_{module}_applied_torque_Nm"]],
                sample[index[f"module_{module}_applied_theta_deg"]],
                sample[index[f"module_{module}_measured_phi_deg"]],
                sample[index[f"module_{module}_measured_theta_deg"]],
            ])
        row.extend(
            sample[index[f"{sensor_id(sensor)}_V"]] for sensor in SENSORS
        )
        rows.append(row)
    return rows


def save_workbook(path, args, cases, samples, started_at, case_results=None):
    header = sample_header()
    sensor_data_rows = make_sensor_data_rows(samples, header)
    final_sample_rows = make_final_sample_rows(samples, header)
    case_results = case_results or []
    successful_cases = sum(result["status"] == "recorded" for result in case_results)
    scenario_information = [
        ["key", "value"],
        ["sweep mode", args.sweep_mode.replace("_", " ")],
        ["torques Nm", ", ".join(map(str, args.torques))],
        ["thetas deg", ", ".join(map(str, args.thetas))],
        ["sample rate Hz", args.sample_rate],
        ["torque tolerance Nm", args.torque_tolerance_nm],
        ["theta tolerance deg", args.theta_tolerance_deg],
        ["target hold seconds", args.target_hold_seconds],
        ["planned cases", len(cases) * args.repetitions],
        ["planned total samples", len(cases) * args.repetitions * args.samples_per_case],
        ["successful cases", successful_cases],
        ["failed cases", len(case_results) - successful_cases],
        ["recorded samples", len(samples)],
        ["sensor data rows", max(0, len(sensor_data_rows) - 1)],
        ["sensor count", len(SENSORS)],
        ["bridge model", "350 ohm quarter bridge, GF=2.0, Vexc=2.5 V"],
        ["interpretation", "Pose-derived simulated voltage; not hardware calibrated"],
    ]
    sensor_map = [["sensor id", "module", "station", "gauge", "angle deg"]]
    for sensor in SENSORS:
        module, _, station_label, gauge, angle = sensor
        sensor_map.append([display_sensor_id(sensor), module, station_label, gauge, angle])
    return write_xlsx(path, {
        "Scenario Information": scenario_information,
        "Sensor Map": sensor_map,
        "Samples": final_sample_rows,
        "Final Sensor Data": sensor_data_rows,
    })


def main():
    args = parse_arguments()
    finite_values = (
        list(args.torques) + list(args.thetas)
        + [args.settle_seconds, args.sample_rate, args.sensor_timeout,
           args.torque_tolerance_nm, args.theta_tolerance_deg,
           args.target_hold_seconds, args.target_timeout]
    )
    if not all(math.isfinite(value) for value in finite_values):
        raise ValueError("All numeric experiment settings must be finite")
    if args.repetitions < 1 or args.samples_per_case < 1 or args.sample_rate <= 0.0:
        raise ValueError("repetitions, samples-per-case, and sample-rate must be positive")
    if args.settle_seconds < 0.0 or args.target_hold_seconds < 0.0:
        raise ValueError("settle-seconds and target-hold-seconds must be nonnegative")
    if args.torque_tolerance_nm < 0.0 or args.theta_tolerance_deg < 0.0:
        raise ValueError("command tolerances must be nonnegative")
    if args.target_timeout <= 0.0:
        raise ValueError("target-timeout must be positive")
    if args.sensor_timeout <= 0.0:
        raise ValueError("sensor-timeout must be positive")
    if any(abs(torque) > 1.0 for torque in args.torques):
        raise ValueError("Torque commands must stay within +/-1.0 N*m")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or Path(
        f"research_data_working/Final_Data_collection_working_{timestamp}.xlsx"
    )
    if output.suffix.lower() != ".xlsx":
        output = output.with_suffix(".xlsx")
    if output.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing workbook: {output}. "
            "Choose a new --output path so existing research data is preserved."
        )

    cases = build_cases(args)
    total_cases = len(cases) * args.repetitions
    planned_samples = total_cases * args.samples_per_case
    nominal_seconds = total_cases * (
        args.settle_seconds + args.target_hold_seconds
        + max(0, args.samples_per_case - 1) / args.sample_rate
    )
    print(
        f"Cases: {total_cases}; samples/case: {args.samples_per_case}; "
        f"planned samples: {planned_samples}; "
        f"nominal minimum duration: {nominal_seconds / 60.0:.1f} min"
    )
    print(
        "Recording gate: applied torque within "
        f"{args.torque_tolerance_nm:g} N*m and applied theta within "
        f"{args.theta_tolerance_deg:g} deg for "
        f"{args.target_hold_seconds:g} s"
    )
    print(f"Excel output: {output.resolve()}")

    rclpy.init()
    node = VoltageExperimentNode()
    started = time.monotonic()
    started_at = datetime.now(timezone.utc).isoformat()
    samples = []
    completed = 0
    successful = 0
    case_results = []
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
                reached, target_wait, state = wait_for_target(
                    node, case["commands"], args
                )
                recorded_for_case = 0
                status = "target_timeout"
                if reached:
                    for sample_number in range(args.samples_per_case):
                        if sample_number:
                            spin_for(node, 1.0 / args.sample_rate)
                        current_reached, state = target_state(
                            node, case["commands"], args.torque_tolerance_nm,
                            args.theta_tolerance_deg,
                        )
                        if not current_reached:
                            reacquired, extra_wait, state = wait_for_target(
                                node, case["commands"], args
                            )
                            target_wait += extra_wait
                            if not reacquired:
                                status = "target_lost"
                                break
                        if not node.sensors_ready():
                            status = "sensor_stale"
                            break
                        samples.append(make_sample_row(
                            node, started, case_id, repetition, case
                        ))
                        recorded_for_case += 1
                    else:
                        status = "recorded"
                        successful += 1
                print(
                    f"  -> {status}; target wait={target_wait:.3f} s; "
                    f"samples={recorded_for_case}"
                )
                case_results.append({
                    "case_id": case_id,
                    "repetition": repetition,
                    "label": case["label"],
                    "status": status,
                    "target_reached": reached,
                    "target_wait_s": target_wait,
                    "recorded_samples": recorded_for_case,
                    "state": state,
                })
                completed += 1
    except KeyboardInterrupt:
        print("Collection interrupted; saving the completed samples.")
    finally:
        node.set_commands([(0.0, 0.0)] * MODULE_COUNT)
        spin_for(node, 0.25)
        destination = save_workbook(
            output, args, cases, samples, started_at, case_results
        )
        print(
            f"Saved {len(samples)} samples from {successful}/{total_cases} successful "
            f"cases to {destination.resolve()}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
