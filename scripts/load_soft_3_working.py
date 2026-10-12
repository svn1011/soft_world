#!/usr/bin/env python3

"""Load and control three serial six-disc modules as a hanging soft snake."""

import os

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": os.environ.get("SOFT_SNAKE_HEADLESS", "0") == "1",
})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.viewports import set_camera_view
from omni.physx import get_physx_interface
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray
from bridge_model_working import (
    BRIDGE_EXCITATION_VOLTS,
    DAQ_INPUT_LIMIT_VOLTS_PER_VOLT,
    FOIL_GAUGE_STRAIN_LIMIT,
    GAUGE_FACTOR_NOMINAL,
    GAUGE_NOMINAL_RESISTANCE_OHMS,
    bridge_reading_is_within_nominal_range,
    quarter_bridge_output_voltage,
)


MODULE_COUNT = 3
DISKS_PER_MODULE = 6
INTERVALS_PER_MODULE = DISKS_PER_MODULE - 1
DISK_COUNT = MODULE_COUNT * DISKS_PER_MODULE
ASSEMBLY_PATH = "/World/SoftSnake"
USD_NAME = "soft_3_working.usd"
DISK_PATHS = tuple(
    f"{ASSEMBLY_PATH}/Disk_{index:02d}" for index in range(DISK_COUNT)
)
DISK_PITCH = 0.025
DISK_RADIUS = 0.02
DISK_MASS = 0.01
GRAVITY_ACCELERATION = 9.81
PHYSICS_FREQUENCY_HZ = 240.0
BEND_STIFFNESS = 6400
BEND_DAMPING = 240
MAX_INTERVAL_BEND_DEGREES = 80.0
MAX_ABS_TORQUE = 1.0
MAX_TOTAL_MODULE_BEND_DEGREES = 180.0
MAX_TORQUE_SLEW_RATE = 2.0
MAX_INTERVAL_TARGET_SPEED_DEGREES_PER_SECOND = 30.0
MAX_THETA_SPEED_DEGREES_PER_SECOND = 90.0
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 8
BODY_LINEAR_DAMPING = 2.0
BODY_ANGULAR_DAMPING = 10.0
STRAIN_TO_MICROSTRAIN = 1_000_000.0
SENSOR_RING_DEFINITIONS = (
    ("one_third", 2, (0.0, 120.0, 240.0), Gf.Vec3f(1.0, 0.45, 0.0)),
    ("two_thirds", 3, (180.0, 300.0, 60.0), Gf.Vec3f(0.0, 0.8, 1.0)),
)


def module_start_disk(module_index):
    return module_index * DISKS_PER_MODULE


def module_joint_path(module_index, interval_index):
    return (
        f"{ASSEMBLY_PATH}/Joints/Module_{module_index + 1:02d}/"
        f"Interval_{interval_index + 1:02d}_D6Joint"
    )


def all_joint_paths():
    for module_index in range(MODULE_COUNT):
        for interval_index in range(INTERVALS_PER_MODULE):
            yield module_joint_path(module_index, interval_index)


def get_usd_path():
    script_directory = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(script_directory, "..", "usd", USD_NAME),
        os.path.join(
            script_directory, "..", "..", "..", "..", "..", "usd", USD_NAME
        ),
    )
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f"Could not find usd/{USD_NAME}")


def use_grey_studio_light_rig():
    """Select Isaac Sim's Grey Studio viewport light rig for repeatable views."""
    try:
        import omni.kit.actions.core

        registry = omni.kit.actions.core.get_action_registry()
        action = registry.get_action(
            "omni.kit.viewport.menubar.lighting", "set_lighting_mode_rig"
        )
        if action is None:
            raise RuntimeError("Grey Studio lighting action is unavailable")
        result = action.execute("Grey Studio")
        if isinstance(result, tuple) and result and not result[0]:
            raise RuntimeError(f"Grey Studio action failed: {result}")
        print("Viewport lighting : Grey Studio")
    except Exception as error:
        print(f"WARNING: could not select Grey Studio lighting: {error}")


def validate_stage(stage):
    missing = [path for path in DISK_PATHS if not stage.GetPrimAtPath(path).IsValid()]
    missing.extend(
        path for path in all_joint_paths()
        if not stage.GetPrimAtPath(path).IsValid()
    )
    for connector in (1, 2):
        path = f"{ASSEMBLY_PATH}/Joints/Connector_{connector:02d}_FixedJoint"
        if not stage.GetPrimAtPath(path).IsValid():
            missing.append(path)
    if missing:
        raise RuntimeError("Invalid three-module snake USD; missing: " + ", ".join(missing))


def configure_stage(stage):
    for joint_path in all_joint_paths():
        prim = stage.GetPrimAtPath(joint_path)
        PhysxSchema.PhysxJointAPI.Apply(
            prim
        ).CreateEnableProjectionAttr().Set(False)
        for axis in ("transX", "transY", "transZ", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(prim, axis)
            limit.CreateLowAttr().Set(1.0)
            limit.CreateHighAttr().Set(-1.0)
        for axis in ("rotX", "rotY"):
            limit = UsdPhysics.LimitAPI.Apply(prim, axis)
            limit.CreateLowAttr().Set(-MAX_INTERVAL_BEND_DEGREES)
            limit.CreateHighAttr().Set(MAX_INTERVAL_BEND_DEGREES)
            drive = UsdPhysics.DriveAPI.Apply(prim, axis)
            drive.CreateTypeAttr().Set("acceleration")
            drive.CreateTargetPositionAttr().Set(0.0)
            drive.CreateStiffnessAttr().Set(BEND_STIFFNESS)
            drive.CreateDampingAttr().Set(BEND_DAMPING)

    for disk_path in DISK_PATHS:
        prim = stage.GetPrimAtPath(disk_path)
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        body.CreateSolverPositionIterationCountAttr().Set(
            SOLVER_POSITION_ITERATIONS
        )
        body.CreateSolverVelocityIterationCountAttr().Set(
            SOLVER_VELOCITY_ITERATIONS
        )
        body.CreateSleepThresholdAttr().Set(0.0)
        body.CreateLinearDampingAttr().Set(BODY_LINEAR_DAMPING)
        body.CreateAngularDampingAttr().Set(BODY_ANGULAR_DAMPING)
        xform = UsdGeom.Xformable(prim)
        ordered = xform.GetOrderedXformOps()
        if not any(op.GetOpType() == UsdGeom.XformOp.TypeOrient for op in ordered):
            orient = prim.GetAttribute("xformOp:orient")
            if orient.IsValid():
                xform.SetXformOpOrder([*ordered, UsdGeom.XformOp(orient)])


def add_sensor_visuals(stage):
    for module_index in range(MODULE_COUNT):
        start = module_start_disk(module_index)
        for station_name, local_disk, angles, color in SENSOR_RING_DEFINITIONS:
            disk_index = start + local_disk
            for gauge_index, angle_degrees in enumerate(angles, start=1):
                angle = np.deg2rad(angle_degrees)
                path = (
                    f"{DISK_PATHS[disk_index]}/Sensors/{station_name}/"
                    f"Gauge_{gauge_index}"
                )
                gauge = UsdGeom.Cube.Define(stage, path)
                gauge.CreateSizeAttr().Set(1.0)
                gauge.CreateDisplayColorAttr().Set([color])
                xform = UsdGeom.Xformable(gauge.GetPrim())
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(
                    DISK_RADIUS * np.cos(angle),
                    DISK_RADIUS * np.sin(angle),
                    0.0,
                ))
                xform.AddRotateZOp().Set(angle_degrees)
                xform.AddScaleOp().Set(Gf.Vec3f(0.0005, 0.002, 0.0025))


def quaternion_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ], dtype=np.float64)


def relative_quaternion(reference, measured):
    reference = np.asarray(reference, dtype=np.float64)
    measured = np.asarray(measured, dtype=np.float64)
    reference /= np.linalg.norm(reference)
    measured /= np.linalg.norm(measured)
    relative = quaternion_multiply(
        reference * np.array([1.0, -1.0, -1.0, -1.0]), measured
    )
    relative /= np.linalg.norm(relative)
    return -relative if relative[0] < 0.0 else relative


def quaternion_rotation_vector(quaternion):
    vector = np.asarray(quaternion[1:], dtype=np.float64)
    norm = np.linalg.norm(vector)
    if norm < 1.0e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(norm, quaternion[0])
    return angle * vector / norm


def accumulated_bend_vector(orientations):
    total = np.zeros(2, dtype=np.float64)
    for previous, current in zip(orientations[:-1], orientations[1:]):
        total += quaternion_rotation_vector(
            relative_quaternion(previous, current)
        )[:2]
    return total


class Soft3WorkingCommandSubscriber(Node):
    def __init__(self, stage):
        super().__init__("soft_3_working_subscriber")
        self.torque_commands = np.zeros(MODULE_COUNT, dtype=np.float64)
        self.applied_torques = np.zeros(MODULE_COUNT, dtype=np.float64)
        self.theta_commands = np.zeros(MODULE_COUNT, dtype=np.float64)
        self.applied_thetas = np.zeros(MODULE_COUNT, dtype=np.float64)
        self.applied_interval_bends = np.zeros(MODULE_COUNT, dtype=np.float64)
        self.drive_targets = []
        self.sensor_publishers = {}
        self.module_measurement_publishers = []

        for module_index in range(MODULE_COUNT):
            module_number = module_index + 1
            root = f"/soft_snake/module_{module_number}"
            self.create_subscription(
                Float64, f"{root}/phi_torque",
                lambda message, index=module_index: self._set_torque(index, message),
                10,
            )
            self.create_subscription(
                Float64, f"{root}/theta_position",
                lambda message, index=module_index: self._set_theta(index, message),
                10,
            )
            targets = []
            for interval_index in range(INTERVALS_PER_MODULE):
                prim = stage.GetPrimAtPath(
                    module_joint_path(module_index, interval_index)
                )
                targets.append((
                    UsdPhysics.DriveAPI(prim, "rotX").GetTargetPositionAttr(),
                    UsdPhysics.DriveAPI(prim, "rotY").GetTargetPositionAttr(),
                ))
            self.drive_targets.append(targets)
            self.module_measurement_publishers.append({
                "phi": self.create_publisher(Float64, f"{root}/measured_phi", 10),
                "theta": self.create_publisher(Float64, f"{root}/measured_theta", 10),
                "applied_torque": self.create_publisher(
                    Float64, f"{root}/applied_phi_torque", 10
                ),
                "applied_theta": self.create_publisher(
                    Float64, f"{root}/applied_theta", 10
                ),
            })
            for station_name, _, angles, _ in SENSOR_RING_DEFINITIONS:
                sensor_root = f"{root}/sensors/{station_name}"
                key = (module_index, station_name)
                self.sensor_publishers[key] = {
                    "orientation": self.create_publisher(
                        Float64MultiArray, f"{sensor_root}/orientation", 10
                    ),
                    "strain": self.create_publisher(
                        Float64MultiArray, f"{sensor_root}/strain", 10
                    ),
                    "bridge_voltage": self.create_publisher(
                        Float64MultiArray, f"{sensor_root}/bridge_voltage", 10
                    ),
                    "bridge_valid": self.create_publisher(
                        Float64MultiArray, f"{sensor_root}/bridge_valid", 10
                    ),
                    "gauges": [
                        self.create_publisher(
                            Float64, f"{sensor_root}/gauge_{index}/strain", 10
                        ) for index in range(1, len(angles) + 1)
                    ],
                    "voltages": [
                        self.create_publisher(
                            Float64,
                            f"{sensor_root}/gauge_{index}/bridge_voltage", 10,
                        ) for index in range(1, len(angles) + 1)
                    ],
                    "validity": [
                        self.create_publisher(
                            Bool, f"{sensor_root}/gauge_{index}/bridge_valid", 10
                        ) for index in range(1, len(angles) + 1)
                    ],
                }

    def _set_torque(self, module_index, message):
        self.torque_commands[module_index] = np.clip(
            message.data, -MAX_ABS_TORQUE, MAX_ABS_TORQUE
        )

    def _set_theta(self, module_index, message):
        self.theta_commands[module_index] = (
            message.data + 180.0
        ) % 360.0 - 180.0

    def update_drive_targets(self, time_step):
        torque_step = MAX_TORQUE_SLEW_RATE * time_step
        self.applied_torques += np.clip(
            self.torque_commands - self.applied_torques,
            -torque_step, torque_step,
        )
        bend_step = MAX_INTERVAL_TARGET_SPEED_DEGREES_PER_SECOND * time_step
        theta_step = MAX_THETA_SPEED_DEGREES_PER_SECOND * time_step
        for module_index in range(MODULE_COUNT):
            target_interval_bend = (
                self.applied_torques[module_index] / MAX_ABS_TORQUE
                * MAX_TOTAL_MODULE_BEND_DEGREES / INTERVALS_PER_MODULE
            )
            self.applied_interval_bends[module_index] += np.clip(
                target_interval_bend - self.applied_interval_bends[module_index],
                -bend_step, bend_step,
            )
            theta_delta = (
                self.theta_commands[module_index]
                - self.applied_thetas[module_index] + 180.0
            ) % 360.0 - 180.0
            self.applied_thetas[module_index] += np.clip(
                theta_delta, -theta_step, theta_step
            )
            direction = np.deg2rad(self.applied_thetas[module_index])
            target_x = float(
                -self.applied_interval_bends[module_index] * np.sin(direction)
            )
            target_y = float(
                self.applied_interval_bends[module_index] * np.cos(direction)
            )
            for x_target, y_target in self.drive_targets[module_index]:
                x_target.Set(target_x)
                y_target.Set(target_y)

    def publish_measurements(self, orientations):
        orientations = np.asarray(orientations, dtype=np.float64)
        for module_index in range(MODULE_COUNT):
            start = module_start_disk(module_index)
            module_orientations = orientations[start:start + DISKS_PER_MODULE]
            bend_x, bend_y = accumulated_bend_vector(module_orientations)
            phi = np.rad2deg(np.hypot(bend_x, bend_y))
            theta = np.rad2deg(np.arctan2(-bend_x, bend_y))
            publishers = self.module_measurement_publishers[module_index]
            publishers["phi"].publish(Float64(data=float(phi)))
            publishers["theta"].publish(Float64(data=float(theta)))
            publishers["applied_torque"].publish(Float64(
                data=float(self.applied_torques[module_index])
            ))
            publishers["applied_theta"].publish(Float64(
                data=float(self.applied_thetas[module_index])
            ))

            for station_name, local_disk, angles, _ in SENSOR_RING_DEFINITIONS:
                disk_index = start + local_disk
                key = (module_index, station_name)
                station_publishers = self.sensor_publishers[key]
                station_orientation = relative_quaternion(
                    orientations[start], orientations[disk_index]
                )
                local_orientation = relative_quaternion(
                    orientations[disk_index - 1],
                    orientations[disk_index + 1],
                )
                local_rotation = quaternion_rotation_vector(local_orientation)
                curvature_x = local_rotation[0] / (2.0 * DISK_PITCH)
                curvature_y = local_rotation[1] / (2.0 * DISK_PITCH)
                strains = []
                voltages = []
                validity = []
                for angle_degrees in angles:
                    angle = np.deg2rad(angle_degrees)
                    strain = DISK_RADIUS * (
                        curvature_y * np.cos(angle)
                        - curvature_x * np.sin(angle)
                    )
                    voltage = quarter_bridge_output_voltage(strain)
                    strains.append(strain * STRAIN_TO_MICROSTRAIN)
                    voltages.append(voltage)
                    validity.append(float(
                        bridge_reading_is_within_nominal_range(strain, voltage)
                    ))
                orientation_message = Float64MultiArray()
                orientation_message.data = station_orientation.tolist()
                station_publishers["orientation"].publish(orientation_message)
                strain_message = Float64MultiArray()
                strain_message.data = strains
                station_publishers["strain"].publish(strain_message)
                voltage_message = Float64MultiArray()
                voltage_message.data = voltages
                station_publishers["bridge_voltage"].publish(voltage_message)
                valid_message = Float64MultiArray()
                valid_message.data = validity
                station_publishers["bridge_valid"].publish(valid_message)
                for strain_pub, voltage_pub, valid_pub, strain, voltage, valid in zip(
                    station_publishers["gauges"],
                    station_publishers["voltages"],
                    station_publishers["validity"],
                    strains, voltages, validity,
                ):
                    strain_pub.publish(Float64(data=float(strain)))
                    voltage_pub.publish(Float64(data=float(voltage)))
                    valid_pub.publish(Bool(data=bool(valid)))


def main():
    path = get_usd_path()
    if not open_stage(path):
        raise RuntimeError(f"Could not open {path}")
    from omni.usd import get_context
    stage = get_context().get_stage()
    validate_stage(stage)
    configure_stage(stage)
    add_sensor_visuals(stage)
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_FREQUENCY_HZ,
        rendering_dt=1.0 / 60.0,
    )
    world.get_physics_context().set_physx_update_transformations_settings(
        update_to_usd=True,
        update_velocities_to_usd=True,
        output_velocities_local_space=False,
    )
    disks = world.scene.add(RigidPrim(
        prim_paths_expr=list(DISK_PATHS),
        name="soft_3_working_disks",
        reset_xform_properties=False,
    ))
    world.reset()
    disks.set_sleep_thresholds(np.zeros(DISK_COUNT, dtype=np.float32))
    gravity_compensation = np.zeros((DISK_COUNT, 3), dtype=np.float32)
    gravity_compensation[:, 2] = DISK_MASS * GRAVITY_ACCELERATION
    zero_torques = np.zeros((DISK_COUNT, 3), dtype=np.float32)
    world.play()

    started_rclpy = not rclpy.ok()
    if started_rclpy:
        rclpy.init()
    node = Soft3WorkingCommandSubscriber(stage)
    physx_interface = get_physx_interface()
    set_camera_view(
        eye=np.array([0.48, 0.48, 0.75]),
        target=np.array([0.0, 0.0, 0.75]),
        camera_prim_path="/OmniverseKit_Persp",
    )
    use_grey_studio_light_rig()
    print("--------------------------------")
    print("THREE-MODULE HANGING SOFT SNAKE LOADED")
    print(f"USD               : {path}")
    print("Modules           : 3 x six discs, five actuated intervals each")
    print("Connectors        : two fixed inter-module joints")
    print("Commands          : three phi torques + three theta directions")
    print("Per-module limit  : +/-1.0 N*m -> +/-180 degrees")
    print("Sensors           : 18 pose-derived gauges (six per module)")
    print(
        f"Bridge            : {GAUGE_NOMINAL_RESISTANCE_OHMS:.0f} ohm, "
        f"GF={GAUGE_FACTOR_NOMINAL}, Vexc={BRIDGE_EXCITATION_VOLTS} V"
    )
    print(
        f"Bridge validity   : strain <= {FOIL_GAUGE_STRAIN_LIMIT:.0%}, "
        f"input <= {DAQ_INPUT_LIMIT_VOLTS_PER_VOLT * 1000:.1f} mV/V"
    )
    print("--------------------------------")

    while simulation_app.is_running() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        if world.is_playing():
            node.update_drive_targets(1.0 / PHYSICS_FREQUENCY_HZ)
            disks.apply_forces_and_torques_at_pos(
                forces=gravity_compensation,
                torques=zero_torques,
                is_global=True,
            )
            world.step(render=False)
            physx_interface.update_transformations(False, True, True)
        world.render()
        if not rclpy.ok():
            break
        _, orientations = disks.get_world_poses()
        node.publish_measurements(orientations)

    node.destroy_node()
    if started_rclpy and rclpy.ok():
        rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
