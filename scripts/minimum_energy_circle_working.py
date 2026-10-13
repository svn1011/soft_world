#!/usr/bin/env python3

"""Minimum-command-energy kinematics for the three-module soft snake."""

import math

import numpy as np
from scipy.optimize import minimize


MODULE_COUNT = 3
INTERVALS_PER_MODULE = 5
DISK_PITCH_M = 0.025
CONNECTOR_COUNT = MODULE_COUNT - 1
MAX_MODULE_BEND_RAD = math.pi
TOTAL_CENTERLINE_LENGTH_M = (
    MODULE_COUNT * INTERVALS_PER_MODULE + CONNECTOR_COUNT
) * DISK_PITCH_M
DEFAULT_CIRCLE_RADIUS_M = 0.5 * TOTAL_CENTERLINE_LENGTH_M


def rotation_matrix(rotation_vector):
    """Convert a three-component rotation vector to a rotation matrix."""
    vector = np.asarray(rotation_vector, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-12:
        return np.eye(3, dtype=np.float64)
    axis = vector / angle
    cross = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (
        np.eye(3)
        + math.sin(angle) * cross
        + (1.0 - math.cos(angle)) * (cross @ cross)
    )


def forward_tip_position(module_bends_rad, bend_direction_rad=math.pi):
    """Return the modeled tip position relative to the fixed base disc.

    Each module is a five-interval constant-curvature section. The two rigid
    inter-module connectors are included as one straight disk pitch each.
    """
    bends = np.asarray(module_bends_rad, dtype=np.float64)
    if bends.shape != (MODULE_COUNT,):
        raise ValueError(f"Expected {MODULE_COUNT} module bends")
    direction = float(bend_direction_rad)
    orientation = np.eye(3, dtype=np.float64)
    position = np.zeros(3, dtype=np.float64)
    half_step = np.array([0.0, 0.0, -0.5 * DISK_PITCH_M])
    connector_step = np.array([0.0, 0.0, -DISK_PITCH_M])
    for module_index, bend in enumerate(bends):
        interval_bend = bend / INTERVALS_PER_MODULE
        interval_rotation = np.array([
            -interval_bend * math.sin(direction),
            interval_bend * math.cos(direction),
            0.0,
        ])
        for _ in range(INTERVALS_PER_MODULE):
            position += orientation @ half_step
            orientation = orientation @ rotation_matrix(interval_rotation)
            position += orientation @ half_step
        if module_index < CONNECTOR_COUNT:
            position += orientation @ connector_step
    return position


def normalized_command_energy(module_bends_rad, weights=None):
    """Return 0.5*sum(weight*torque_command^2), with pi rad mapped to 1 N m."""
    bends = np.asarray(module_bends_rad, dtype=np.float64)
    energy_weights = (
        np.ones(MODULE_COUNT, dtype=np.float64)
        if weights is None else np.asarray(weights, dtype=np.float64)
    )
    if energy_weights.shape != (MODULE_COUNT,) or np.any(energy_weights <= 0.0):
        raise ValueError("Energy weights must contain three positive values")
    torques = bends / MAX_MODULE_BEND_RAD
    return 0.5 * float(np.dot(energy_weights, torques * torques))


def solve_minimum_energy_posture(radius_m, vertical_drop_m, weights=None):
    """Find the least command-energy posture for a radius and vertical drop."""
    radius = float(radius_m)
    drop = float(vertical_drop_m)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("circle radius must be positive and finite")
    if not math.isfinite(drop) or drop <= 0.0:
        raise ValueError("vertical drop must be positive and finite")
    energy_weights = (
        np.ones(MODULE_COUNT, dtype=np.float64)
        if weights is None else np.asarray(weights, dtype=np.float64)
    )
    if energy_weights.shape != (MODULE_COUNT,) or np.any(energy_weights <= 0.0):
        raise ValueError("Energy weights must contain three positive values")

    def constraints(bends):
        tip = forward_tip_position(bends)
        return np.array([tip[0] - radius, tip[2] + drop])

    constraint = {"type": "eq", "fun": constraints}
    starts = (
        (0.30, 0.30, 0.30),
        (0.80, 0.00, 0.00),
        (0.00, 0.80, 0.00),
        (0.00, 0.00, 0.80),
        (-0.50, 1.00, 1.00),
        (1.00, -0.50, 0.50),
    )
    candidates = []
    for start in starts:
        result = minimize(
            lambda bends: normalized_command_energy(bends, energy_weights),
            np.asarray(start, dtype=np.float64),
            method="SLSQP",
            bounds=[(-MAX_MODULE_BEND_RAD, MAX_MODULE_BEND_RAD)] * MODULE_COUNT,
            constraints=[constraint],
            options={"ftol": 1.0e-12, "maxiter": 600, "disp": False},
        )
        residual = constraints(result.x)
        if result.success and float(np.linalg.norm(residual)) <= 1.0e-6:
            candidates.append(result)
    if not candidates:
        raise ValueError(
            "No feasible minimum-energy posture was found. Choose a smaller "
            "radius or a reachable circle height."
        )
    best = min(candidates, key=lambda result: result.fun)
    bends = np.asarray(best.x, dtype=np.float64)
    tip = forward_tip_position(bends)
    return {
        "module_bends_rad": bends,
        "torques_Nm": bends / MAX_MODULE_BEND_RAD,
        "relative_tip_m": tip,
        "energy_proxy": normalized_command_energy(bends, energy_weights),
        "constraint_error_m": float(np.linalg.norm(constraints(bends))),
    }


def circle_target(base_position_m, radius_m, height_m, angle_rad):
    """Return a horizontal circular target centered on the base vertical axis."""
    base = np.asarray(base_position_m, dtype=np.float64)
    return np.array([
        base[0] + radius_m * math.cos(angle_rad),
        base[1] + radius_m * math.sin(angle_rad),
        height_m,
    ], dtype=np.float64)
