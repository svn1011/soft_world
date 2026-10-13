#!/usr/bin/env python3

"""Offline regressions for the minimum-energy circular-tip model."""

import math
import unittest

import numpy as np

from minimum_energy_circle_working import (
    DEFAULT_CIRCLE_RADIUS_M,
    TOTAL_CENTERLINE_LENGTH_M,
    circle_target,
    forward_tip_position,
    solve_minimum_energy_posture,
)


class MinimumEnergyCircleTest(unittest.TestCase):
    def test_default_radius_is_half_the_limb_length(self):
        self.assertAlmostEqual(
            DEFAULT_CIRCLE_RADIUS_M, 0.5 * TOTAL_CENTERLINE_LENGTH_M
        )
        self.assertAlmostEqual(TOTAL_CENTERLINE_LENGTH_M, 0.425)
        self.assertAlmostEqual(DEFAULT_CIRCLE_RADIUS_M, 0.2125)

    def test_default_posture_reaches_radius_and_height(self):
        solution = solve_minimum_energy_posture(DEFAULT_CIRCLE_RADIUS_M, 0.35)
        np.testing.assert_allclose(
            solution["relative_tip_m"],
            [DEFAULT_CIRCLE_RADIUS_M, 0.0, -0.35],
            atol=1.0e-6,
        )
        self.assertLessEqual(np.max(np.abs(solution["torques_Nm"])), 1.0)
        self.assertLess(solution["constraint_error_m"], 1.0e-6)

    def test_rotating_bending_plane_makes_flat_circle(self):
        solution = solve_minimum_energy_posture(DEFAULT_CIRCLE_RADIUS_M, 0.35)
        points = []
        for angle in np.linspace(0.0, 2.0 * math.pi, 13):
            points.append(forward_tip_position(
                solution["module_bends_rad"], angle + math.pi
            ))
        points = np.asarray(points)
        np.testing.assert_allclose(
            np.linalg.norm(points[:, :2], axis=1),
            DEFAULT_CIRCLE_RADIUS_M,
            atol=1.0e-6,
        )
        np.testing.assert_allclose(points[:, 2], -0.35, atol=1.0e-6)

    def test_world_targets_stay_horizontal(self):
        base = [0.0, 0.0, 0.75]
        points = np.asarray([
            circle_target(base, DEFAULT_CIRCLE_RADIUS_M, 0.40, angle)
            for angle in np.linspace(0.0, 2.0 * math.pi, 17)
        ])
        np.testing.assert_allclose(points[:, 2], 0.40, atol=1.0e-12)
        np.testing.assert_allclose(
            np.linalg.norm(points[:, :2], axis=1),
            DEFAULT_CIRCLE_RADIUS_M,
            atol=1.0e-12,
        )


if __name__ == "__main__":
    unittest.main()
