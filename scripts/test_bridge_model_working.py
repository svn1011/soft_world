#!/usr/bin/env python3
"""Offline numerical checks for the working virtual quarter-bridge model."""

import math
import unittest

from bridge_model_working import (
    BRIDGE_EXCITATION_VOLTS,
    bridge_reading_is_within_nominal_range,
    quarter_bridge_output_voltage,
)


class QuarterBridgeTests(unittest.TestCase):
    def test_zero_strain_is_balanced(self):
        self.assertEqual(quarter_bridge_output_voltage(0.0), 0.0)

    def test_1000_microstrain_matches_quarter_bridge_equation(self):
        output = quarter_bridge_output_voltage(0.001)
        expected = -BRIDGE_EXCITATION_VOLTS * 0.002 / 4.004
        self.assertAlmostEqual(output, expected)
        self.assertTrue(bridge_reading_is_within_nominal_range(0.001, output))

    def test_large_virtual_strain_is_flagged(self):
        output = quarter_bridge_output_voltage(0.25)
        self.assertTrue(math.isfinite(output))
        self.assertFalse(bridge_reading_is_within_nominal_range(0.25, output))

    def test_nonphysical_compression_is_nan(self):
        self.assertTrue(math.isnan(quarter_bridge_output_voltage(-0.5)))


if __name__ == "__main__":
    unittest.main()
