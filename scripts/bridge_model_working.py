"""Ideal quarter-bridge model used by the six-disc virtual hardware layer.

Values describe a nominated foil-gauge measurement chain. They are not a
calibration file for a purchased sensor or a claim of physical survivability.
"""

import math


GAUGE_NOMINAL_RESISTANCE_OHMS = 350.0
GAUGE_FACTOR_NOMINAL = 2.0
BRIDGE_EXCITATION_VOLTS = 2.5
FOIL_GAUGE_STRAIN_LIMIT = 0.03
DAQ_INPUT_LIMIT_VOLTS_PER_VOLT = 0.025


def quarter_bridge_output_voltage(strain):
    """Return ideal output for one active arm; sign follows bridge wiring.

    The active resistance is R(1 + GF * strain).  Nominal R cancels from the
    ratiometric bridge output, but remains published as hardware metadata.
    """
    delta = GAUGE_FACTOR_NOMINAL * float(strain)
    denominator = 4.0 + 2.0 * delta
    # R_active = R_nominal * (1 + delta) must remain positive.
    if delta <= -1.0 or denominator <= 0.0:
        return math.nan
    return -BRIDGE_EXCITATION_VOLTS * delta / denominator


def bridge_reading_is_within_nominal_range(strain, output_voltage):
    """Flag, rather than hide, readings outside chosen foil/DAQ assumptions."""
    return (
        math.isfinite(output_voltage)
        and abs(float(strain)) <= FOIL_GAUGE_STRAIN_LIMIT
        and abs(output_voltage / BRIDGE_EXCITATION_VOLTS)
        <= DAQ_INPUT_LIMIT_VOLTS_PER_VOLT
    )
