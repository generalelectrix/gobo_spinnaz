"""Helpers for computing a resistor ladder for readout of binary pushwheels."""

def par(*resistor_values):
    """Return the resistance of a parallel network of resistors."""
    return 1.0 / sum(1.0 / r for r in resistor_values)

def ladder(r1, r2, r4, r8):
    """Return the switch resistance for each digit, given bias resistors.

    r1, r2, r4, and r8 are the resistance values for the 1, 2, 4, and 8 taps on
    the pushwheel switch.
    """
    return [
        float("inf"), # open circuit
        r1,
        r2,
        par(r1, r2),
        r4,
        par(r1, r4),
        par(r2, r4),
        par(r1, r2, r4),
        r8,
        par(r1, r8),
    ]

def sense_levels(r1, r2, r4, r8):
    """Return the sense voltage, normalized to unit bias resistance and voltage."""
    return [1.0 / (1.0 + r_b) for r_b in ladder(r1, r2, r4, r8)]


