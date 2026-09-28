"""Ideal voltage sources."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GridSource:
    """Balanced, stiff three-phase sinusoidal grid (positive sequence).

    Parameters
    ----------
    line_voltage_rms:
        Line-to-line RMS voltage in V.
    frequency_hz:
        Grid frequency in Hz.
    phase_rad:
        Phase of the ``a``-phase voltage at ``t = 0``.
    """

    line_voltage_rms: float
    frequency_hz: float
    phase_rad: float = 0.0

    @property
    def phase_peak(self) -> float:
        """Peak phase-to-neutral voltage (= αβ space-vector magnitude)."""
        return self.line_voltage_rms * math.sqrt(2.0 / 3.0)

    def alpha_beta(self, t: float) -> tuple[float, float]:
        """Stator αβ voltage at time ``t``."""
        theta = 2.0 * math.pi * self.frequency_hz * t + self.phase_rad
        v = self.phase_peak
        return v * math.cos(theta), v * math.sin(theta)
