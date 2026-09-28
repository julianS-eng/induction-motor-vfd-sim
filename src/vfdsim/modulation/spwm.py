"""Sinusoidal (carrier-based) PWM.

Each phase reference is compared with a triangular carrier spanning
``[-v_dc/2, +v_dc/2]``. In the period-average sense the pole voltage referred
to the DC mid-point equals the reference, so::

    d_k = 1/2 + v_k* / v_dc,          k ∈ {a, b, c}

The linear range ends when the peak phase reference reaches ``v_dc / 2``
(modulation index ``m = 2 V̂ / v_dc = 1``); beyond that the reference is
clipped (over-modulation), which produces low-order harmonics.
"""

from __future__ import annotations

from vfdsim.modulation.base import Duties
from vfdsim.transforms import inverse_clarke


def _clip01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


class SinusoidalPWM:
    """Sine-triangle PWM without zero-sequence injection."""

    name = "spwm"

    def max_phase_peak(self, v_dc: float) -> float:
        """Linear-range limit ``v_dc / 2``."""
        return 0.5 * v_dc

    def duties(self, v_alpha: float, v_beta: float, v_dc: float) -> Duties:
        """Clipped duty cycles for the αβ reference."""
        if v_dc <= 0.0:
            return 0.5, 0.5, 0.5
        va, vb, vc = inverse_clarke(v_alpha, v_beta)
        inv = 1.0 / v_dc
        return _clip01(0.5 + va * inv), _clip01(0.5 + vb * inv), _clip01(0.5 + vc * inv)
