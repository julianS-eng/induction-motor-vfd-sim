"""Two-level voltage-source inverter models.

The engine integrates the plant over *segments* of constant pole state inside
each switching period. A pole state ``s_k ∈ [0, 1]`` multiplies the DC-link
voltage to give the pole voltage referred to the negative rail,
``v_kN = s_k v_dc``. With the motor neutral isolated, the αβ stator voltage is
the Clarke transform of the pole voltages (the common-mode component is
rejected by the transform).

* :class:`AveragedInverter` returns a single segment per period whose pole
  states are the duty cycles themselves (switching-period average model,
  ideal switches, no dead time). It is the standard model for control design.
* The switched model with dead time is delivered in Phase 2 and implements the
  same :class:`PowerStage` protocol.
"""

from __future__ import annotations

from typing import Protocol

from vfdsim.modulation.base import Duties

#: A segment of constant pole state: ``(duration_s, (s_a, s_b, s_c))``.
Segment = tuple[float, Duties]


class PowerStage(Protocol):
    """Protocol of an inverter model used by the simulation engine."""

    name: str

    def segments(
        self, duties: Duties, period: float, currents_abc: tuple[float, float, float]
    ) -> list[Segment]:
        """Split one switching period into segments of constant pole state."""
        ...


class AveragedInverter:
    """Switching-period-averaged ideal inverter."""

    name = "averaged"

    def segments(
        self, duties: Duties, period: float, currents_abc: tuple[float, float, float]
    ) -> list[Segment]:
        """One segment with the pole states equal to the duty cycles."""
        return [(period, duties)]


def pole_to_alpha_beta(poles: Duties, v_dc: float) -> tuple[float, float]:
    """Stator αβ voltage produced by pole states ``poles`` on a bus ``v_dc``."""
    va, vb, vc = poles[0] * v_dc, poles[1] * v_dc, poles[2] * v_dc
    return (2.0 * va - vb - vc) / 3.0, (vb - vc) * 0.5773502691896258


def dc_link_current(poles: Duties, currents_abc: tuple[float, float, float]) -> float:
    """Current drawn from the DC link by the inverter, ``Σ s_k i_k``."""
    return poles[0] * currents_abc[0] + poles[1] * currents_abc[1] + poles[2] * currents_abc[2]
