"""Common interface of the pulse-width modulators."""

from __future__ import annotations

from typing import Protocol

Duties = tuple[float, float, float]


class Modulator(Protocol):
    """Converts a stator-voltage reference into three pole duty cycles.

    A duty cycle ``d_k ∈ [0, 1]`` is the fraction of the switching period
    during which the upper switch of leg ``k`` conducts, so the period-average
    pole voltage (to the negative DC rail) is ``d_k v_dc``.
    """

    name: str

    def max_phase_peak(self, v_dc: float) -> float:
        """Largest fundamental phase-voltage peak available in the linear range."""
        ...

    def duties(self, v_alpha: float, v_beta: float, v_dc: float) -> Duties:
        """Duty cycles for an αβ voltage reference (V) and DC-link voltage ``v_dc``."""
        ...
