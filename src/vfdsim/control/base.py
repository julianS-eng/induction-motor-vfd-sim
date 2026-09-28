"""Interfaces shared by all drive controllers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True, slots=True)
class Measurements:
    """Signals sampled by the controller at the start of a control period.

    Attributes
    ----------
    t:
        Sampling instant (s).
    i_a, i_b, i_c:
        Measured stator phase currents (A).
    omega_m:
        Measured mechanical speed (rad/s), e.g. from an encoder.
    theta_m:
        Measured mechanical angle (rad).
    v_dc:
        Measured DC-link voltage (V).
    """

    t: float
    i_a: float
    i_b: float
    i_c: float
    omega_m: float
    theta_m: float
    v_dc: float


class Controller(Protocol):
    """Discrete-time drive controller executed once per control period."""

    sample_time: float

    def reset(self) -> None:
        """Return all internal states to their initial values."""
        ...

    def step(self, m: Measurements) -> tuple[float, float]:
        """Compute the stator-voltage reference ``(v_α*, v_β*)`` in V."""
        ...

    def signals(self) -> dict[str, float]:
        """Internal signals of the last step, for logging (name → value)."""
        ...
