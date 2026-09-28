"""Pulse-width modulation strategies."""

from vfdsim.modulation.base import Duties, Modulator
from vfdsim.modulation.spwm import SinusoidalPWM

__all__ = ["Duties", "Modulator", "SinusoidalPWM", "make_modulator"]


def make_modulator(method: str) -> Modulator:
    """Instantiate a modulator by name (``"spwm"``)."""
    if method == "spwm":
        return SinusoidalPWM()
    raise ValueError(f"unknown modulation method {method!r}")
