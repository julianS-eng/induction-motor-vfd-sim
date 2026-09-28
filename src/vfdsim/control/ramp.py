"""Rate limiter (ramp generator) used for frequency and speed set-points."""

from __future__ import annotations


class RampGenerator:
    """Slew-rate limiter with separate acceleration and deceleration rates.

    "Acceleration" means increasing magnitude (moving away from zero) and
    "deceleration" means decreasing magnitude, as in industrial drives.

    Parameters
    ----------
    accel_rate:
        Maximum rate of increase of ``|value|`` per second (> 0).
    decel_rate:
        Maximum rate of decrease of ``|value|`` per second (> 0).
    initial:
        Initial output value.
    """

    def __init__(self, accel_rate: float, decel_rate: float, initial: float = 0.0) -> None:
        if accel_rate <= 0.0 or decel_rate <= 0.0:
            raise ValueError("ramp rates must be positive")
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self._initial = initial
        self.value = initial

    def reset(self, value: float | None = None) -> None:
        """Reset the output (to ``initial`` if no value is given)."""
        self.value = self._initial if value is None else value

    def step(self, target: float, dt: float, hold_acceleration: bool = False) -> float:
        """Advance one sample towards ``target``.

        Parameters
        ----------
        target:
            Set-point to track.
        dt:
            Sample time in s.
        hold_acceleration:
            If ``True`` the output is frozen while it would increase in
            magnitude (used for stall prevention / current limit).
        """
        v = self.value
        err = target - v
        if err == 0.0:
            return v
        accelerating = (err > 0.0 and v >= 0.0) or (err < 0.0 and v <= 0.0)
        if accelerating and hold_acceleration:
            return v
        max_step = (self.accel_rate if accelerating else self.decel_rate) * dt
        if err > max_step:
            err = max_step
        elif err < -max_step:
            err = -max_step
        self.value = v + err
        return self.value
