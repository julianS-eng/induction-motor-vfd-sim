"""Induction-motor parameter containers.

All electrical parameters are per phase of the star-equivalent machine with
rotor quantities referred to the stator, which is the convention used both by
the steady-state T-equivalent circuit and by the dq model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

Provenance = Literal["assumed", "nameplate", "measured", "identified"]


@dataclass(frozen=True)
class Nameplate:
    """Nameplate (rating plate) data of the motor.

    Attributes
    ----------
    rated_power_w:
        Rated shaft output power in W.
    rated_voltage_v:
        Rated line-to-line RMS voltage in V.
    rated_current_a:
        Rated line RMS current in A.
    rated_frequency_hz:
        Rated supply frequency in Hz.
    rated_speed_rpm:
        Rated (full-load) shaft speed in r/min.
    poles:
        Number of magnetic poles (``2 * pole_pairs``).
    connection:
        Stator connection, ``"star"`` or ``"delta"``. The electrical parameters
        are always expressed for the star-equivalent machine.
    """

    rated_power_w: float
    rated_voltage_v: float
    rated_current_a: float
    rated_frequency_hz: float
    rated_speed_rpm: float
    poles: int
    connection: Literal["star", "delta"] = "star"

    def __post_init__(self) -> None:
        for name in (
            "rated_power_w",
            "rated_voltage_v",
            "rated_current_a",
            "rated_frequency_hz",
            "rated_speed_rpm",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"nameplate.{name} must be positive")
        if self.poles < 2 or self.poles % 2:
            raise ValueError("nameplate.poles must be an even integer >= 2")
        if self.connection not in ("star", "delta"):
            raise ValueError("nameplate.connection must be 'star' or 'delta'")
        if self.rated_speed_rpm >= self.synchronous_speed_rpm:
            raise ValueError("rated speed must be below synchronous speed (motoring)")

    @property
    def pole_pairs(self) -> int:
        """Number of pole pairs ``p = poles / 2``."""
        return self.poles // 2

    @property
    def synchronous_speed_rpm(self) -> float:
        """Synchronous speed at rated frequency, ``120 f / P`` in r/min."""
        return 120.0 * self.rated_frequency_hz / self.poles

    @property
    def rated_slip(self) -> float:
        """Rated slip ``(n_s - n_r) / n_s``."""
        ns = self.synchronous_speed_rpm
        return (ns - self.rated_speed_rpm) / ns

    @property
    def rated_slip_frequency_hz(self) -> float:
        """Rotor (slip) frequency at rated load, ``s_r f_r`` in Hz."""
        return self.rated_slip * self.rated_frequency_hz

    @property
    def rated_speed_rad_s(self) -> float:
        """Rated mechanical speed in rad/s."""
        return self.rated_speed_rpm * 2.0 * math.pi / 60.0

    @property
    def rated_torque_nm(self) -> float:
        """Rated shaft torque ``P_r / ω_r`` in N·m."""
        return self.rated_power_w / self.rated_speed_rad_s

    @property
    def rated_phase_voltage_v(self) -> float:
        """Rated phase RMS voltage of the star-equivalent machine."""
        return self.rated_voltage_v / math.sqrt(3.0)


@dataclass(frozen=True)
class MotorParameters:
    """Electrical and mechanical parameters of a squirrel-cage induction motor.

    Attributes
    ----------
    r_s:
        Stator resistance per phase, Ω.
    r_r:
        Rotor resistance per phase referred to the stator, Ω.
    l_ls:
        Stator leakage inductance, H.
    l_lr:
        Rotor leakage inductance referred to the stator, H.
    l_m:
        Magnetising inductance, H.
    inertia:
        Rotor (+ coupling) moment of inertia, kg·m².
    viscous_friction:
        Viscous friction coefficient of the motor bearings, N·m·s/rad.
    nameplate:
        Rating-plate data.
    name:
        Human-readable identifier.
    provenance:
        Where each parameter group comes from (``assumed``, ``nameplate``,
        ``measured`` or ``identified``).
    """

    r_s: float
    r_r: float
    l_ls: float
    l_lr: float
    l_m: float
    inertia: float
    viscous_friction: float
    nameplate: Nameplate
    name: str = "motor"
    provenance: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("r_s", "r_r", "l_ls", "l_lr", "l_m", "inertia"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"motor parameter {name} must be positive")
        if self.viscous_friction < 0.0:
            raise ValueError("viscous_friction must be non-negative")

    # ------------------------------------------------------------ derived
    @property
    def pole_pairs(self) -> int:
        """Number of pole pairs."""
        return self.nameplate.pole_pairs

    @property
    def l_s(self) -> float:
        """Stator self-inductance ``L_s = L_ls + L_m``."""
        return self.l_ls + self.l_m

    @property
    def l_r(self) -> float:
        """Rotor self-inductance ``L_r = L_lr + L_m``."""
        return self.l_lr + self.l_m

    @property
    def sigma(self) -> float:
        """Total leakage factor ``σ = 1 - L_m² / (L_s L_r)``."""
        return 1.0 - self.l_m**2 / (self.l_s * self.l_r)

    @property
    def rotor_time_constant(self) -> float:
        """Rotor time constant ``τ_r = L_r / R_r`` in s."""
        return self.l_r / self.r_r

    @property
    def transient_inductance(self) -> float:
        """Stator transient inductance ``σ L_s`` in H."""
        return self.sigma * self.l_s

    @property
    def is_assumed(self) -> bool:
        """``True`` if any parameter group is flagged as an assumption."""
        return any(v == "assumed" for v in self.provenance.values())

    def rated_flux_linkage(self) -> float:
        """Approximate rated stator flux-linkage amplitude ``V̂ / ω`` in Wb."""
        v_peak = self.nameplate.rated_phase_voltage_v * math.sqrt(2.0)
        return v_peak / (2.0 * math.pi * self.nameplate.rated_frequency_hz)
