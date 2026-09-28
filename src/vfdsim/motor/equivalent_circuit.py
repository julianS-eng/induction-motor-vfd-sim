r"""Steady-state per-phase T-equivalent circuit of the induction motor.

::

      R_s     jX_ls              jX_lr
    o--/\/\/--~~~~~~--+--------~~~~~~--+
                     |                |
    V_ph           jX_m             R_r / s
                     |                |
    o----------------+----------------+

The circuit is expressed per phase of the star-equivalent machine, with RMS
phasors and reactances evaluated at the supply frequency (``X = 2π f L``).
Core losses are not represented (no ``R_c`` branch), consistent with the dq
model. The rotor branch is handled through its admittance
``Y_r = s / (R_r + j s X_lr)`` so the synchronous point ``s = 0`` is regular.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.optimize import brentq

from vfdsim.motor.parameters import MotorParameters

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class OperatingPoint:
    """Steady-state operating point predicted by the equivalent circuit.

    Currents are RMS phasors referred to the phase voltage (angle 0). Powers
    are three-phase totals.
    """

    slip: float
    frequency_hz: float
    phase_voltage_rms: float
    stator_current: complex
    rotor_current: complex
    magnetizing_current: complex
    torque_nm: float
    speed_rpm: float
    input_power_w: float
    reactive_power_var: float
    stator_copper_loss_w: float
    rotor_copper_loss_w: float
    air_gap_power_w: float
    internal_mechanical_power_w: float

    @property
    def power_factor(self) -> float:
        """Displacement power factor ``cos φ``."""
        s = math.hypot(self.input_power_w, self.reactive_power_var)
        return self.input_power_w / s if s > 0.0 else 0.0

    @property
    def stator_current_rms_a(self) -> float:
        """Line (= phase, star) RMS current."""
        return abs(self.stator_current)


class EquivalentCircuit:
    """Per-phase T-equivalent circuit built from :class:`MotorParameters`."""

    def __init__(self, params: MotorParameters) -> None:
        self.params = params

    # ---------------------------------------------------------------- helpers
    def synchronous_speed_rad_s(self, frequency_hz: float) -> float:
        """Mechanical synchronous speed ``ω_s / p`` in rad/s."""
        return 2.0 * math.pi * frequency_hz / self.params.pole_pairs

    def synchronous_speed_rpm(self, frequency_hz: float) -> float:
        """Mechanical synchronous speed ``120 f / P`` in r/min."""
        return 120.0 * frequency_hz / self.params.nameplate.poles

    def _reactances(self, frequency_hz: float) -> tuple[float, float, float]:
        w = 2.0 * math.pi * frequency_hz
        p = self.params
        return w * p.l_ls, w * p.l_lr, w * p.l_m

    def thevenin(self, v_phase_rms: float, frequency_hz: float) -> tuple[complex, complex]:
        """Thevenin equivalent seen by the rotor branch: ``(V_th, Z_th)``."""
        x_ls, _, x_m = self._reactances(frequency_hz)
        z_s = complex(self.params.r_s, x_ls)
        z_m = complex(0.0, x_m)
        v_th = v_phase_rms * z_m / (z_s + z_m)
        z_th = z_s * z_m / (z_s + z_m)
        return v_th, z_th

    # ---------------------------------------------------------- core relations
    def torque(self, slip: ArrayLike, v_line_rms: float, frequency_hz: float) -> FloatArray:
        """Electromagnetic torque (N·m) for one or many slips (vectorised)."""
        s = np.asarray(slip, dtype=np.float64)
        v_ph = v_line_rms / math.sqrt(3.0)
        v_th, z_th = self.thevenin(v_ph, frequency_hz)
        _, x_lr, _ = self._reactances(frequency_hz)
        r_r = self.params.r_r
        # I_r = V_th / (Z_th + R_r/s + jX_lr) = V_th s / (s Z_th + R_r + j s X_lr)
        # P_ag = 3 |I_r|² R_r / s = 3 |V_th|² s R_r / |s Z_th + R_r + j s X_lr|²
        denom = s * z_th + r_r + 1j * s * x_lr
        p_ag = 3.0 * abs(v_th) ** 2 * s * r_r / np.abs(denom) ** 2
        return np.asarray(p_ag / self.synchronous_speed_rad_s(frequency_hz), dtype=np.float64)

    def stator_current(
        self, slip: ArrayLike, v_line_rms: float, frequency_hz: float
    ) -> NDArray[np.complex128]:
        """Stator RMS current phasor(s) for one or many slips (vectorised)."""
        s = np.asarray(slip, dtype=np.float64)
        x_ls, x_lr, x_m = self._reactances(frequency_hz)
        y_r = s / (self.params.r_r + 1j * s * x_lr)
        y_m = 1.0 / (1j * x_m)
        z = self.params.r_s + 1j * x_ls + 1.0 / (y_m + y_r)
        return np.asarray((v_line_rms / math.sqrt(3.0)) / z, dtype=np.complex128)

    def operating_point(
        self, slip: float, v_line_rms: float, frequency_hz: float
    ) -> OperatingPoint:
        """Full steady-state solution at the given slip."""
        p = self.params
        v_ph = v_line_rms / math.sqrt(3.0)
        x_ls, x_lr, x_m = self._reactances(frequency_hz)
        y_r = slip / complex(p.r_r, slip * x_lr)
        y_m = 1.0 / complex(0.0, x_m)
        z_par = 1.0 / (y_m + y_r)
        i_s = v_ph / (complex(p.r_s, x_ls) + z_par)
        e_m = i_s * z_par  # air-gap EMF
        i_r = e_m * y_r
        i_m = e_m * y_m
        s_in = 3.0 * v_ph * i_s.conjugate()
        p_cu_s = 3.0 * p.r_s * abs(i_s) ** 2
        p_cu_r = 3.0 * p.r_r * abs(i_r) ** 2
        p_ag = s_in.real - p_cu_s
        w_sync = self.synchronous_speed_rad_s(frequency_hz)
        return OperatingPoint(
            slip=slip,
            frequency_hz=frequency_hz,
            phase_voltage_rms=v_ph,
            stator_current=i_s,
            rotor_current=i_r,
            magnetizing_current=i_m,
            torque_nm=p_ag / w_sync,
            speed_rpm=(1.0 - slip) * self.synchronous_speed_rpm(frequency_hz),
            input_power_w=s_in.real,
            reactive_power_var=s_in.imag,
            stator_copper_loss_w=p_cu_s,
            rotor_copper_loss_w=p_cu_r,
            air_gap_power_w=p_ag,
            internal_mechanical_power_w=p_ag - p_cu_r,
        )

    # ------------------------------------------------------------ derived
    def breakdown(self, v_line_rms: float, frequency_hz: float) -> tuple[float, float]:
        """Breakdown (pull-out) slip and torque in motoring, closed form.

        ``s_max = R_r / sqrt(R_th² + (X_th + X_lr)²)`` and
        ``T_max = 3 V_th² / (2 ω_sm (R_th + sqrt(R_th² + (X_th + X_lr)²)))``.
        """
        v_ph = v_line_rms / math.sqrt(3.0)
        v_th, z_th = self.thevenin(v_ph, frequency_hz)
        _, x_lr, _ = self._reactances(frequency_hz)
        root = math.hypot(z_th.real, z_th.imag + x_lr)
        s_max = self.params.r_r / root
        t_max = (
            3.0
            * abs(v_th) ** 2
            / (2.0 * self.synchronous_speed_rad_s(frequency_hz) * (z_th.real + root))
        )
        return s_max, t_max

    def torque_speed_curve(
        self,
        v_line_rms: float,
        frequency_hz: float,
        n_points: int = 400,
        slip_range: tuple[float, float] = (1.0, 0.0),
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """Return ``(speed_rpm, torque_nm, current_rms_a)`` over a slip range."""
        s = np.linspace(slip_range[0], slip_range[1], n_points)
        torque = self.torque(s, v_line_rms, frequency_hz)
        current = np.abs(self.stator_current(s, v_line_rms, frequency_hz))
        speed = (1.0 - s) * self.synchronous_speed_rpm(frequency_hz)
        return speed, torque, np.asarray(current, dtype=np.float64)

    def slip_for_torque(self, torque_nm: float, v_line_rms: float, frequency_hz: float) -> float:
        """Slip on the stable branch (``0 ≤ s ≤ s_max``) producing ``torque_nm``.

        Raises
        ------
        ValueError
            If the requested torque exceeds the breakdown torque.
        """
        s_max, t_max = self.breakdown(v_line_rms, frequency_hz)
        if torque_nm > t_max:
            raise ValueError(f"requested torque {torque_nm:.3f} N·m exceeds T_max={t_max:.3f}")
        if torque_nm <= 0.0:
            return 0.0

        def residual(s: float) -> float:
            return float(self.torque(s, v_line_rms, frequency_hz)) - torque_nm

        return float(brentq(residual, 0.0, s_max, xtol=1e-12))

    def operating_point_with_load(
        self,
        load_torque: Callable[[float], float],
        v_line_rms: float,
        frequency_hz: float,
    ) -> OperatingPoint:
        """Stable steady-state point where ``T_e(ω) = T_load(ω)``.

        Parameters
        ----------
        load_torque:
            Total opposing torque (load + friction) as a function of the
            mechanical speed in rad/s.
        """
        s_max, _ = self.breakdown(v_line_rms, frequency_hz)
        w_sync = self.synchronous_speed_rad_s(frequency_hz)

        def residual(s: float) -> float:
            return float(self.torque(s, v_line_rms, frequency_hz)) - load_torque((1.0 - s) * w_sync)

        if residual(s_max) < 0.0:
            raise ValueError("load torque exceeds the breakdown torque: the motor stalls")
        s = float(brentq(residual, 0.0, s_max, xtol=1e-12))
        return self.operating_point(s, v_line_rms, frequency_hz)
