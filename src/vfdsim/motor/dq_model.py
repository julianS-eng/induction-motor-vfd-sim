"""Dynamic dq model of the squirrel-cage induction motor.

The model uses stator and rotor flux linkages as electrical state variables and
is written in the stationary (αβ) reference frame, so the inverter voltages can
be applied directly without any angle information. All vectors follow the
amplitude-invariant convention of :mod:`vfdsim.transforms`.

Electrical equations (rotor short-circuited, ``ω_r = p ω_m`` electrical)::

    dψ_s/dt = v_s - R_s i_s
    dψ_r/dt = -R_r i_r + ω_r J ψ_r          J = [[0, -1], [1, 0]]

    ψ_s = L_s i_s + L_m i_r
    ψ_r = L_m i_s + L_r i_r

Electromagnetic torque::

    T_e = 3/2 p (ψ_sα i_sβ - ψ_sβ i_sα)

The mechanical equation lives in :mod:`vfdsim.simulation.plant`, where the
load models are attached.
"""

from __future__ import annotations

import cmath
import math
from dataclasses import dataclass

from vfdsim.motor.parameters import MotorParameters


@dataclass(frozen=True)
class DQSteadyState:
    """Sinusoidal steady state of the dq model in the synchronous frame.

    Complex numbers are space-vector phasors (magnitude = peak phase value),
    with the stator voltage aligned with the real axis.
    """

    slip: float
    electrical_frequency_hz: float
    v_s: complex
    i_s: complex
    i_r: complex
    psi_s: complex
    psi_r: complex
    torque_nm: float
    input_power_w: float
    reactive_power_var: float
    stator_copper_loss_w: float
    rotor_copper_loss_w: float
    air_gap_power_w: float

    @property
    def mechanical_power_w(self) -> float:
        """Internal mechanical power ``(1 - s) P_ag``."""
        return (1.0 - self.slip) * self.air_gap_power_w

    @property
    def stator_current_rms_a(self) -> float:
        """Stator phase RMS current."""
        return abs(self.i_s) / math.sqrt(2.0)


class InductionMotorModel:
    """Flux-linkage state-space model of the induction motor (stationary frame).

    Parameters
    ----------
    params:
        Motor parameters.
    """

    def __init__(self, params: MotorParameters) -> None:
        self.params = params
        det = params.l_s * params.l_r - params.l_m**2
        # Inverse inductance matrix coefficients: i = L^-1 ψ
        self._k_ss = params.l_r / det
        self._k_sr = params.l_m / det
        self._k_rr = params.l_s / det
        self._pp = float(params.pole_pairs)
        self._r_s = params.r_s
        self._r_r = params.r_r

    @property
    def pole_pairs(self) -> int:
        """Number of pole pairs."""
        return self.params.pole_pairs

    def currents(
        self, psi_sa: float, psi_sb: float, psi_ra: float, psi_rb: float
    ) -> tuple[float, float, float, float]:
        """Return stator and rotor currents ``(i_sα, i_sβ, i_rα, i_rβ)`` from fluxes."""
        k_ss, k_sr, k_rr = self._k_ss, self._k_sr, self._k_rr
        return (
            k_ss * psi_sa - k_sr * psi_ra,
            k_ss * psi_sb - k_sr * psi_rb,
            k_rr * psi_ra - k_sr * psi_sa,
            k_rr * psi_rb - k_sr * psi_sb,
        )

    def torque(self, psi_sa: float, psi_sb: float, i_sa: float, i_sb: float) -> float:
        """Electromagnetic torque in N·m."""
        return 1.5 * self._pp * (psi_sa * i_sb - psi_sb * i_sa)

    def electrical_derivatives(
        self,
        psi_sa: float,
        psi_sb: float,
        psi_ra: float,
        psi_rb: float,
        v_sa: float,
        v_sb: float,
        omega_m: float,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Flux derivatives plus the currents they were computed with.

        Returns
        -------
        tuple
            ``(dψ_sα, dψ_sβ, dψ_rα, dψ_rβ, i_sα, i_sβ, i_rα, i_rβ)``.
        """
        i_sa, i_sb, i_ra, i_rb = self.currents(psi_sa, psi_sb, psi_ra, psi_rb)
        omega_r = self._pp * omega_m
        return (
            v_sa - self._r_s * i_sa,
            v_sb - self._r_s * i_sb,
            -self._r_r * i_ra - omega_r * psi_rb,
            -self._r_r * i_rb + omega_r * psi_ra,
            i_sa,
            i_sb,
            i_ra,
            i_rb,
        )

    def steady_state(self, v_phase_rms: float, frequency_hz: float, slip: float) -> DQSteadyState:
        """Solve the sinusoidal steady state for a given slip.

        Parameters
        ----------
        v_phase_rms:
            Stator phase RMS voltage (fundamental).
        frequency_hz:
            Stator electrical frequency, must be non-zero.
        slip:
            Slip ``s = (ω_e - ω_r) / ω_e``.
        """
        if frequency_hz == 0.0:
            raise ValueError("steady state requires a non-zero stator frequency")
        p = self.params
        w_e = 2.0 * math.pi * frequency_hz
        w_sl = slip * w_e
        v_s = complex(v_phase_rms * math.sqrt(2.0), 0.0)
        # [v_s; 0] = [[R_s + jω_e L_s, jω_e L_m], [jω_sl L_m, R_r + jω_sl L_r]] [i_s; i_r]
        a11 = complex(p.r_s, w_e * p.l_s)
        a12 = complex(0.0, w_e * p.l_m)
        a21 = complex(0.0, w_sl * p.l_m)
        a22 = complex(p.r_r, w_sl * p.l_r)
        det = a11 * a22 - a12 * a21
        i_s = v_s * a22 / det
        i_r = -v_s * a21 / det
        psi_s = p.l_s * i_s + p.l_m * i_r
        psi_r = p.l_m * i_s + p.l_r * i_r
        torque = 1.5 * self._pp * (psi_s.conjugate() * i_s).imag
        s_in = 1.5 * v_s * i_s.conjugate()
        p_cu_s = 1.5 * p.r_s * abs(i_s) ** 2
        p_cu_r = 1.5 * p.r_r * abs(i_r) ** 2
        return DQSteadyState(
            slip=slip,
            electrical_frequency_hz=frequency_hz,
            v_s=v_s,
            i_s=i_s,
            i_r=i_r,
            psi_s=psi_s,
            psi_r=psi_r,
            torque_nm=torque,
            input_power_w=s_in.real,
            reactive_power_var=s_in.imag,
            stator_copper_loss_w=p_cu_s,
            rotor_copper_loss_w=p_cu_r,
            air_gap_power_w=s_in.real - p_cu_s,
        )

    def steady_state_initial_fluxes(
        self, v_phase_rms: float, frequency_hz: float, slip: float, angle: float = 0.0
    ) -> tuple[float, float, float, float]:
        """Stationary-frame fluxes of the steady state at voltage angle ``angle``.

        Useful to start a simulation directly in sinusoidal steady state.
        """
        ss = self.steady_state(v_phase_rms, frequency_hz, slip)
        rot = cmath.exp(1j * angle)
        psi_s = ss.psi_s * rot
        psi_r = ss.psi_r * rot
        return psi_s.real, psi_s.imag, psi_r.real, psi_r.imag
