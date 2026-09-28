"""Continuous-time plant: induction motor + shaft + load + energy bookkeeping.

State vector layout (see the ``IDX_*`` constants):

====  ===========  =============================================
idx   symbol       meaning
====  ===========  =============================================
0-1   ψ_sα, ψ_sβ   stator flux linkage (Wb)
2-3   ψ_rα, ψ_rβ   rotor flux linkage referred to stator (Wb)
4     ω_m          mechanical speed (rad/s)
5     θ_m          mechanical angle (rad, not wrapped)
6     E_in         electrical energy into the stator terminals (J)
7     E_cu,s       stator copper losses (J)
8     E_cu,r       rotor copper losses (J)
9     E_em         electromagnetic work ``∫T_e ω_m dt`` (J)
10    E_load       work delivered to the load ``∫T_L ω_m dt`` (J)
11    E_fric       motor bearing friction losses (J)
====  ===========  =============================================

The energy accumulators are integrated together with the physical states, so
power balances can be verified to integration accuracy instead of from
sampled waveforms.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from vfdsim.mechanics.loads import Load
from vfdsim.motor.dq_model import InductionMotorModel

IDX_PSI_SA = 0
IDX_PSI_SB = 1
IDX_PSI_RA = 2
IDX_PSI_RB = 3
IDX_OMEGA = 4
IDX_THETA = 5
IDX_E_IN = 6
IDX_E_CU_S = 7
IDX_E_CU_R = 8
IDX_E_EM = 9
IDX_E_LOAD = 10
IDX_E_FRIC = 11
N_STATES = 12


@dataclass(frozen=True, slots=True)
class PlantOutputs:
    """Algebraic outputs of the plant at one instant."""

    i_sa: float
    i_sb: float
    i_ra: float
    i_rb: float
    torque_e: float
    torque_load: float
    torque_friction: float
    psi_s: float
    psi_r: float

    @property
    def currents_abc(self) -> tuple[float, float, float]:
        """Stator phase currents ``(i_a, i_b, i_c)``."""
        a = self.i_sa
        b = -0.5 * self.i_sa + 0.8660254037844386 * self.i_sb
        return a, b, -a - b


class DrivePlant:
    """Induction motor coupled to a mechanical load.

    Parameters
    ----------
    motor:
        dq model of the machine.
    load:
        Mechanical load (its inertia is added to the rotor inertia).
    locked_speed_rad_s:
        If given, the shaft speed is held at this value (speed source), which
        is how the torque–speed characteristic is measured on a dynamometer.
    """

    def __init__(
        self,
        motor: InductionMotorModel,
        load: Load,
        *,
        locked_speed_rad_s: float | None = None,
    ) -> None:
        self.motor = motor
        self.load = load
        self.inertia = motor.params.inertia + load.inertia
        self.viscous_friction = motor.params.viscous_friction
        self.locked_speed_rad_s = locked_speed_rad_s
        self._inv_j = 1.0 / self.inertia
        self._pp = float(motor.pole_pairs)
        self._r_s = motor.params.r_s
        self._r_r = motor.params.r_r

    def initial_state(
        self,
        omega_m: float = 0.0,
        theta_m: float = 0.0,
        fluxes: Sequence[float] = (0.0, 0.0, 0.0, 0.0),
    ) -> list[float]:
        """Build an initial state vector (energies start at zero)."""
        if self.locked_speed_rad_s is not None:
            omega_m = self.locked_speed_rad_s
        x = [0.0] * N_STATES
        x[IDX_PSI_SA : IDX_PSI_RB + 1] = [float(v) for v in fluxes]
        x[IDX_OMEGA] = omega_m
        x[IDX_THETA] = theta_m
        return x

    def derivatives(self, t: float, x: Sequence[float], v_sa: float, v_sb: float) -> list[float]:
        """Right-hand side ``dx/dt`` for a stator voltage ``(v_sα, v_sβ)``."""
        omega = x[IDX_OMEGA]
        d_sa, d_sb, d_ra, d_rb, i_sa, i_sb, i_ra, i_rb = self.motor.electrical_derivatives(
            x[0], x[1], x[2], x[3], v_sa, v_sb, omega
        )
        torque_e = 1.5 * self._pp * (x[0] * i_sb - x[1] * i_sa)
        torque_l = self.load.torque(t, omega)
        torque_f = self.viscous_friction * omega
        if self.locked_speed_rad_s is None:
            d_omega = (torque_e - torque_l - torque_f) * self._inv_j
        else:
            d_omega = 0.0
        return [
            d_sa,
            d_sb,
            d_ra,
            d_rb,
            d_omega,
            omega,
            1.5 * (v_sa * i_sa + v_sb * i_sb),
            1.5 * self._r_s * (i_sa * i_sa + i_sb * i_sb),
            1.5 * self._r_r * (i_ra * i_ra + i_rb * i_rb),
            torque_e * omega,
            torque_l * omega,
            torque_f * omega,
        ]

    def outputs(self, t: float, x: Sequence[float]) -> PlantOutputs:
        """Currents, torques and flux magnitudes at state ``x``."""
        i_sa, i_sb, i_ra, i_rb = self.motor.currents(x[0], x[1], x[2], x[3])
        omega = x[IDX_OMEGA]
        return PlantOutputs(
            i_sa=i_sa,
            i_sb=i_sb,
            i_ra=i_ra,
            i_rb=i_rb,
            torque_e=1.5 * self._pp * (x[0] * i_sb - x[1] * i_sa),
            torque_load=self.load.torque(t, omega),
            torque_friction=self.viscous_friction * omega,
            psi_s=math.hypot(x[0], x[1]),
            psi_r=math.hypot(x[2], x[3]),
        )

    def stored_energy(self, x: Sequence[float]) -> tuple[float, float]:
        """Magnetic and kinetic energy ``(W_mag, W_kin)`` in J.

        ``W_mag = 3/4 (ψ_s·i_s + ψ_r·i_r)`` in the amplitude-invariant frame.
        """
        i_sa, i_sb, i_ra, i_rb = self.motor.currents(x[0], x[1], x[2], x[3])
        w_mag = 0.75 * (x[0] * i_sa + x[1] * i_sb + x[2] * i_ra + x[3] * i_rb)
        w_kin = 0.5 * self.inertia * x[IDX_OMEGA] ** 2
        return w_mag, w_kin
