"""Scalar V/f (volts-per-hertz) control with boost and slip compensation.

Principle
---------
Neglecting the stator resistance, the stator flux amplitude is
``ψ_s ≈ V̂_s / ω_e``; keeping ``V/f`` constant therefore keeps the flux (and
the torque capability) constant while the frequency is varied. At low
frequency the ``R_s i_s`` drop is no longer negligible and a *boost* voltage
is added. Above base frequency the voltage is clamped (field weakening).

Signal flow (one call of :meth:`VFController.step` per control period)::

    n* ──► f* = p n*/60 ──► ramp ──► f_ramp ──(+)──► f_out ──► θ += 2π f_out T_s
                                             ▲                 │
                            slip comp. Δf ───┘                 ▼
                                                   V = V(f_out) (profile + boost,
                                                   limited by the modulator)
                                                   v_αβ* = V [cos θ, sin θ]

Slip compensation estimates the air-gap torque from the input power minus the
stator copper losses, ``T̂ = (p_in - 3/2 R_s |i_s|²) p / ω_e``, and adds the slip
frequency the machine needs to develop it::

    Δf = K · f_sl,rated · (T̂ / T_rated) / (ψ/ψ_rated)²

(the flux correction is only active in field weakening). The estimate is
low-pass filtered because it is derived from noisy, ripple-laden currents.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

from vfdsim.control.base import Measurements
from vfdsim.control.ramp import RampGenerator
from vfdsim.modulation.base import Modulator
from vfdsim.motor.parameters import MotorParameters
from vfdsim.transforms import TWO_PI, clarke

SQRT2_3 = math.sqrt(2.0 / 3.0)


@dataclass(frozen=True)
class VFSettings:
    """Tuning of the V/f controller.

    Attributes
    ----------
    base_frequency_hz:
        Frequency at which the base voltage is reached.
    base_voltage_v:
        Line-to-line RMS voltage at base frequency.
    profile:
        ``"linear"`` (constant torque) or ``"quadratic"`` (fans/pumps).
    boost_fraction:
        Voltage at 0 Hz as a fraction of the base voltage.
    max_frequency_hz:
        Output frequency limit.
    accel_rate_hz_per_s, decel_rate_hz_per_s:
        Frequency ramp rates.
    slip_compensation_gain:
        Gain ``K`` of the slip compensation (0 disables it).
    slip_filter_time_constant_s:
        Time constant of the first-order filter on the slip estimate.
    current_limit_a:
        RMS current above which the acceleration ramp and the slip
        compensation are held (stall prevention). ``None`` disables it.
    allow_overmodulation:
        If ``False`` the voltage is limited to the linear range of the
        modulator; otherwise the modulator clips.
    current_filter_time_constant_s:
        Filter time constant of the RMS current used by the current limit.
    min_estimation_frequency_hz:
        Below this output frequency the torque estimate is disabled.
    """

    base_frequency_hz: float = 60.0
    base_voltage_v: float = 208.0
    profile: Literal["linear", "quadratic"] = "linear"
    boost_fraction: float = 0.06
    max_frequency_hz: float = 60.0
    accel_rate_hz_per_s: float = 30.0
    decel_rate_hz_per_s: float = 30.0
    slip_compensation_gain: float = 1.0
    slip_filter_time_constant_s: float = 0.05
    current_limit_a: float | None = 3.0
    allow_overmodulation: bool = False
    current_filter_time_constant_s: float = 0.005
    min_estimation_frequency_hz: float = 1.0

    def __post_init__(self) -> None:
        if self.base_frequency_hz <= 0.0 or self.base_voltage_v <= 0.0:
            raise ValueError("base frequency and voltage must be positive")
        if self.profile not in ("linear", "quadratic"):
            raise ValueError("profile must be 'linear' or 'quadratic'")
        if not 0.0 <= self.boost_fraction < 1.0:
            raise ValueError("boost_fraction must be in [0, 1)")
        if self.max_frequency_hz <= 0.0:
            raise ValueError("max_frequency_hz must be positive")
        if self.slip_compensation_gain < 0.0:
            raise ValueError("slip_compensation_gain must be non-negative")
        if self.slip_filter_time_constant_s <= 0.0:
            raise ValueError("slip_filter_time_constant_s must be positive")
        if self.current_limit_a is not None and self.current_limit_a <= 0.0:
            raise ValueError("current_limit_a must be positive or None")

    @property
    def base_phase_peak_v(self) -> float:
        """Peak phase voltage at base frequency."""
        return self.base_voltage_v * SQRT2_3


def vf_voltage_peak(frequency_hz: float, settings: VFSettings) -> float:
    """Peak phase voltage commanded by the V/f profile at ``frequency_hz``.

    ``V(f) = V_0 + (V_b - V_0) x`` (linear) or ``V_0 + (V_b - V_0) x²``
    (quadratic), with ``x = |f| / f_b`` saturated at 1 and ``V_0`` the boost.
    """
    v_base = settings.base_phase_peak_v
    v_boost = settings.boost_fraction * v_base
    x = abs(frequency_hz) / settings.base_frequency_hz
    if x >= 1.0:
        return v_base
    if settings.profile == "quadratic":
        x = x * x
    return v_boost + (v_base - v_boost) * x


class VFController:
    """Open-loop scalar V/f controller.

    Parameters
    ----------
    settings:
        Controller tuning.
    motor:
        Motor parameters (nameplate data for the slip model and ``R_s`` for the
        torque estimate; the controller does not use the dq model).
    modulator:
        Modulator, used to know the available voltage for a given DC link.
    sample_time:
        Control period in s.
    speed_reference_rpm:
        Speed set-point as a function of time (r/min).
    voltage_delay_samples:
        Number of samples between computing a voltage command and applying it
        (1 with the usual computation delay). Used to align the voltage and
        current vectors in the power estimate: the fundamental of a
        zero-order-held command lags it by ``(0.5 + d) ω T_s``.
    """

    def __init__(
        self,
        settings: VFSettings,
        motor: MotorParameters,
        modulator: Modulator,
        sample_time: float,
        speed_reference_rpm: Callable[[float], float],
        voltage_delay_samples: int = 1,
    ) -> None:
        if sample_time <= 0.0:
            raise ValueError("sample_time must be positive")
        if voltage_delay_samples < 0:
            raise ValueError("voltage_delay_samples must be non-negative")
        self.voltage_delay_samples = voltage_delay_samples
        self.settings = settings
        self.modulator = modulator
        self.sample_time = sample_time
        self.speed_reference_rpm = speed_reference_rpm
        self._pp = float(motor.pole_pairs)
        self._r_s = motor.r_s
        self._t_rated = motor.nameplate.rated_torque_nm
        self._f_slip_rated = motor.nameplate.rated_slip_frequency_hz
        self._vf_ratio_base = settings.base_phase_peak_v / settings.base_frequency_hz
        self._ramp = RampGenerator(settings.accel_rate_hz_per_s, settings.decel_rate_hz_per_s)
        self._k_slip = sample_time / max(settings.slip_filter_time_constant_s, sample_time)
        self._k_cur = sample_time / max(settings.current_filter_time_constant_s, sample_time)
        self.reset()

    def reset(self) -> None:
        """Reset ramp, angle, filters and signal log."""
        self._ramp.reset(0.0)
        self._theta = 0.0
        self._f_out = 0.0
        self._slip_hz = 0.0
        self._i_rms_f = 0.0
        self._v_alpha = 0.0
        self._v_beta = 0.0
        self._v_peak = 0.0
        self._signals: dict[str, float] = {}

    def step(self, m: Measurements) -> tuple[float, float]:
        """Execute one control period and return ``(v_α*, v_β*)``."""
        s = self.settings
        ts = self.sample_time

        # --- set-point and ramp (with stall prevention) -----------------------
        f_target = self.speed_reference_rpm(m.t) * self._pp / 60.0
        f_target = max(-s.max_frequency_hz, min(s.max_frequency_hz, f_target))
        i_alpha, i_beta = clarke(m.i_a, m.i_b, m.i_c)
        i_sq = i_alpha * i_alpha + i_beta * i_beta
        self._i_rms_f += self._k_cur * (math.sqrt(0.5 * i_sq) - self._i_rms_f)
        hold = s.current_limit_a is not None and self._i_rms_f > s.current_limit_a
        f_ramp = self._ramp.step(f_target, ts, hold_acceleration=hold)

        # --- slip compensation from an air-gap torque estimate ----------------
        f_prev = self._f_out
        torque_est = 0.0
        slip_raw = 0.0
        if abs(f_prev) >= s.min_estimation_frequency_hz:
            # Applied fundamental voltage at the sampling instant: last command
            # rotated by the ZOH + computation-delay phase shift.
            phi = TWO_PI * f_prev * ts * (0.5 - self.voltage_delay_samples)
            c_phi, s_phi = math.cos(phi), math.sin(phi)
            v_a = self._v_alpha * c_phi - self._v_beta * s_phi
            v_b = self._v_alpha * s_phi + self._v_beta * c_phi
            p_in = 1.5 * (v_a * i_alpha + v_b * i_beta)
            p_ag = p_in - 1.5 * self._r_s * i_sq
            torque_est = p_ag * self._pp / (TWO_PI * f_prev)
            flux_ratio = min(1.0, (self._v_peak / abs(f_prev)) / self._vf_ratio_base)
            flux_ratio = max(flux_ratio, 0.2)
            slip_raw = self._f_slip_rated * (torque_est / self._t_rated) / (flux_ratio**2)
        # While the current limit is active the slip compensation may not grow
        # in magnitude: otherwise it would keep raising the frequency and
        # defeat the stall prevention.
        slip_new = self._slip_hz + self._k_slip * (slip_raw - self._slip_hz)
        if not hold or abs(slip_new) < abs(self._slip_hz):
            self._slip_hz = slip_new
        limit = 2.0 * self._f_slip_rated
        delta_f = s.slip_compensation_gain * max(-limit, min(limit, self._slip_hz))
        f_out = f_ramp + delta_f
        # The maximum output frequency is a hard limit (it protects the mechanics),
        # so the slip compensation cannot push the output beyond it.
        f_out = max(-s.max_frequency_hz, min(s.max_frequency_hz, f_out))

        # --- angle integration and voltage magnitude --------------------------
        self._theta = (self._theta + TWO_PI * f_out * ts) % TWO_PI
        v_peak = vf_voltage_peak(f_out, s)
        if s.allow_overmodulation:
            v_max = 2.0 * m.v_dc / math.pi  # six-step fundamental
        else:
            v_max = self.modulator.max_phase_peak(m.v_dc)
        v_peak = min(v_peak, v_max)

        self._f_out = f_out
        self._v_peak = v_peak
        self._v_alpha = v_peak * math.cos(self._theta)
        self._v_beta = v_peak * math.sin(self._theta)
        self._signals = {
            "f_target_hz": f_target,
            "f_ramp_hz": f_ramp,
            "f_out_hz": f_out,
            "slip_comp_hz": delta_f,
            "torque_est_nm": torque_est,
            "v_cmd_peak_v": v_peak,
            "i_rms_filtered_a": self._i_rms_f,
            "ramp_hold": 1.0 if hold else 0.0,
            "theta_e_rad": self._theta,
        }
        return self._v_alpha, self._v_beta

    def signals(self) -> dict[str, float]:
        """Internal signals of the last step."""
        return self._signals
