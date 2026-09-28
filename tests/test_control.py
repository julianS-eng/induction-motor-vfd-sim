from __future__ import annotations

import math

import pytest

from vfdsim.control import (
    Measurements,
    PiecewiseConstant,
    RampGenerator,
    VFController,
    VFSettings,
    vf_voltage_peak,
)
from vfdsim.modulation import SinusoidalPWM, make_modulator
from vfdsim.motor import MotorParameters
from vfdsim.power import AveragedInverter, dc_link_current, make_power_stage, pole_to_alpha_beta

TS = 1e-4


# ---------------------------------------------------------------- ramp / ref
def test_ramp_rates_and_hold() -> None:
    r = RampGenerator(accel_rate=10.0, decel_rate=5.0)
    for _ in range(1000):  # 0.1 s
        r.step(100.0, TS)
    assert r.value == pytest.approx(1.0)
    r.step(100.0, TS, hold_acceleration=True)
    assert r.value == pytest.approx(1.0)
    for _ in range(1000):
        r.step(0.0, TS)
    assert r.value == pytest.approx(0.5)
    r.step(0.0, TS, hold_acceleration=True)  # hold does not block deceleration
    assert r.value == pytest.approx(0.5 - 5.0 * TS)
    r.reset()
    assert r.value == 0.0
    with pytest.raises(ValueError, match="positive"):
        RampGenerator(0.0, 1.0)


def test_ramp_negative_direction_and_exact_target() -> None:
    r = RampGenerator(accel_rate=10.0, decel_rate=10.0, initial=0.0)
    for _ in range(20000):
        r.step(-3.0, TS)
    assert r.value == -3.0
    assert r.step(-3.0, TS) == -3.0


def test_piecewise_constant_reference() -> None:
    ref = PiecewiseConstant.from_pairs([[0.5, 100.0], [1.0, -50.0]])
    assert ref(0.0) == 100.0
    assert ref(0.75) == 100.0
    assert ref(1.0) == -50.0
    assert PiecewiseConstant.constant(7.0)(123.0) == 7.0
    with pytest.raises(ValueError, match="non-decreasing"):
        PiecewiseConstant(((1.0, 0.0), (0.5, 1.0)))
    with pytest.raises(ValueError, match="breakpoint"):
        PiecewiseConstant(())


# ------------------------------------------------------------ V/f profile
def test_vf_profile_linear_with_boost() -> None:
    s = VFSettings(base_frequency_hz=60.0, base_voltage_v=208.0, boost_fraction=0.05)
    v_base = 208.0 * math.sqrt(2.0 / 3.0)
    assert vf_voltage_peak(0.0, s) == pytest.approx(0.05 * v_base)
    assert vf_voltage_peak(60.0, s) == pytest.approx(v_base)
    assert vf_voltage_peak(90.0, s) == pytest.approx(v_base)  # field weakening
    assert vf_voltage_peak(30.0, s) == pytest.approx(0.05 * v_base + 0.95 * v_base * 0.5)
    assert vf_voltage_peak(-30.0, s) == vf_voltage_peak(30.0, s)


def test_vf_profile_quadratic() -> None:
    s = VFSettings(profile="quadratic", boost_fraction=0.0)
    v_base = s.base_phase_peak_v
    assert vf_voltage_peak(30.0, s) == pytest.approx(0.25 * v_base)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"base_frequency_hz": 0.0},
        {"profile": "cubic"},
        {"boost_fraction": 1.2},
        {"max_frequency_hz": -1.0},
        {"slip_compensation_gain": -1.0},
        {"slip_filter_time_constant_s": 0.0},
        {"current_limit_a": 0.0},
    ],
)
def test_vf_settings_validation(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):  # noqa: PT011 - several messages
        VFSettings(**kwargs)  # type: ignore[arg-type]


def _meas(t: float, v_dc: float = 285.0) -> Measurements:
    return Measurements(t=t, i_a=0.0, i_b=0.0, i_c=0.0, omega_m=0.0, theta_m=0.0, v_dc=v_dc)


def test_vf_controller_follows_ramp_and_profile(motor: MotorParameters) -> None:
    settings = VFSettings(accel_rate_hz_per_s=20.0, slip_compensation_gain=0.0)
    ctrl = VFController(settings, motor, SinusoidalPWM(), TS, PiecewiseConstant.constant(900.0))
    for k in range(5000):  # 0.5 s -> 10 Hz on the ramp
        v = ctrl.step(_meas(k * TS))
    sig = ctrl.signals()
    assert sig["f_target_hz"] == pytest.approx(30.0)
    assert sig["f_out_hz"] == pytest.approx(10.0, abs=1e-6)
    assert math.hypot(*v) == pytest.approx(vf_voltage_peak(10.0, settings))
    ctrl.reset()
    assert ctrl.signals() == {}


def test_vf_controller_voltage_limited_by_modulator(motor: MotorParameters) -> None:
    settings = VFSettings(accel_rate_hz_per_s=1000.0, slip_compensation_gain=0.0)
    ctrl = VFController(settings, motor, SinusoidalPWM(), TS, PiecewiseConstant.constant(1800.0))
    for k in range(2000):
        v = ctrl.step(_meas(k * TS, v_dc=200.0))
    assert math.hypot(*v) == pytest.approx(100.0)
    over = VFController(
        VFSettings(accel_rate_hz_per_s=1000.0, allow_overmodulation=True),
        motor,
        SinusoidalPWM(),
        TS,
        PiecewiseConstant.constant(1800.0),
    )
    for k in range(2000):
        v = over.step(_meas(k * TS, v_dc=200.0))
    assert math.hypot(*v) == pytest.approx(min(settings.base_phase_peak_v, 400.0 / math.pi))


def test_vf_controller_frequency_limit(motor: MotorParameters) -> None:
    settings = VFSettings(max_frequency_hz=50.0, accel_rate_hz_per_s=1e4)
    ctrl = VFController(settings, motor, SinusoidalPWM(), TS, PiecewiseConstant.constant(3000.0))
    ctrl.step(_meas(0.0))
    assert ctrl.signals()["f_target_hz"] == 50.0


def test_vf_controller_argument_validation(motor: MotorParameters) -> None:
    ref = PiecewiseConstant.constant(0.0)
    with pytest.raises(ValueError, match="sample_time"):
        VFController(VFSettings(), motor, SinusoidalPWM(), 0.0, ref)
    with pytest.raises(ValueError, match="delay"):
        VFController(VFSettings(), motor, SinusoidalPWM(), TS, ref, voltage_delay_samples=-1)


# ------------------------------------------------------ modulation / inverter
@pytest.mark.parametrize("angle", [0.0, 0.4, 1.3, 2.9, -2.0])
def test_spwm_averaged_inverter_reproduces_reference(angle: float) -> None:
    v_dc = 285.0
    ref = (0.9 * v_dc / 2) * math.cos(angle), (0.9 * v_dc / 2) * math.sin(angle)
    duties = SinusoidalPWM().duties(*ref, v_dc)
    assert all(0.0 <= d <= 1.0 for d in duties)
    assert pole_to_alpha_beta(duties, v_dc) == pytest.approx(ref)
    segs = AveragedInverter().segments(duties, TS, (0.0, 0.0, 0.0))
    assert segs == [(TS, duties)]


def test_spwm_clips_in_overmodulation() -> None:
    v_dc = 100.0
    duties = SinusoidalPWM().duties(80.0, 0.0, v_dc)
    assert duties[0] == 1.0  # phase a reference (80 V) exceeds v_dc/2 and is clipped
    assert duties[1] == pytest.approx(0.1)
    va, _ = pole_to_alpha_beta(duties, v_dc)
    assert va < 80.0
    assert SinusoidalPWM().max_phase_peak(v_dc) == 50.0
    assert SinusoidalPWM().duties(1.0, 1.0, 0.0) == (0.5, 0.5, 0.5)


def test_dc_link_current_and_factories() -> None:
    assert dc_link_current((1.0, 0.0, 0.5), (2.0, -1.0, -1.0)) == pytest.approx(1.5)
    assert make_modulator("spwm").name == "spwm"
    assert make_power_stage("averaged").name == "averaged"
    with pytest.raises(ValueError, match="modulation"):
        make_modulator("foo")
    with pytest.raises(ValueError, match="inverter"):
        make_power_stage("foo")
