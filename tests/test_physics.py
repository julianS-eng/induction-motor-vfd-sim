"""Physics validation of the Phase 1 plant and V/f drive (time-domain simulations).

These tests check the simulator against first principles and against the
independent steady-state equivalent circuit:

* the rotor turns at the synchronous speed ``120 f / P`` without load;
* the steady-state slip under load is in the expected range and equals the one
  predicted by the equivalent circuit;
* the power / energy balance closes;
* the torque–speed characteristic measured on the dq model equals the
  equivalent-circuit curve.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vfdsim.config import DriveConfig
from vfdsim.control import PiecewiseConstant, VFSettings
from vfdsim.mechanics import CompositeLoad, ConstantTorqueLoad, InertiaLoad, Load
from vfdsim.motor import EquivalentCircuit, MotorParameters
from vfdsim.scenarios import build_dol_simulator, build_vf_simulator
from vfdsim.simulation import SensorModel, SimulationResult, Simulator

RPM = 60.0 / (2.0 * math.pi)


def _vf(cfg: DriveConfig, **changes: object) -> VFSettings:
    base = dataclasses.replace(cfg.vf_control, accel_rate_hz_per_s=100.0, decel_rate_hz_per_s=100.0)
    return dataclasses.replace(base, **changes)  # type: ignore[arg-type]


def _run_vf(
    cfg: DriveConfig,
    *,
    speed_rpm: float,
    load: Load,
    t_end: float,
    motor: MotorParameters | None = None,
    **vf_changes: object,
) -> SimulationResult:
    c = cfg if motor is None else dataclasses.replace(cfg, motor=motor)
    sim = build_vf_simulator(
        c,
        load=load,
        speed_reference_rpm=PiecewiseConstant.constant(speed_rpm),
        vf_settings=_vf(c, **vf_changes),
    )
    return sim.run(t_end)


def _applied_line_voltage(res: SimulationResult, t0: float) -> float:
    """Fundamental line-to-line RMS voltage of the averaged inverter output."""
    mask = res.window(t0)
    v_peak = float(np.mean(np.hypot(res["v_alpha"][mask], res["v_beta"][mask])))
    return v_peak * math.sqrt(3.0 / 2.0)


# ------------------------------------------------------------------ 120 f / P
@pytest.mark.parametrize("poles", [2, 4, 6])
@pytest.mark.parametrize("freq", [20.0, 45.0])
def test_no_load_speed_equals_synchronous_speed(
    cfg: DriveConfig, frictionless_motor: MotorParameters, poles: int, freq: float
) -> None:
    npl = frictionless_motor.nameplate
    sync_rated = 120.0 * npl.rated_frequency_hz / poles
    nameplate = dataclasses.replace(
        npl, poles=poles, rated_speed_rpm=sync_rated * (1.0 - npl.rated_slip)
    )
    motor = dataclasses.replace(frictionless_motor, nameplate=nameplate)
    res = _run_vf(
        cfg,
        motor=motor,
        speed_rpm=120.0 * freq / poles,
        load=CompositeLoad((InertiaLoad(1e-3),)),
        t_end=1.5,
        slip_compensation_gain=0.0,
    )
    f_out = res.mean("ctrl.f_out_hz", 1.2)
    assert f_out == pytest.approx(freq, rel=1e-9)
    speed = float(np.mean(res.speed_rpm[res.window(1.2)]))
    assert speed == pytest.approx(120.0 * f_out / poles, rel=5e-4)


# --------------------------------------------------------- slip under load
@pytest.mark.parametrize("freq", [30.0, 60.0])
def test_loaded_slip_in_range_and_matches_equivalent_circuit(
    cfg: DriveConfig, motor: MotorParameters, ec: EquivalentCircuit, freq: float
) -> None:
    t_load = motor.nameplate.rated_torque_nm
    res = _run_vf(
        cfg,
        speed_rpm=30.0 * freq,
        load=CompositeLoad((InertiaLoad(1e-3), ConstantTorqueLoad(t_load))),
        t_end=2.0,
        slip_compensation_gain=0.0,
    )
    t0 = 1.6
    f_out = res.mean("ctrl.f_out_hz", t0)
    n_sync = 120.0 * f_out / motor.nameplate.poles
    speed = float(np.mean(res.speed_rpm[res.window(t0)]))
    slip = (n_sync - speed) / n_sync
    assert 0.01 < slip < 0.15

    v_ll = _applied_line_voltage(res, t0)
    op = ec.operating_point_with_load(
        lambda w: t_load * math.tanh(w / 0.5) + motor.viscous_friction * w, v_ll, f_out
    )
    assert slip == pytest.approx(op.slip, rel=5e-3)
    torque = res.mean("torque_e", t0)
    assert torque == pytest.approx(op.torque_nm, rel=2e-3)


def test_dol_rated_load_reaches_nameplate_speed(cfg: DriveConfig, motor: MotorParameters) -> None:
    load = CompositeLoad((InertiaLoad(1e-3), ConstantTorqueLoad(motor.nameplate.rated_torque_nm)))
    res = build_dol_simulator(cfg, load=load).run(1.5)
    expected = motor.nameplate.rated_speed_rpm
    assert float(np.mean(res.speed_rpm[res.window(1.2)])) == pytest.approx(expected, abs=2.0)
    # direct-on-line start draws several times rated current
    assert float(np.max(np.abs(res["i_a"]))) > 3.0 * motor.nameplate.rated_current_a


# ------------------------------------------------------------ power balance
def test_steady_state_power_balance(
    cfg: DriveConfig, motor: MotorParameters, ec: EquivalentCircuit
) -> None:
    t_load = motor.nameplate.rated_torque_nm
    res = _run_vf(
        cfg,
        speed_rpm=1350.0,
        load=CompositeLoad((InertiaLoad(1e-3), ConstantTorqueLoad(t_load))),
        t_end=2.0,
        slip_compensation_gain=0.0,
    )
    t0, t1 = 1.5, 2.0 - 1e-4
    p_in = res.average_power("e_in", t0, t1)
    p_cu_s = res.average_power("e_cu_s", t0, t1)
    p_cu_r = res.average_power("e_cu_r", t0, t1)
    p_fric = res.average_power("e_fric", t0, t1)
    p_load = res.average_power("e_load", t0, t1)
    p_em = res.average_power("e_em", t0, t1)
    assert p_in == pytest.approx(p_cu_s + p_cu_r + p_fric + p_load, rel=2e-3)
    assert p_em == pytest.approx(p_fric + p_load, rel=2e-3)
    # rotor copper loss = s * P_airgap
    f_out = res.mean("ctrl.f_out_hz", t0)
    speed = float(np.mean(res.speed_rpm[res.window(t0)]))
    slip = 1.0 - speed / (30.0 * f_out)
    assert p_cu_r == pytest.approx(slip * (p_in - p_cu_s), rel=5e-3)
    # and agrees with the equivalent circuit at the same operating point
    op = ec.operating_point(slip, _applied_line_voltage(res, t0), f_out)
    assert p_in == pytest.approx(op.input_power_w, rel=5e-3)
    assert p_cu_s == pytest.approx(op.stator_copper_loss_w, rel=5e-3)


def test_energy_is_conserved_during_transients(cfg: DriveConfig, motor: MotorParameters) -> None:
    sim = build_vf_simulator(
        cfg,
        load=CompositeLoad((InertiaLoad(0.01), ConstantTorqueLoad(0.1))),
        speed_reference_rpm=PiecewiseConstant(((0.0, 1500.0), (1.5, 300.0))),
    )
    res = sim.run(2.2)
    x = res.metadata["final_state"]
    w_mag, w_kin = sim.plant.stored_energy(x)
    e_in, e_cus, e_cur, e_em, e_load, e_fric = x[6:12]
    assert e_in == pytest.approx(e_cus + e_cur + e_em + w_mag, rel=1e-6)
    assert e_em == pytest.approx(e_load + e_fric + w_kin, rel=1e-6)
    # regenerative braking during the deceleration: negative torque and power
    decel = res.window(1.5, 2.2)
    assert float(np.min(res["torque_e"][decel])) < 0.0
    assert float(np.min(np.diff(res["e_in"][decel]))) < 0.0


# ------------------------------------------------ torque–speed characteristic
@pytest.mark.parametrize("speed_rpm", [0.0, 600.0, 1250.0, 1550.0, 1700.0, 1790.0, 1860.0])
def test_torque_speed_curve_matches_equivalent_circuit(
    cfg: DriveConfig, ec: EquivalentCircuit, speed_rpm: float
) -> None:
    sim = build_dol_simulator(cfg, load=CompositeLoad(), locked_speed_rpm=speed_rpm)
    res = sim.run(0.8)
    torque = res.mean("torque_e", 0.8 - 10.0 / 60.0)
    expected = float(ec.torque((1800.0 - speed_rpm) / 1800.0, 208.0, 60.0))
    assert torque == pytest.approx(expected, rel=1e-3, abs=1e-4)


@pytest.mark.parametrize("speed_rpm", [300.0, 800.0, 880.0])
def test_torque_speed_curve_at_reduced_frequency(
    cfg: DriveConfig, ec: EquivalentCircuit, speed_rpm: float
) -> None:
    sim = build_dol_simulator(
        cfg,
        load=CompositeLoad(),
        line_voltage_rms=104.0,
        frequency_hz=30.0,
        locked_speed_rpm=speed_rpm,
    )
    res = sim.run(0.8)
    torque = res.mean("torque_e", 0.8 - 5.0 / 30.0)
    expected = float(ec.torque((900.0 - speed_rpm) / 900.0, 104.0, 30.0))
    assert torque == pytest.approx(expected, rel=1e-3, abs=1e-4)


# ------------------------------------------------------- V/f drive features
def test_slip_compensation_reduces_speed_error(cfg: DriveConfig, motor: MotorParameters) -> None:
    load = CompositeLoad((InertiaLoad(1e-3), ConstantTorqueLoad(motor.nameplate.rated_torque_nm)))
    errors = {}
    for gain in (0.0, 1.0):
        res = _run_vf(cfg, speed_rpm=900.0, load=load, t_end=2.0, slip_compensation_gain=gain)
        speed = float(np.mean(res.speed_rpm[res.window(1.6)]))
        errors[gain] = abs(speed - 900.0) / 900.0
    assert errors[0.0] > 0.05
    assert errors[1.0] < 0.015


def test_boost_prevents_stall_at_low_frequency(cfg: DriveConfig, motor: MotorParameters) -> None:
    load = CompositeLoad(
        (InertiaLoad(1e-3), ConstantTorqueLoad(0.3 * motor.nameplate.rated_torque_nm))
    )
    speeds = {}
    for boost in (0.0, 0.12):
        res = _run_vf(
            cfg,
            speed_rpm=90.0,  # 3 Hz
            load=load,
            t_end=1.5,
            boost_fraction=boost,
            slip_compensation_gain=0.0,
        )
        speeds[boost] = float(np.mean(res.speed_rpm[res.window(1.2)]))
    assert speeds[0.0] < 0.2 * 90.0
    assert speeds[0.12] > 0.5 * 90.0


def test_current_limit_holds_the_ramp(cfg: DriveConfig) -> None:
    load = CompositeLoad((InertiaLoad(0.03), ConstantTorqueLoad(0.3)))
    limit = 1.6
    runs = {}
    for lim in (None, limit):
        runs[lim] = _run_vf(
            cfg,
            speed_rpm=1500.0,
            load=load,
            t_end=1.0,
            accel_rate_hz_per_s=200.0,
            current_limit_a=lim,
        )
    limited = runs[limit]
    assert float(np.max(limited["ctrl.ramp_hold"])) == 1.0
    i_filt = limited["ctrl.i_rms_filtered_a"]
    # after the initial transient the current is regulated around the limit
    assert float(np.max(i_filt[limited.window(0.3)])) < 1.1 * limit
    assert float(np.max(runs[None]["ctrl.i_rms_filtered_a"])) > 1.5 * float(np.max(i_filt))


def test_averaged_inverter_applies_the_vf_voltage(cfg: DriveConfig) -> None:
    res = _run_vf(
        cfg,
        speed_rpm=900.0,
        load=CompositeLoad((InertiaLoad(1e-3),)),
        t_end=0.6,
        slip_compensation_gain=0.0,
    )
    mask = res.window(0.4)
    v_mag = np.hypot(res["v_alpha"][mask], res["v_beta"][mask])
    np.testing.assert_allclose(v_mag, res["ctrl.v_cmd_peak_v"][mask], rtol=1e-9)


# ------------------------------------------------------ engine behaviour
def test_results_are_reproducible_with_seeded_noise(cfg: DriveConfig) -> None:
    def run(seed: int) -> SimulationResult:
        c = dataclasses.replace(
            cfg, sensors=dataclasses.replace(cfg.sensors, current_noise_std_a=0.02, seed=seed)
        )
        return build_vf_simulator(c).run(0.2)

    a, b, c = run(7), run(7), run(8)
    np.testing.assert_array_equal(a["omega_m"], b["omega_m"])
    assert not np.array_equal(a["omega_m"], c["omega_m"])


def test_record_decimation_and_metadata(cfg: DriveConfig) -> None:
    c = dataclasses.replace(cfg, simulation=dataclasses.replace(cfg.simulation, record_every=10))
    res = build_vf_simulator(c).run(0.1)
    assert len(res.t) == 100
    assert res.metadata["mode"] == "inverter"
    assert res.metadata["final_time_s"] == pytest.approx(0.1)
    assert "ctrl.f_out_hz" in res
    assert set(res) >= {"t", "omega_m", "torque_e"}


def test_simulator_argument_validation(cfg: DriveConfig) -> None:
    sim = build_dol_simulator(cfg)
    with pytest.raises(ValueError, match="either"):
        Simulator(sim.plant, control_period=1e-4)
    with pytest.raises(ValueError, match="control_period"):
        Simulator(sim.plant, control_period=0.0, grid=sim.grid)
    with pytest.raises(ValueError, match="substeps"):
        Simulator(sim.plant, control_period=1e-4, grid=sim.grid, plant_substeps=0)
    with pytest.raises(ValueError, match="t_end"):
        sim.run(0.0)
    with pytest.raises(ValueError, match="non-negative"):
        SensorModel(current_noise_std_a=-1.0)
