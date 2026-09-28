"""Steady-state model checks: equivalent circuit vs dq phasor solution."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vfdsim.motor import EquivalentCircuit, InductionMotorModel, MotorParameters

V_RATED = 208.0
F_RATED = 60.0


@pytest.mark.parametrize("slip", [-0.3, -0.02, 0.001, 0.0722, 0.2, 0.5, 1.0])
@pytest.mark.parametrize(("v_ll", "freq"), [(208.0, 60.0), (104.0, 30.0), (30.0, 5.0)])
def test_dq_steady_state_equals_equivalent_circuit(
    ec: EquivalentCircuit, dq: InductionMotorModel, slip: float, v_ll: float, freq: float
) -> None:
    op = ec.operating_point(slip, v_ll, freq)
    ss = dq.steady_state(v_ll / math.sqrt(3.0), freq, slip)
    assert ss.torque_nm == pytest.approx(op.torque_nm, rel=1e-9, abs=1e-12)
    assert ss.stator_current_rms_a == pytest.approx(op.stator_current_rms_a, rel=1e-9)
    assert ss.input_power_w == pytest.approx(op.input_power_w, rel=1e-9, abs=1e-9)
    assert ss.reactive_power_var == pytest.approx(op.reactive_power_var, rel=1e-9)
    assert ss.rotor_copper_loss_w == pytest.approx(op.rotor_copper_loss_w, rel=1e-9, abs=1e-12)


def test_operating_point_power_flow(ec: EquivalentCircuit) -> None:
    op = ec.operating_point(0.05, V_RATED, F_RATED)
    assert op.input_power_w == pytest.approx(op.stator_copper_loss_w + op.air_gap_power_w)
    # rotor copper loss = s * P_ag
    assert op.rotor_copper_loss_w == pytest.approx(0.05 * op.air_gap_power_w, rel=1e-9)
    assert op.internal_mechanical_power_w == pytest.approx(0.95 * op.air_gap_power_w, rel=1e-9)
    w_m = op.speed_rpm * 2 * math.pi / 60
    assert op.torque_nm * w_m == pytest.approx(op.internal_mechanical_power_w, rel=1e-9)
    assert 0.0 < op.power_factor < 1.0


def test_torque_sign_and_synchronous_point(ec: EquivalentCircuit) -> None:
    assert float(ec.torque(0.0, V_RATED, F_RATED)) == 0.0
    assert float(ec.torque(0.03, V_RATED, F_RATED)) > 0.0
    assert float(ec.torque(-0.03, V_RATED, F_RATED)) < 0.0  # generating


def test_breakdown_closed_form_matches_numerical_maximum(ec: EquivalentCircuit) -> None:
    s = np.linspace(1e-4, 1.0, 200_001)
    t = ec.torque(s, V_RATED, F_RATED)
    s_max, t_max = ec.breakdown(V_RATED, F_RATED)
    assert t_max == pytest.approx(float(t.max()), rel=1e-6)
    assert s_max == pytest.approx(float(s[np.argmax(t)]), abs=1e-4)


def test_vectorised_current_matches_operating_point(ec: EquivalentCircuit) -> None:
    slips = np.array([0.01, 0.1, 0.7])
    i = ec.stator_current(slips, V_RATED, F_RATED)
    for k, s in enumerate(slips):
        assert i[k] == pytest.approx(ec.operating_point(float(s), V_RATED, F_RATED).stator_current)


def test_slip_for_torque_inverts_torque(ec: EquivalentCircuit) -> None:
    for torque in (0.1, 0.5, 1.0, 2.0):
        s = ec.slip_for_torque(torque, V_RATED, F_RATED)
        assert float(ec.torque(s, V_RATED, F_RATED)) == pytest.approx(torque, rel=1e-9)
    assert ec.slip_for_torque(0.0, V_RATED, F_RATED) == 0.0
    with pytest.raises(ValueError, match="exceeds"):
        ec.slip_for_torque(10.0, V_RATED, F_RATED)


def test_assumed_parameters_reproduce_nameplate_rated_point(
    motor: MotorParameters, ec: EquivalentCircuit
) -> None:
    """The parameter file is self-consistent: rated torque + friction at rated speed."""
    npl = motor.nameplate
    w_r = npl.rated_speed_rad_s
    op = ec.operating_point_with_load(
        lambda w: npl.rated_torque_nm + motor.viscous_friction * w, V_RATED, F_RATED
    )
    assert op.speed_rpm == pytest.approx(npl.rated_speed_rpm, abs=1.0)
    shaft_power = (op.torque_nm - motor.viscous_friction * w_r) * w_r
    assert shaft_power == pytest.approx(npl.rated_power_w, rel=0.01)
    # plausibility of a small educational motor
    assert 0.8 < op.stator_current_rms_a / npl.rated_current_a < 1.1
    _, t_max = ec.breakdown(V_RATED, F_RATED)
    assert 2.0 < t_max / npl.rated_torque_nm < 3.5


def test_stall_is_reported(ec: EquivalentCircuit) -> None:
    with pytest.raises(ValueError, match="stalls"):
        ec.operating_point_with_load(lambda w: 5.0, V_RATED, F_RATED)


def test_steady_state_requires_frequency(dq: InductionMotorModel) -> None:
    with pytest.raises(ValueError, match="non-zero"):
        dq.steady_state(120.0, 0.0, 0.1)
