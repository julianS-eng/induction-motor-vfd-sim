from __future__ import annotations

import math

import pytest

from vfdsim.mechanics import (
    CompositeLoad,
    ConstantTorqueLoad,
    FrictionLoad,
    InertiaLoad,
    Load,
    QuadraticLoad,
    StepLoad,
    composite_from_specs,
    load_from_spec,
    smooth_sign,
)


def test_smooth_sign() -> None:
    assert smooth_sign(0.0) == 0.0
    assert smooth_sign(50.0) == pytest.approx(1.0)
    assert smooth_sign(-50.0) == pytest.approx(-1.0)


def test_inertia_load() -> None:
    ld = InertiaLoad(0.01)
    assert ld.inertia == 0.01
    assert ld.torque(0.0, 100.0) == 0.0
    with pytest.raises(ValueError, match="non-negative"):
        InertiaLoad(-1.0)


def test_friction_load() -> None:
    ld = FrictionLoad(viscous_nm_s_per_rad=0.01, coulomb_nm=0.1)
    assert ld.torque(0.0, 100.0) == pytest.approx(1.0 + 0.1)
    assert ld.torque(0.0, -100.0) == pytest.approx(-1.1)
    assert ld.torque(0.0, 0.0) == 0.0


def test_constant_passive_and_active() -> None:
    passive = ConstantTorqueLoad(2.0)
    active = ConstantTorqueLoad(2.0, passive=False)
    assert passive.torque(0.0, 100.0) == pytest.approx(2.0)
    assert passive.torque(0.0, -100.0) == pytest.approx(-2.0)
    assert active.torque(0.0, -100.0) == 2.0
    assert active.torque(0.0, 0.0) == 2.0


def test_quadratic_load() -> None:
    ld = QuadraticLoad(torque_at_speed_nm=1.0, speed_rpm=1500.0)
    w = 1500.0 * 2 * math.pi / 60
    assert ld.torque(0.0, w) == pytest.approx(1.0)
    assert ld.torque(0.0, w / 2) == pytest.approx(0.25)
    assert ld.torque(0.0, -w) == pytest.approx(-1.0)
    with pytest.raises(ValueError, match="positive"):
        QuadraticLoad(1.0, 0.0)


def test_step_load() -> None:
    ld = StepLoad(time_s=1.0, torque_after_nm=1.5, torque_before_nm=0.2)
    assert ld.torque(0.99, 100.0) == pytest.approx(0.2)
    assert ld.torque(1.0, 100.0) == pytest.approx(1.5)


def test_composite_and_specs() -> None:
    specs = [
        {"type": "inertia", "inertia_kgm2": 0.002},
        {"type": "friction", "viscous_nm_s_per_rad": 0.001, "coulomb_nm": 0.0},
        {"type": "constant", "torque_nm": 0.5},
        {"type": "quadratic", "torque_at_speed_nm": 0.4, "speed_rpm": 1000.0},
        {"type": "step", "time_s": 2.0, "torque_after_nm": 1.0},
    ]
    load = composite_from_specs(specs)
    assert isinstance(load, Load)
    assert load.inertia == pytest.approx(0.002)
    w = 1000.0 * 2 * math.pi / 60
    assert load.torque(0.0, w) == pytest.approx(0.001 * w + 0.5 + 0.4)
    assert load.torque(3.0, w) == pytest.approx(0.001 * w + 0.5 + 0.4 + 1.0)
    extended = load.with_component(InertiaLoad(0.001))
    assert extended.inertia == pytest.approx(0.003)
    assert CompositeLoad().torque(0.0, 10.0) == 0.0


def test_spec_errors() -> None:
    with pytest.raises(ValueError, match="unknown load type"):
        load_from_spec({"type": "magic"})
    with pytest.raises(ValueError, match="unknown keys"):
        load_from_spec({"type": "constant", "torque": 1.0})
