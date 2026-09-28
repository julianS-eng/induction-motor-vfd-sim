from __future__ import annotations

import math
from collections.abc import Sequence

import pytest

from vfdsim.config import DriveConfig
from vfdsim.mechanics import CompositeLoad
from vfdsim.motor import EquivalentCircuit, InductionMotorModel
from vfdsim.scenarios import build_dol_simulator
from vfdsim.simulation.integrators import integrate, rk4_step


def _decay(t: float, x: Sequence[float]) -> list[float]:
    return [-x[0], -2.0 * x[1]]


def test_rk4_fourth_order_convergence() -> None:
    errors = []
    for n in (10, 20):
        y = integrate(_decay, 0.0, [1.0, 1.0], 1.0, n)
        errors.append(abs(y[0] - math.exp(-1.0)))
    assert errors[0] / errors[1] == pytest.approx(16.0, rel=0.1)
    assert rk4_step(_decay, 0.0, [1.0, 0.0], 0.0) == [1.0, 0.0]
    with pytest.raises(ValueError, match="n_steps"):
        integrate(_decay, 0.0, [1.0, 1.0], 1.0, 0)


def test_simulation_started_in_steady_state_has_no_transient(
    cfg: DriveConfig, dq: InductionMotorModel, ec: EquivalentCircuit
) -> None:
    speed_rpm = 1650.0
    slip = (1800.0 - speed_rpm) / 1800.0
    sim = build_dol_simulator(cfg, load=CompositeLoad(), locked_speed_rpm=speed_rpm)
    fluxes = dq.steady_state_initial_fluxes(208.0 / math.sqrt(3.0), 60.0, slip, angle=0.0)
    x0 = sim.plant.initial_state(fluxes=fluxes)
    res = sim.run(0.02, x0=x0)
    expected = float(ec.torque(slip, 208.0, 60.0))
    assert res["torque_e"].min() == pytest.approx(expected, rel=1e-6)
    assert res["torque_e"].max() == pytest.approx(expected, rel=1e-6)
    assert res.delta("e_in", 0.0, 0.0199) > 0.0
