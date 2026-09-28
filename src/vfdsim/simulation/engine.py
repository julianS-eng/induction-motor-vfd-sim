"""Fixed-step simulation engine (software-in-the-loop style).

The engine reproduces the timing of a digital drive:

1. At each sampling instant ``t_k = k T_s`` the sensors are read.
2. The controller computes a voltage reference and the modulator turns it
   into duty cycles.
3. With ``computation_delay`` enabled the new duties take effect at
   ``t_{k+1}`` (the usual one-sample delay of an MCU that updates its PWM
   registers at the next carrier period); otherwise they are applied at once.
4. The power stage splits the period into segments of constant pole state
   and the continuous plant is integrated through them with RK4.

Alternatively the motor can be fed directly from a :class:`GridSource`
(direct-on-line operation), in which case no controller is involved.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from vfdsim.control.base import Controller, Measurements
from vfdsim.modulation.base import Duties, Modulator
from vfdsim.power.inverter import PowerStage, pole_to_alpha_beta
from vfdsim.power.sources import GridSource
from vfdsim.simulation.integrators import rk4_step
from vfdsim.simulation.plant import (
    IDX_E_CU_R,
    IDX_E_CU_S,
    IDX_E_EM,
    IDX_E_FRIC,
    IDX_E_IN,
    IDX_E_LOAD,
    IDX_OMEGA,
    IDX_THETA,
    DrivePlant,
)
from vfdsim.simulation.results import SimulationResult


@dataclass(frozen=True)
class SensorModel:
    """Measurement chain model.

    Attributes
    ----------
    current_noise_std_a:
        Standard deviation of white Gaussian noise added to each phase current.
    seed:
        Seed of the random generator (results are reproducible).
    """

    current_noise_std_a: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        if self.current_noise_std_a < 0.0:
            raise ValueError("current_noise_std_a must be non-negative")


_BASE_CHANNELS = (
    "t",
    "i_a",
    "i_b",
    "i_c",
    "i_alpha",
    "i_beta",
    "v_alpha",
    "v_beta",
    "omega_m",
    "theta_m",
    "torque_e",
    "torque_load",
    "torque_friction",
    "psi_s",
    "psi_r",
    "v_dc",
    "d_a",
    "d_b",
    "d_c",
    "e_in",
    "e_cu_s",
    "e_cu_r",
    "e_em",
    "e_load",
    "e_fric",
)


class Simulator:
    """Couples a :class:`DrivePlant` with a digital controller or a grid.

    Parameters
    ----------
    plant:
        Motor + load.
    control_period:
        Sampling / PWM-update period ``T_s`` in s.
    controller, modulator, power_stage:
        Inverter-fed operation (all three required together).
    v_dc:
        DC-link voltage (ideal, stiff bus) in V.
    grid:
        Direct-on-line operation (mutually exclusive with ``controller``).
    plant_substeps:
        RK4 steps per constant-input segment.
    computation_delay:
        Apply the duty cycles one sample after they are computed.
    sensors:
        Measurement noise model.
    record_every:
        Keep one sample out of ``record_every``.
    """

    def __init__(
        self,
        plant: DrivePlant,
        *,
        control_period: float,
        controller: Controller | None = None,
        modulator: Modulator | None = None,
        power_stage: PowerStage | None = None,
        v_dc: float = 0.0,
        grid: GridSource | None = None,
        plant_substeps: int = 1,
        computation_delay: bool = True,
        sensors: SensorModel | None = None,
        record_every: int = 1,
    ) -> None:
        inverter_fed = controller is not None
        if inverter_fed == (grid is not None):
            raise ValueError("provide either a controller (inverter-fed) or a grid (DOL)")
        if inverter_fed and (modulator is None or power_stage is None):
            raise ValueError("inverter-fed operation requires a modulator and a power stage")
        if inverter_fed and v_dc <= 0.0:
            raise ValueError("v_dc must be positive for inverter-fed operation")
        if control_period <= 0.0:
            raise ValueError("control_period must be positive")
        if plant_substeps < 1 or record_every < 1:
            raise ValueError("plant_substeps and record_every must be >= 1")
        self.plant = plant
        self.control_period = control_period
        self.controller = controller
        self.modulator = modulator
        self.power_stage = power_stage
        self.v_dc = v_dc
        self.grid = grid
        self.plant_substeps = plant_substeps
        self.computation_delay = computation_delay
        self.sensors = sensors or SensorModel()
        self.record_every = record_every

    # ------------------------------------------------------------------ run
    def run(self, t_end: float, x0: Sequence[float] | None = None) -> SimulationResult:
        """Simulate from ``t = 0`` to ``t_end`` and return the recorded signals."""
        ts = self.control_period
        n_steps = round(t_end / ts)
        if n_steps < 1:
            raise ValueError("t_end must be at least one control period")
        plant = self.plant
        x = list(plant.initial_state() if x0 is None else x0)
        rng = np.random.default_rng(self.sensors.seed)
        noise = (
            rng.standard_normal((n_steps, 3)) * self.sensors.current_noise_std_a
            if self.sensors.current_noise_std_a > 0.0
            else None
        )
        if self.controller is not None:
            self.controller.reset()

        rows: list[tuple[float, ...]] = []
        ctrl_names: list[str] | None = None
        ctrl_rows: list[tuple[float, ...]] = []
        duties_pending: Duties = (0.5, 0.5, 0.5)
        n_sub = self.plant_substeps
        wall_start = time.perf_counter()

        for k in range(n_steps):
            t = k * ts
            x_k = x  # integrators return new lists, so x_k keeps the sampled state
            out = plant.outputs(t, x)
            i_abc = out.currents_abc
            v_dc = self.v_dc

            if self.controller is not None:
                assert self.modulator is not None
                assert self.power_stage is not None
                if noise is not None:
                    n = noise[k]
                    i_meas = (i_abc[0] + n[0], i_abc[1] + n[1], i_abc[2] + n[2])
                else:
                    i_meas = i_abc
                meas = Measurements(
                    t=t,
                    i_a=i_meas[0],
                    i_b=i_meas[1],
                    i_c=i_meas[2],
                    omega_m=x[IDX_OMEGA],
                    theta_m=x[IDX_THETA],
                    v_dc=v_dc,
                )
                v_ref = self.controller.step(meas)
                duties_new = self.modulator.duties(v_ref[0], v_ref[1], v_dc)
                if self.computation_delay:
                    duties, duties_pending = duties_pending, duties_new
                else:
                    duties = duties_new
                segments = self.power_stage.segments(duties, ts, i_abc)
                t_seg = t
                va_avg = vb_avg = 0.0
                for dt_seg, poles in segments:
                    if dt_seg <= 0.0:
                        continue
                    va, vb = pole_to_alpha_beta(poles, v_dc)
                    va_avg += va * dt_seg
                    vb_avg += vb * dt_seg

                    def f_seg(
                        tt: float, xx: Sequence[float], va: float = va, vb: float = vb
                    ) -> list[float]:
                        return plant.derivatives(tt, xx, va, vb)

                    h = dt_seg / n_sub
                    for _ in range(n_sub):
                        x = rk4_step(f_seg, t_seg, x, h)
                        t_seg += h
                va_avg /= ts
                vb_avg /= ts
            else:
                assert self.grid is not None
                grid = self.grid
                duties = (math.nan, math.nan, math.nan)
                va_avg, vb_avg = grid.alpha_beta(t)

                def f_grid(tt: float, xx: Sequence[float], grid: GridSource = grid) -> list[float]:
                    va, vb = grid.alpha_beta(tt)
                    return plant.derivatives(tt, xx, va, vb)

                h = ts / n_sub
                t_seg = t
                for _ in range(n_sub):
                    x = rk4_step(f_grid, t_seg, x, h)
                    t_seg += h

            if k % self.record_every:
                continue
            rows.append(
                (
                    t,
                    i_abc[0],
                    i_abc[1],
                    i_abc[2],
                    out.i_sa,
                    out.i_sb,
                    va_avg,
                    vb_avg,
                    x_k[IDX_OMEGA],
                    x_k[IDX_THETA],
                    out.torque_e,
                    out.torque_load,
                    out.torque_friction,
                    out.psi_s,
                    out.psi_r,
                    v_dc,
                    duties[0],
                    duties[1],
                    duties[2],
                    x_k[IDX_E_IN],
                    x_k[IDX_E_CU_S],
                    x_k[IDX_E_CU_R],
                    x_k[IDX_E_EM],
                    x_k[IDX_E_LOAD],
                    x_k[IDX_E_FRIC],
                )
            )
            if self.controller is not None:
                sig = self.controller.signals()
                if ctrl_names is None:
                    ctrl_names = list(sig)
                ctrl_rows.append(tuple(sig[name] for name in ctrl_names))

        result = self._pack(rows, ctrl_names, ctrl_rows, time.perf_counter() - wall_start)
        result.metadata["final_time_s"] = n_steps * ts
        result.metadata["final_state"] = list(x)
        return result

    def _pack(
        self,
        rows: list[tuple[float, ...]],
        ctrl_names: list[str] | None,
        ctrl_rows: list[tuple[float, ...]],
        wall_time: float,
    ) -> SimulationResult:
        data = np.asarray(rows, dtype=np.float64)
        signals = {name: np.ascontiguousarray(data[:, i]) for i, name in enumerate(_BASE_CHANNELS)}
        if ctrl_names:
            cdata = np.asarray(ctrl_rows, dtype=np.float64)
            for i, name in enumerate(ctrl_names):
                signals[f"ctrl.{name}"] = np.ascontiguousarray(cdata[:, i])
        metadata = {
            "control_period_s": self.control_period,
            "plant_substeps": self.plant_substeps,
            "computation_delay": self.computation_delay,
            "record_every": self.record_every,
            "mode": "inverter" if self.controller is not None else "grid",
            "wall_time_s": wall_time,
        }
        return SimulationResult(signals=signals, metadata=metadata)
