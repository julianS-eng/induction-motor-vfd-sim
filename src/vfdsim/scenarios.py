"""Factories that assemble complete simulations from a :class:`DriveConfig`."""

from __future__ import annotations

import math
from collections.abc import Callable

from vfdsim.config import DriveConfig
from vfdsim.control.reference import PiecewiseConstant
from vfdsim.control.vf import VFController, VFSettings
from vfdsim.mechanics.loads import CompositeLoad, Load, composite_from_specs
from vfdsim.modulation import make_modulator
from vfdsim.motor.dq_model import InductionMotorModel
from vfdsim.power import make_power_stage
from vfdsim.power.sources import GridSource
from vfdsim.simulation.engine import SensorModel, Simulator
from vfdsim.simulation.plant import DrivePlant
from vfdsim.simulation.results import SimulationResult


def build_load(cfg: DriveConfig) -> CompositeLoad:
    """Composite load described in the configuration."""
    return composite_from_specs(cfg.load_components)


def build_vf_simulator(
    cfg: DriveConfig,
    *,
    load: Load | None = None,
    speed_reference_rpm: Callable[[float], float] | None = None,
    vf_settings: VFSettings | None = None,
) -> Simulator:
    """Inverter-fed motor under V/f control, as described by ``cfg``.

    Keyword arguments override the corresponding parts of the configuration.
    """
    motor_model = InductionMotorModel(cfg.motor)
    plant = DrivePlant(motor_model, load if load is not None else build_load(cfg))
    modulator = make_modulator(cfg.modulation.method)
    ts = cfg.inverter.control_period_s
    reference = speed_reference_rpm or PiecewiseConstant(cfg.speed_reference_rpm)
    controller = VFController(
        vf_settings or cfg.vf_control,
        cfg.motor,
        modulator,
        ts,
        reference,
        voltage_delay_samples=1 if cfg.simulation.computation_delay else 0,
    )
    return Simulator(
        plant,
        control_period=ts,
        controller=controller,
        modulator=modulator,
        power_stage=make_power_stage(cfg.inverter.model),
        v_dc=cfg.dc_link.nominal_voltage_v,
        plant_substeps=cfg.simulation.plant_substeps,
        computation_delay=cfg.simulation.computation_delay,
        sensors=SensorModel(cfg.sensors.current_noise_std_a, cfg.sensors.seed),
        record_every=cfg.simulation.record_every,
    )


def build_dol_simulator(
    cfg: DriveConfig,
    *,
    load: Load | None = None,
    line_voltage_rms: float | None = None,
    frequency_hz: float | None = None,
    locked_speed_rpm: float | None = None,
    time_step: float | None = None,
) -> Simulator:
    """Motor connected direct-on-line to a stiff sinusoidal grid.

    Parameters
    ----------
    locked_speed_rpm:
        Hold the shaft at this speed (dynamometer in speed mode).
    time_step:
        Integration step (defaults to the configured control period).
    """
    np_ = cfg.motor.nameplate
    grid = GridSource(
        line_voltage_rms if line_voltage_rms is not None else np_.rated_voltage_v,
        frequency_hz if frequency_hz is not None else np_.rated_frequency_hz,
    )
    locked = None if locked_speed_rpm is None else locked_speed_rpm * 2.0 * math.pi / 60.0
    plant = DrivePlant(
        InductionMotorModel(cfg.motor),
        load if load is not None else build_load(cfg),
        locked_speed_rad_s=locked,
    )
    return Simulator(
        plant,
        control_period=time_step or cfg.inverter.control_period_s,
        grid=grid,
        plant_substeps=cfg.simulation.plant_substeps,
    )


def run_config(cfg: DriveConfig, t_end: float | None = None) -> SimulationResult:
    """Run the V/f scenario described by ``cfg``."""
    sim = build_vf_simulator(cfg)
    return sim.run(t_end if t_end is not None else cfg.simulation.t_end_s)
