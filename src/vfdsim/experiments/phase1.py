"""Phase 1 experiments: plant validation and scalar V/f control.

Run with ``python -m vfdsim.experiments.phase1`` (or ``vfdsim experiments
phase1``). Every figure in ``docs/img/phase1_*.png`` and every number in
``docs/results/phase1.md`` / ``phase1_metrics.json`` is produced here.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from vfdsim.config import DriveConfig, default_config
from vfdsim.control import PiecewiseConstant, VFSettings, vf_voltage_peak
from vfdsim.mechanics import CompositeLoad, ConstantTorqueLoad, InertiaLoad, Load, StepLoad
from vfdsim.motor import EquivalentCircuit
from vfdsim.plotting import (
    SERIES,
    TEXT_SECONDARY,
    apply_style,
    marker_kwargs,
    reference_line_kwargs,
    save,
)
from vfdsim.scenarios import build_dol_simulator, build_vf_simulator
from vfdsim.simulation import SimulationResult

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)
REPO_ROOT = Path(__file__).resolve().parents[3]


@dataclass
class ExperimentReport:
    """Figures, metrics and Markdown produced by an experiment script."""

    metrics: dict[str, Any] = field(default_factory=dict)
    figures: list[Path] = field(default_factory=list)
    markdown: list[str] = field(default_factory=list)


def _fmt(x: float, nd: int = 2) -> str:
    return f"{x:,.{nd}f}"


def _table(header: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


def _mean_speed(res: SimulationResult, t0: float, t1: float | None = None) -> float:
    return float(np.mean(res.speed_rpm[res.window(t0, t1)]))


def _rms(x: np.ndarray[Any, Any]) -> float:
    return float(np.sqrt(np.mean(np.square(x))))


def _vf(cfg: DriveConfig, **changes: Any) -> VFSettings:
    return dataclasses.replace(cfg.vf_control, **changes)


def _available_line_voltage(cfg: DriveConfig, freq: float, settings: VFSettings) -> float:
    """Fundamental line RMS voltage the V/f drive applies at ``freq`` (linear SPWM)."""
    v_peak = min(vf_voltage_peak(freq, settings), 0.5 * cfg.dc_link.nominal_voltage_v)
    return v_peak * math.sqrt(1.5)


# ------------------------------------------------------------------ E1 + E2
def rated_point_and_torque_speed(
    cfg: DriveConfig, report: ExperimentReport, img_dir: Path, quick: bool
) -> None:
    """Equivalent-circuit rated point and dq-vs-circuit torque–speed validation."""
    m = cfg.motor
    npl = m.nameplate
    ec = EquivalentCircuit(m)
    v, f = npl.rated_voltage_v, npl.rated_frequency_hz

    op = ec.operating_point_with_load(lambda w: npl.rated_torque_nm + m.viscous_friction * w, v, f)
    s_max, t_max = ec.breakdown(v, f)
    start = ec.operating_point(1.0, v, f)
    no_load = ec.operating_point(0.0, v, f)
    w = op.speed_rpm / RPM_PER_RAD_S
    shaft = (op.torque_nm - m.viscous_friction * w) * w
    rated = {
        "speed_rpm": op.speed_rpm,
        "slip": op.slip,
        "current_a": op.stator_current_rms_a,
        "power_factor": op.power_factor,
        "input_power_w": op.input_power_w,
        "shaft_power_w": shaft,
        "efficiency_no_core_loss": shaft / op.input_power_w,
        "torque_nm": op.torque_nm,
        "breakdown_torque_nm": t_max,
        "breakdown_slip": s_max,
        "starting_current_a": start.stator_current_rms_a,
        "starting_torque_nm": start.torque_nm,
        "no_load_current_a": no_load.stator_current_rms_a,
    }
    report.metrics["rated_point"] = rated
    report.markdown.append(
        f"#### Motor {m.name}: rated operating point (equivalent circuit, 208 V / 60 Hz)\n\n"
        + _table(
            ["Quantity", "Value", f"Rating plate ({m.provenance.get('nameplate', 'unknown')})"],
            [
                [
                    "Speed",
                    f"{_fmt(op.speed_rpm, 1)} r/min",
                    f"{_fmt(npl.rated_speed_rpm, 0)} r/min",
                ],
                ["Slip", f"{_fmt(100 * op.slip, 2)} %", f"{_fmt(100 * npl.rated_slip, 2)} %"],
                ["Shaft power", f"{_fmt(shaft, 1)} W", f"{_fmt(npl.rated_power_w, 0)} W"],
                [
                    "Line current",
                    f"{_fmt(op.stator_current_rms_a, 3)} A",
                    f"{npl.rated_current_a} A",
                ],
                ["Power factor", _fmt(op.power_factor, 3), "-"],
                ["Efficiency (no core loss)", f"{_fmt(100 * shaft / op.input_power_w, 1)} %", "-"],
                ["No-load current", f"{_fmt(no_load.stator_current_rms_a, 3)} A", "-"],
                [
                    "Breakdown torque / slip",
                    f"{_fmt(t_max, 3)} N·m ({_fmt(t_max / npl.rated_torque_nm, 2)}× rated)"
                    f" at s = {_fmt(s_max, 3)}",
                    "-",
                ],
                [
                    "Locked-rotor current / torque",
                    f"{_fmt(start.stator_current_rms_a, 2)} A / {_fmt(start.torque_nm, 3)} N·m",
                    "-",
                ],
            ],
        )
    )

    # dq model on a speed-controlled dynamometer vs the equivalent circuit
    speeds = [0.0, 300.0, 600.0, 900.0, 1200.0, 1350.0, 1500.0, 1600.0, 1700.0, 1760.0, 1800.0]
    speeds += [1850.0]
    if quick:
        speeds = speeds[::3]
    t_sim = 0.8
    dq_torque, dq_current = [], []
    for n in speeds:
        res = build_dol_simulator(cfg, load=CompositeLoad(), locked_speed_rpm=n).run(t_sim)
        win = res.window(t_sim - 10.0 / f)
        dq_torque.append(float(np.mean(res["torque_e"][win])))
        dq_current.append(_rms(res["i_a"][win]))
    slips = [(ec.synchronous_speed_rpm(f) - n) / ec.synchronous_speed_rpm(f) for n in speeds]
    ec_torque = ec.torque(slips, v, f)
    ec_current = np.abs(ec.stator_current(slips, v, f))
    err_t = np.abs(np.asarray(dq_torque) - ec_torque)
    err_i = np.abs(np.asarray(dq_current) - ec_current) / ec_current
    report.metrics["torque_speed_validation"] = {
        "points": len(speeds),
        "max_abs_torque_error_nm": float(err_t.max()),
        "max_rel_torque_error": float(
            (err_t / np.maximum(np.abs(ec_torque), 1e-9))[ec_torque != 0].max()
        ),
        "max_rel_current_error": float(err_i.max()),
    }

    n_curve, t_curve, i_curve = ec.torque_speed_curve(v, f, slip_range=(1.0, -0.06))
    apply_style()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.5, 3.8))
    ax1.plot(n_curve, t_curve, color=SERIES[0], label="Equivalent circuit")
    ax1.plot(speeds, dq_torque, **marker_kwargs(SERIES[1]), label="dq model (locked speed)")
    ax1.axhline(npl.rated_torque_nm, **reference_line_kwargs())
    ax1.text(20, npl.rated_torque_nm + 0.06, "rated torque", color=TEXT_SECONDARY, fontsize=8)
    n_bd = (1 - s_max) * ec.synchronous_speed_rpm(f)
    ax1.annotate(
        f"breakdown {t_max:.2f} N·m at {n_bd:.0f} r/min",
        xy=(n_bd, t_max),
        xytext=(n_bd + 120, t_max + 0.35),
        fontsize=8,
        color=TEXT_SECONDARY,
        arrowprops={"arrowstyle": "-", "color": TEXT_SECONDARY, "linewidth": 0.8},
    )
    ax1.set_ylim(top=t_max + 0.6)
    ax1.set_xlabel("Speed (r/min)")
    ax1.set_ylabel("Electromagnetic torque (N·m)")
    ax1.set_title("Torque–speed characteristic, 208 V / 60 Hz")
    ax1.legend(loc="lower left")
    ax2.plot(n_curve, i_curve, color=SERIES[0], label="Equivalent circuit")
    ax2.plot(speeds, dq_current, **marker_kwargs(SERIES[1]), label="dq model (locked speed)")
    ax2.set_xlabel("Speed (r/min)")
    ax2.set_ylabel("Line current (A rms)")
    ax2.set_title("Stator current vs speed")
    ax2.set_ylim(bottom=0)
    ax2.legend(loc="lower left")
    fig.tight_layout()
    report.figures.append(save(fig, img_dir / "phase1_torque_speed.png"))


# ------------------------------------------------------------------------ E3
def vf_family(cfg: DriveConfig, report: ExperimentReport, img_dir: Path) -> None:
    """V/f law with boost and bus limit, and the resulting torque–speed family."""
    m = cfg.motor
    ec = EquivalentCircuit(m)
    s = cfg.vf_control
    v_lim = 0.5 * cfg.dc_link.nominal_voltage_v
    f_axis = np.linspace(0.0, 60.0, 241)
    v_cmd = np.array([vf_voltage_peak(float(fx), s) for fx in f_axis])
    v_no_boost = np.array(
        [vf_voltage_peak(float(fx), dataclasses.replace(s, boost_fraction=0.0)) for fx in f_axis]
    )
    corner = f_axis[np.argmax(v_cmd >= v_lim)] if np.any(v_cmd >= v_lim) else float("nan")

    apply_style()
    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(10.5, 3.8), gridspec_kw={"width_ratios": [1, 1.4]}
    )
    ax1.plot(
        f_axis,
        np.minimum(v_cmd, v_lim),
        color=SERIES[0],
        label=f"V/f with {s.boost_fraction:.0%} boost",
    )
    ax1.plot(f_axis, np.minimum(v_no_boost, v_lim), color=SERIES[1], label="V/f without boost")
    ax1.axhline(v_lim, **reference_line_kwargs())
    ax1.text(1, v_lim + 3, f"SPWM limit V_dc/2 = {v_lim:.1f} V", color=TEXT_SECONDARY, fontsize=8)
    ax1.set_xlabel("Output frequency (Hz)")
    ax1.set_ylabel("Phase voltage, peak (V)")
    ax1.set_title("V/f law")
    ax1.set_ylim(0, v_lim * 1.18)
    ax1.legend(loc="lower right")

    rows = []
    family = {}
    for k, freq in enumerate((15.0, 30.0, 45.0, 60.0)):
        v_ll = _available_line_voltage(cfg, freq, s)
        n, t, _ = ec.torque_speed_curve(v_ll, freq, slip_range=(1.0, 0.0))
        s_max, t_max = ec.breakdown(v_ll, freq)
        ax2.plot(n, t, color=SERIES[k], label=f"{freq:.0f} Hz")
        n_peak = max((1.0 - s_max) * ec.synchronous_speed_rpm(freq), 60.0)
        ax2.text(
            n_peak, t_max + 0.07, f"{freq:.0f} Hz", ha="center", fontsize=8, color=TEXT_SECONDARY
        )
        family[f"{freq:.0f}"] = {"line_voltage_v": v_ll, "breakdown_torque_nm": t_max}
        rows.append(
            [
                f"{freq:.0f} Hz",
                f"{_fmt(v_ll, 1)} V",
                f"{_fmt(t_max, 3)} N·m",
                f"{_fmt(t_max / m.nameplate.rated_torque_nm, 2)}×",
            ]
        )
    ax2.axhline(m.nameplate.rated_torque_nm, **reference_line_kwargs())
    ax2.text(
        20, m.nameplate.rated_torque_nm - 0.2, "rated torque", color=TEXT_SECONDARY, fontsize=8
    )
    ax2.set_xlabel("Speed (r/min)")
    ax2.set_ylabel("Torque (N·m)")
    ax2.set_title("Steady-state torque–speed curves under V/f")
    ax2.set_ylim(0, 1.3 * max(c["breakdown_torque_nm"] for c in family.values()))
    ax2.legend(loc="upper right", ncols=4)
    fig.tight_layout()
    report.figures.append(save(fig, img_dir / "phase1_vf_family.png"))
    report.metrics["vf_family"] = {"corner_frequency_hz": float(corner), "curves": family}
    report.markdown.append(
        "#### Torque capability under V/f (equivalent circuit, "
        f"{s.boost_fraction:.0%} boost, {cfg.dc_link.nominal_voltage_v:.0f} V bus, SPWM)\n\n"
        + _table(["Frequency", "Applied line voltage", "Breakdown torque", "vs rated"], rows)
        + f"\n\nThe SPWM voltage limit is reached at {corner:.2f} Hz: above it the drive can no"
        " longer hold V/f constant and the machine is under-fluxed."
    )


# ------------------------------------------------------------------------ E4
def vf_start(cfg: DriveConfig, report: ExperimentReport, img_dir: Path) -> None:
    """Ramp start under load followed by a load step, default V/f settings."""
    m = cfg.motor
    t_rated = m.nameplate.rated_torque_nm
    t_step = 3.0
    load: Load = CompositeLoad(
        (
            InertiaLoad(1e-3),
            ConstantTorqueLoad(0.3 * t_rated),
            StepLoad(t_step, torque_after_nm=0.7 * t_rated),
        )
    )
    ref = PiecewiseConstant.constant(1500.0)
    sim = build_vf_simulator(cfg, load=load, speed_reference_rpm=ref)
    res = sim.run(4.5)
    x = res.metadata["final_state"]
    w_mag, w_kin = sim.plant.stored_energy(x)
    e_in, e_cus, e_cur, e_em, e_load, e_fric = x[6:12]
    residual_el = e_in - (e_cus + e_cur + e_em + w_mag)
    residual_mech = e_em - (e_load + e_fric + w_kin)

    speed = res.speed_rpm
    reach = res.t[np.argmax(speed >= 0.95 * 1500.0)]
    before = _mean_speed(res, t_step - 0.5, t_step)
    after = _mean_speed(res, 4.0)
    dip = float(np.min(speed[res.window(t_step, 4.5)]))
    i_peak = float(np.max(np.abs(np.column_stack([res["i_a"], res["i_b"], res["i_c"]]))))
    report.metrics["vf_start"] = {
        "time_to_95pct_speed_s": float(reach),
        "peak_phase_current_a": i_peak,
        "speed_before_step_rpm": before,
        "speed_after_step_rpm": after,
        "speed_dip_rpm": dip,
        "energy_residual_electrical_rel": residual_el / e_in,
        "energy_residual_mechanical_rel": residual_mech / e_em,
        "input_energy_j": e_in,
    }
    report.markdown.append(
        "#### V/f start to 1500 r/min"
        " (30 % rated load, +70 % load step at 3 s, slip compensation on)\n\n"
        + _table(
            ["Metric", "Value"],
            [
                ["Time to 95 % of reference speed", f"{_fmt(reach, 3)} s"],
                ["Peak phase current", f"{_fmt(i_peak, 3)} A"],
                ["Speed before / after load step", f"{_fmt(before, 1)} / {_fmt(after, 1)} r/min"],
                ["Minimum speed after the step", f"{_fmt(dip, 1)} r/min"],
                [
                    "Energy balance residual (electrical / mechanical)",
                    f"{residual_el / e_in:.1e} / {residual_mech / e_em:.1e} (relative)",
                ],
            ],
        )
    )

    apply_style()
    fig, axes = plt.subplots(4, 1, figsize=(9.5, 9.0), sharex=True)
    t = res.t
    ax = axes[0]
    ax.plot(
        t,
        120.0 * res["ctrl.f_out_hz"] / m.nameplate.poles,
        color=SERIES[1],
        label="Synchronous speed of f_out",
    )
    ax.plot(t, speed, color=SERIES[0], label="Rotor speed")
    ax.axhline(1500.0, **reference_line_kwargs())
    ax.set_ylabel("Speed (r/min)")
    ax.set_title("Speed")
    ax.legend(loc="lower right")
    ax = axes[1]
    ax.plot(t, res["torque_e"], color=SERIES[0], label="Electromagnetic torque")
    ax.plot(
        t, res["torque_load"] + res["torque_friction"], color=SERIES[1], label="Load + friction"
    )
    ax.set_ylabel("Torque (N·m)")
    ax.set_title("Torque")
    ax.legend(loc="upper right")
    ax = axes[2]
    ax.plot(t, res["i_a"], color=SERIES[0], linewidth=0.8, label="i_a")
    lim = cfg.vf_control.current_limit_a
    if lim is not None:
        for sgn in (1, -1):
            ax.axhline(sgn * lim * math.sqrt(2.0), **reference_line_kwargs())
        ax.text(
            0.02,
            lim * math.sqrt(2.0) + 0.15,
            "current limit (peak)",
            color=TEXT_SECONDARY,
            fontsize=8,
        )
    ax.set_ylabel("Current (A)")
    ax.set_title("Phase current")
    ax = axes[3]
    ax.plot(t, res["psi_s"], color=SERIES[0], label="|ψ_s|")
    ax.plot(t, res["psi_r"], color=SERIES[1], label="|ψ_r|")
    ax.axhline(m.rated_flux_linkage(), **reference_line_kwargs())
    ax.text(3.6, m.rated_flux_linkage() + 0.015, "rated V/ω", color=TEXT_SECONDARY, fontsize=8)
    ax.set_ylabel("Flux linkage (Wb)")
    ax.set_title("Flux magnitude")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="lower right")
    for a in axes:
        a.axvline(t_step, color=TEXT_SECONDARY, linewidth=0.8, alpha=0.5)
    fig.tight_layout()
    report.figures.append(save(fig, img_dir / "phase1_vf_start.png"))


# ------------------------------------------------------------------------ E5
def slip_compensation(cfg: DriveConfig, report: ExperimentReport, img_dir: Path) -> None:
    """Speed regulation at 900 r/min with and without slip compensation."""
    t_rated = cfg.motor.nameplate.rated_torque_nm
    t_step = 1.5
    load = CompositeLoad(
        (InertiaLoad(1e-3), ConstantTorqueLoad(0.2 * t_rated), StepLoad(t_step, 0.8 * t_rated))
    )
    runs = {}
    for gain in (0.0, 1.0):
        sim = build_vf_simulator(
            cfg,
            load=load,
            speed_reference_rpm=PiecewiseConstant.constant(900.0),
            vf_settings=_vf(cfg, slip_compensation_gain=gain),
        )
        runs[gain] = sim.run(3.5)
    rows, metrics = [], {}
    for gain, res in runs.items():
        light = _mean_speed(res, t_step - 0.3, t_step)
        full = _mean_speed(res, 3.0)
        metrics[f"gain_{gain:.0f}"] = {"speed_light_rpm": light, "speed_rated_rpm": full}
        rows.append(
            [
                "on" if gain else "off",
                f"{_fmt(light, 1)} r/min ({_fmt(100 * (light - 900) / 900, 2)} %)",
                f"{_fmt(full, 1)} r/min ({_fmt(100 * (full - 900) / 900, 2)} %)",
            ]
        )
    report.metrics["slip_compensation"] = metrics
    report.markdown.append(
        "#### Slip compensation at 900 r/min (30 Hz)\n\n"
        + _table(
            ["Slip compensation", "Speed at 20 % load (error)", "Speed at 100 % load (error)"], rows
        )
    )

    apply_style()
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9.5, 5.2), sharex=True, gridspec_kw={"height_ratios": [2, 1]}
    )
    for k, (gain, res) in enumerate(runs.items()):
        ax1.plot(
            res.t,
            res.speed_rpm,
            color=SERIES[k],
            label=f"slip compensation {'on' if gain else 'off'}",
        )
    ax1.axhline(900.0, **reference_line_kwargs())
    ax1.text(0.05, 905, "reference 900 r/min", color=TEXT_SECONDARY, fontsize=8)
    ax1.set_ylim(600, 960)
    ax1.set_ylabel("Speed (r/min)")
    ax1.set_title("Rotor speed (load step 20 % → 100 % rated at 1.5 s)")
    ax1.legend(loc="lower right")
    res_on = runs[1.0]
    ax2.plot(
        res_on.t,
        res_on["ctrl.slip_comp_hz"],
        color=SERIES[0],
        label="Δf added by slip compensation",
    )
    ax2.set_ylabel("Δf (Hz)")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("Slip-compensation frequency")
    for a in (ax1, ax2):
        a.axvline(t_step, color=TEXT_SECONDARY, linewidth=0.8, alpha=0.5)
    fig.tight_layout()
    report.figures.append(save(fig, img_dir / "phase1_slip_compensation.png"))


# ------------------------------------------------------------------------ E6
def boost(cfg: DriveConfig, report: ExperimentReport, img_dir: Path, quick: bool) -> None:
    """Low-frequency operation with 30 % rated load for several boost levels."""
    m = cfg.motor
    ec = EquivalentCircuit(m)
    t_load = 0.3 * m.nameplate.rated_torque_nm
    coul = 0.0

    def opposing(w: float) -> float:
        return t_load * math.tanh(w / 0.5) + m.viscous_friction * w + coul

    boosts = (0.0, 0.06, 0.12)
    f_axis = np.linspace(1.0, 12.0, 45)
    f_sim = (2.0, 4.0, 8.0) if quick else (2.0, 3.0, 5.0, 8.0)
    apply_style()
    fig, ax = plt.subplots(figsize=(8.0, 4.2))
    ax.plot(f_axis, 30.0 * f_axis, **reference_line_kwargs())
    ax.text(9.0, 30.0 * 9.0 + 12, "synchronous speed", color=TEXT_SECONDARY, fontsize=8, rotation=0)
    table_rows = []
    metrics = {}
    for k, b in enumerate(boosts):
        settings = _vf(cfg, boost_fraction=b, slip_compensation_gain=0.0)
        n_ec = []
        for fx in f_axis:
            v_ll = _available_line_voltage(cfg, float(fx), settings)
            try:
                n_ec.append(ec.operating_point_with_load(opposing, v_ll, float(fx)).speed_rpm)
            except ValueError:
                n_ec.append(0.0)  # stall
        ax.plot(f_axis, n_ec, color=SERIES[k], label=f"boost {b:.0%}")
        n_dq, psi_dq = [], []
        for fx in f_sim:
            res = build_vf_simulator(
                cfg,
                load=CompositeLoad((InertiaLoad(1e-3), ConstantTorqueLoad(t_load))),
                speed_reference_rpm=PiecewiseConstant.constant(30.0 * fx),
                vf_settings=settings,
            ).run(1.5)
            n_dq.append(_mean_speed(res, 1.2))
            psi_dq.append(res.mean("psi_s", 1.2))
        ax.plot(f_sim, n_dq, **marker_kwargs(SERIES[k]))
        metrics[f"boost_{b:.2f}"] = {
            f"{fx:.0f}Hz": {"speed_rpm": n, "psi_s_wb": psi}
            for fx, n, psi in zip(f_sim, n_dq, psi_dq, strict=True)
        }
        table_rows.append(
            [f"{b:.0%}"] + [f"{_fmt(n, 1)}" for n in n_dq] + [f"{_fmt(psi_dq[0], 3)} Wb"]
        )
    ax.set_xlabel("Commanded frequency (Hz)")
    ax.set_ylabel("Steady-state speed (r/min)")
    ax.set_title("Low-speed operation with 30 % rated load")
    ax.set_ylim(bottom=0)
    ax.plot([], [], color=TEXT_SECONDARY, label="lines: equivalent circuit")
    ax.plot([], [], **marker_kwargs(TEXT_SECONDARY), label="dots: dq simulation")
    ax.legend(loc="upper left")
    fig.tight_layout()
    report.figures.append(save(fig, img_dir / "phase1_boost.png"))
    report.metrics["boost"] = metrics
    report.markdown.append(
        "#### Voltage boost at low frequency (30 % rated load, slip compensation off)\n\n"
        + _table(
            ["Boost"]
            + [f"Speed at {fx:.0f} Hz (sync {30 * fx:.0f})" for fx in f_sim]
            + [f"Stator flux at {f_sim[0]:.0f} Hz"],
            table_rows,
        )
        + "\n\nSpeeds in r/min from the dq simulation. Without boost the stator resistance"
        " drop leaves too little flux and the motor stalls (speed ≈ 0). For reference, the"
        f" rated flux is about {m.rated_flux_linkage():.3f} Wb (V/ω at the rating plate)."
    )


# ------------------------------------------------------------------------ E7
def steady_state_table(cfg: DriveConfig, report: ExperimentReport, quick: bool) -> None:
    """V/f steady state at rated load, power balance and synchronous-speed check."""
    m = cfg.motor
    ec = EquivalentCircuit(m)
    t_rated = m.nameplate.rated_torque_nm
    load = CompositeLoad((InertiaLoad(1e-3), ConstantTorqueLoad(t_rated)))
    freqs = (30.0, 60.0) if quick else (15.0, 30.0, 45.0, 60.0)
    rows = []
    data: dict[str, Any] = {}
    for fx in freqs:
        rpm_ref = 30.0 * fx
        entry: dict[str, Any] = {}
        for gain in (0.0, 1.0):
            res = build_vf_simulator(
                cfg,
                load=load,
                speed_reference_rpm=PiecewiseConstant.constant(rpm_ref),
                vf_settings=_vf(cfg, slip_compensation_gain=gain),
            ).run(3.0)
            t0 = 2.5
            f_out = res.mean("ctrl.f_out_hz", t0)
            speed = _mean_speed(res, t0)
            n_sync = 30.0 * f_out
            v_pk = float(np.mean(np.hypot(res["v_alpha"], res["v_beta"])[res.window(t0)]))
            entry[f"gain_{gain:.0f}"] = {
                "f_out_hz": f_out,
                "speed_rpm": speed,
                "slip": (n_sync - speed) / n_sync,
                "current_rms_a": _rms(res["i_a"][res.window(t0)]),
                "phase_voltage_peak_v": v_pk,
                "psi_s_wb": res.mean("psi_s", t0),
            }
            if gain == 0.0:
                op = ec.operating_point(entry["gain_0"]["slip"], v_pk * math.sqrt(1.5), f_out)
                entry["ec_current_rms_a"] = op.stator_current_rms_a
        g0, g1 = entry["gain_0"], entry["gain_1"]
        data[f"{fx:.0f}"] = entry
        rows.append(
            [
                f"{fx:.0f} Hz / {rpm_ref:.0f}",
                f"{_fmt(g0['phase_voltage_peak_v'], 1)} V",
                f"{_fmt(g0['psi_s_wb'], 3)} Wb",
                f"{_fmt(g0['speed_rpm'], 1)} ({_fmt(100 * g0['slip'], 2)} %)",
                f"{_fmt(g0['current_rms_a'], 3)} / {_fmt(entry['ec_current_rms_a'], 3)} A",
                f"{_fmt(g1['speed_rpm'], 1)} ({_fmt(g1['f_out_hz'], 2)} Hz)",
            ]
        )
    report.metrics["vf_rated_load"] = data
    report.markdown.append(
        "#### V/f steady state at rated load torque\n\n"
        + _table(
            [
                "f / reference (r/min)",
                "Applied V (peak, phase)",
                "Stator flux",
                "Speed, no slip comp. (slip)",
                "Current (dq / eq. circuit)",
                "Speed with slip comp. (f_out)",
            ],
            rows,
        )
    )

    # Power balance at 45 Hz, slip compensation off
    res = build_vf_simulator(
        cfg,
        load=load,
        speed_reference_rpm=PiecewiseConstant.constant(1350.0),
        vf_settings=_vf(cfg, slip_compensation_gain=0.0),
    ).run(2.5)
    t0, t1 = 2.0, 2.5 - 1e-4
    p = {
        name: res.average_power(f"e_{name}", t0, t1)
        for name in ("in", "cu_s", "cu_r", "fric", "load", "em")
    }
    losses = p["cu_s"] + p["cu_r"] + p["fric"]
    residual = p["in"] - (losses + p["load"])
    report.metrics["power_balance_45hz"] = {
        **{f"p_{k}_w": v for k, v in p.items()},
        "residual_w": residual,
    }
    report.markdown.append(
        "#### Power balance at 45 Hz, rated load (averages over 0.5 s of steady state)\n\n"
        + _table(
            ["Term", "Power"],
            [
                ["Electrical input P_in", f"{_fmt(p['in'], 2)} W"],
                ["Stator copper loss", f"{_fmt(p['cu_s'], 2)} W"],
                ["Rotor copper loss", f"{_fmt(p['cu_r'], 2)} W"],
                ["Motor bearing friction", f"{_fmt(p['fric'], 2)} W"],
                ["Delivered to the load", f"{_fmt(p['load'], 2)} W"],
                ["Residual P_in − Σ", f"{residual:.1e} W ({residual / p['in']:.1e} of P_in)"],
                ["Efficiency (no core loss)", f"{_fmt(100 * p['load'] / p['in'], 1)} %"],
            ],
        )
    )

    # 120 f / P at no load (frictionless motor)
    motor0 = dataclasses.replace(m, viscous_friction=0.0)
    cfg0 = dataclasses.replace(cfg, motor=motor0)
    res = build_vf_simulator(
        cfg0,
        load=CompositeLoad((InertiaLoad(1e-3),)),
        speed_reference_rpm=PiecewiseConstant.constant(1350.0),
        vf_settings=_vf(cfg0, slip_compensation_gain=0.0),
    ).run(3.0)
    f_out = res.mean("ctrl.f_out_hz", 2.5)
    speed = _mean_speed(res, 2.5)
    expected = 120.0 * f_out / m.nameplate.poles
    report.metrics["synchronous_speed_check"] = {
        "f_out_hz": f_out,
        "speed_rpm": speed,
        "expected_rpm": expected,
        "rel_error": (speed - expected) / expected,
    }


# ------------------------------------------------------------------ driver
def run(out_dir: Path | None = None, quick: bool = False) -> ExperimentReport:
    """Run all Phase 1 experiments and write figures/metrics under ``out_dir``."""
    out = out_dir or REPO_ROOT / "docs"
    img_dir = out / "img"
    res_dir = out / "results"
    res_dir.mkdir(parents=True, exist_ok=True)
    cfg = default_config()
    report = ExperimentReport()
    report.metrics["motor"] = {
        "name": cfg.motor.name,
        "provenance": cfg.motor.provenance,
        "dc_link_v": cfg.dc_link.nominal_voltage_v,
        "switching_frequency_hz": cfg.inverter.switching_frequency_hz,
    }
    rated_point_and_torque_speed(cfg, report, img_dir, quick)
    vf_family(cfg, report, img_dir)
    steady_state_table(cfg, report, quick)
    vf_start(cfg, report, img_dir)
    slip_compensation(cfg, report, img_dir)
    boost(cfg, report, img_dir, quick)

    tsv = report.metrics["torque_speed_validation"]
    sync = report.metrics["synchronous_speed_check"]
    pb = report.metrics["power_balance_45hz"]
    start = report.metrics["vf_start"]
    validation = "#### Model validation summary\n\n" + _table(
        ["Check", "Result"],
        [
            [
                "No-load speed vs 120·f/P (45 Hz, frictionless)",
                f"{_fmt(sync['speed_rpm'], 3)} vs {_fmt(sync['expected_rpm'], 3)} r/min"
                f" (error {sync['rel_error']:.1e})",
            ],
            [
                f"dq torque vs equivalent circuit ({tsv['points']} locked-speed points)",
                f"max abs(ΔT) = {tsv['max_abs_torque_error_nm']:.1e} N·m",
            ],
            [
                "dq current vs equivalent circuit",
                f"max relative error {tsv['max_rel_current_error']:.1e}",
            ],
            [
                "Steady-state power balance (45 Hz, rated load)",
                f"relative residual {pb['residual_w'] / pb['p_in_w']:.1e}",
            ],
            [
                "Energy conservation over a 4.5 s start + load step",
                f"relative residual {start['energy_residual_electrical_rel']:.1e}",
            ],
        ],
    )
    report.markdown.insert(0, validation)
    md = "\n\n".join(report.markdown) + "\n"
    (res_dir / "phase1.md").write_text(md, encoding="utf-8")
    (res_dir / "phase1_metrics.json").write_text(
        json.dumps(report.metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def main(argv: Sequence[str] | None = None) -> None:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=None, help="output directory (default: docs/)")
    parser.add_argument("--quick", action="store_true", help="fewer points (smoke test)")
    args = parser.parse_args(argv)
    report = run(args.out, quick=args.quick)
    for f in report.figures:
        print(f"figure: {f}")
    print("\n".join(report.markdown))


if __name__ == "__main__":
    main()
