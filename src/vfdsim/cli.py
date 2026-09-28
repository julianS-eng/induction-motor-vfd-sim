"""Command-line interface: ``vfdsim {info, simulate, experiments}``."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from vfdsim import __version__
from vfdsim.config import DEFAULT_CONFIG_DIR, DriveConfig, load_config
from vfdsim.motor import EquivalentCircuit
from vfdsim.simulation import SimulationResult


def _load(path: str | None) -> DriveConfig:
    return load_config(Path(path) if path else DEFAULT_CONFIG_DIR / "default.yaml")


def cmd_info(args: argparse.Namespace) -> int:
    """Print motor data, derived quantities and the rated operating point."""
    cfg = _load(args.config)
    m = cfg.motor
    npl = m.nameplate
    ec = EquivalentCircuit(m)
    print(f"Motor: {m.name}")
    if m.is_assumed:
        groups = ", ".join(k for k, v in m.provenance.items() if v == "assumed")
        print(f"  WARNING: assumed (not measured) parameters: {groups}")
    print(
        f"  Nameplate: {npl.rated_power_w:g} W, {npl.rated_voltage_v:g} V, "
        f"{npl.rated_current_a:g} A, {npl.rated_frequency_hz:g} Hz, "
        f"{npl.rated_speed_rpm:g} r/min, {npl.poles} poles ({npl.connection})"
    )
    print(
        f"  Rs={m.r_s:g} Ω  Rr={m.r_r:g} Ω  Lls={m.l_ls:g} H  Llr={m.l_lr:g} H  "
        f"Lm={m.l_m:g} H  J={m.inertia:g} kg·m²  B={m.viscous_friction:g} N·m·s/rad"
    )
    print(
        f"  Derived: n_sync={npl.synchronous_speed_rpm:g} r/min, s_rated={npl.rated_slip:.4f}, "
        f"T_rated={npl.rated_torque_nm:.4f} N·m, σ={m.sigma:.4f}, "
        f"τ_r={1e3 * m.rotor_time_constant:.2f} ms"
    )
    op = ec.operating_point_with_load(
        lambda w: npl.rated_torque_nm + m.viscous_friction * w,
        npl.rated_voltage_v,
        npl.rated_frequency_hz,
    )
    s_max, t_max = ec.breakdown(npl.rated_voltage_v, npl.rated_frequency_hz)
    print(
        f"  Rated point (equivalent circuit): {op.speed_rpm:.1f} r/min, "
        f"{op.stator_current_rms_a:.3f} A, PF={op.power_factor:.3f}; "
        f"breakdown {t_max:.3f} N·m at s={s_max:.3f}"
    )
    print(
        f"Drive: {cfg.inverter.model} inverter, {cfg.modulation.method}, "
        f"f_sw={cfg.inverter.switching_frequency_hz:g} Hz, V_dc={cfg.dc_link.nominal_voltage_v:g} V"
    )
    return 0


def cmd_simulate(args: argparse.Namespace) -> int:
    """Run the V/f scenario of a configuration file."""
    from vfdsim.scenarios import run_config

    cfg = _load(args.config)
    t_end = args.t_end if args.t_end is not None else cfg.simulation.t_end_s
    res = run_config(cfg, t_end)
    speed = res.speed_rpm
    i_peak = float(np.max(np.abs(np.column_stack([res["i_a"], res["i_b"], res["i_c"]]))))
    tail = res.window(max(0.0, res.t[-1] - 0.1))
    print(
        f"Simulated {t_end:g} s in {res.metadata['wall_time_s']:.2f} s wall time "
        f"({len(res.t)} samples)"
    )
    print(f"  final speed      : {float(np.mean(speed[tail])):.2f} r/min")
    print(f"  final torque     : {float(np.mean(res['torque_e'][tail])):.4f} N·m")
    print(f"  peak phase current: {i_peak:.3f} A")
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        res.save_npz(out)
        print(f"  signals saved to {out}")
    if args.plot:
        _quick_plot(res, Path(args.plot))
        print(f"  plot saved to {args.plot}")
    return 0


def _quick_plot(res: SimulationResult, path: Path) -> None:
    import matplotlib.pyplot as plt

    from vfdsim.plotting import SERIES, apply_style, save

    apply_style()
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(res.t, res.speed_rpm, color=SERIES[0])
    axes[0].set_ylabel("Speed (r/min)")
    axes[1].plot(res.t, res["torque_e"], color=SERIES[0], label="T_e")
    axes[1].plot(res.t, res["torque_load"], color=SERIES[1], label="T_load")
    axes[1].set_ylabel("Torque (N·m)")
    axes[1].legend()
    axes[2].plot(res.t, res["i_a"], color=SERIES[0], linewidth=0.8)
    axes[2].set_ylabel("i_a (A)")
    axes[2].set_xlabel("Time (s)")
    save(fig, path)


def cmd_experiments(args: argparse.Namespace) -> int:
    """Regenerate the figures and metrics of one project phase."""
    if args.phase == "phase1":
        from vfdsim.experiments import phase1

        report = phase1.run(Path(args.out) if args.out else None, quick=args.quick)
        for f in report.figures:
            print(f"figure: {f}")
        return 0
    raise SystemExit(f"unknown phase {args.phase!r}")  # pragma: no cover - argparse guards


def build_parser() -> argparse.ArgumentParser:
    """Argument parser of the ``vfdsim`` command."""
    parser = argparse.ArgumentParser(
        prog="vfdsim", description="Induction motor variable-frequency drive simulator."
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="show motor and drive data")
    p_info.add_argument("--config", help="drive YAML (default: configs/default.yaml)")
    p_info.set_defaults(func=cmd_info)

    p_sim = sub.add_parser("simulate", help="run the scenario of a configuration file")
    p_sim.add_argument("--config", help="drive YAML (default: configs/default.yaml)")
    p_sim.add_argument("--t-end", type=float, default=None, help="simulated time in s")
    p_sim.add_argument("--out", help="save all signals to this .npz file")
    p_sim.add_argument("--plot", help="save a quick-look plot to this .png file")
    p_sim.set_defaults(func=cmd_simulate)

    p_exp = sub.add_parser("experiments", help="regenerate the figures/metrics of a phase")
    p_exp.add_argument("phase", choices=["phase1"])
    p_exp.add_argument("--out", help="output directory (default: docs/)")
    p_exp.add_argument("--quick", action="store_true", help="fewer points (smoke test)")
    p_exp.set_defaults(func=cmd_experiments)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point of the ``vfdsim`` console script."""
    args = build_parser().parse_args(argv)
    code: int = args.func(args)
    return code


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
