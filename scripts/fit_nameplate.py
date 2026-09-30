"""Fit the T-equivalent circuit to rating-plate data (motor mode), check generator mode.

The LabVolt 8221-22 plate gives, at 208 V / 60 Hz, a motor point (200 W, PF 0.73,
1685 r/min, 1.14 A) and a generator point (200 W, PF 0.47, 1900 r/min, 1.18 A).
The model has no iron-loss branch, so both points cannot be matched at once: the
motor point is fitted exactly (plus an assumed breakdown torque of 2.0x rated,
Lls = Llr) and the generator point is reported as a validation.

Usage::

    python scripts/fit_nameplate.py            # print the fit
    python scripts/fit_nameplate.py --write    # also write docs/results/labvolt_fit.md
"""

from __future__ import annotations

import argparse
import dataclasses
import math
from pathlib import Path

from scipy.optimize import least_squares

from vfdsim.config import load_motor
from vfdsim.motor import EquivalentCircuit, MotorParameters

ROOT = Path(__file__).resolve().parents[1]
MOTOR_FILE = ROOT / "configs" / "motors" / "labvolt_8221_22.yaml"

# Rating plate (LabVolt 8221-22)
V_LL, F = 208.0, 60.0
MOTOR = {"power_w": 200.0, "pf": 0.73, "speed_rpm": 1685.0, "current_a": 1.14}
GENERATOR = {"power_w": 200.0, "pf": 0.47, "speed_rpm": 1900.0, "current_a": 1.18}
BREAKDOWN_RATIO = 2.0  # assumption (typical small motor), plate does not give it


def _with(base: MotorParameters, x: list[float]) -> MotorParameters:
    r_s, r_r, l_l, l_m = x
    return dataclasses.replace(base, r_s=r_s, r_r=r_r, l_ls=l_l, l_lr=l_l, l_m=l_m)


def fit(base: MotorParameters) -> MotorParameters:
    """Least-squares fit of Rs, Rr, Ll (= Lls = Llr), Lm to the motor-mode plate data."""
    n_s = 120.0 * F / base.nameplate.poles
    slip = (n_s - MOTOR["speed_rpm"]) / n_s
    w_m = MOTOR["speed_rpm"] * 2.0 * math.pi / 60.0
    t_shaft = MOTOR["power_w"] / w_m

    def residual(x: list[float]) -> list[float]:
        ec = EquivalentCircuit(_with(base, list(x)))
        op = ec.operating_point(slip, V_LL, F)
        _, t_max = ec.breakdown(V_LL, F)
        t_needed = t_shaft + base.viscous_friction * w_m
        return [
            (op.stator_current_rms_a - MOTOR["current_a"]) / MOTOR["current_a"],
            (op.power_factor - MOTOR["pf"]) / MOTOR["pf"],
            (op.torque_nm - t_needed) / t_needed,
            (t_max / t_shaft - BREAKDOWN_RATIO) / BREAKDOWN_RATIO,
        ]

    sol = least_squares(
        residual, [10.0, 10.0, 0.03, 0.35], bounds=([0.5, 0.5, 0.002, 0.05], [80, 80, 0.3, 2.0])
    )
    return _with(base, [float(v) for v in sol.x])


def report(m: MotorParameters) -> str:
    """Markdown comparison of plate and model in both operating modes."""
    ec = EquivalentCircuit(m)
    n_s = 120.0 * F / m.nameplate.poles
    rows = []
    for mode, d in (("Motor", MOTOR), ("Generator", GENERATOR)):
        slip = (n_s - d["speed_rpm"]) / n_s
        op = ec.operating_point(slip, V_LL, F)
        w = d["speed_rpm"] * 2.0 * math.pi / 60.0
        if mode == "Motor":
            power = (op.torque_nm - m.viscous_friction * w) * w  # shaft output
        else:
            power = -op.input_power_w  # electrical output
        rows.append(
            f"| {mode} ({d['speed_rpm']:.0f} r/min) | {d['power_w']:.0f} W / {power:.1f} W"
            f" | {d['pf']:.2f} / {abs(op.power_factor):.3f}"
            f" | {d['current_a']:.2f} A / {op.stator_current_rms_a:.3f} A |"
        )
    s_max, t_max = ec.breakdown(V_LL, F)
    return (
        "#### LabVolt 8221-22: equivalent circuit fitted to the rating plate\n\n"
        "| Parameter | Value |\n|---|---|\n"
        f"| R_s | {m.r_s:.3f} Ω |\n| R_r | {m.r_r:.3f} Ω |\n"
        f"| L_ls = L_lr | {1e3 * m.l_ls:.2f} mH |\n| L_m | {1e3 * m.l_m:.1f} mH |\n"
        f"| Breakdown torque | {t_max:.3f} N·m at s = {s_max:.3f} (assumed 2.0× rated) |\n\n"
        "| Mode | Power (plate / model) | PF (plate / model) | Current (plate / model) |\n"
        "|---|---|---|---|\n" + "\n".join(rows) + "\n\n"
        "The motor point is fitted exactly. The generator point is a validation: the model"
        " delivers the plate power but with a lower power factor and higher current, because"
        " it has no iron-loss branch and no saturation.\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write", action="store_true", help="write docs/results/labvolt_fit.md")
    args = parser.parse_args()
    m = fit(load_motor(MOTOR_FILE))
    print(
        f"stator_resistance_ohm: {m.r_s:.4f}\nrotor_resistance_ohm: {m.r_r:.4f}\n"
        f"stator_leakage_inductance_h: {m.l_ls:.5f}\nrotor_leakage_inductance_h: {m.l_lr:.5f}\n"
        f"magnetizing_inductance_h: {m.l_m:.5f}\n"
    )
    text = report(m)
    print(text)
    if args.write:
        (ROOT / "docs" / "results" / "labvolt_fit.md").write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
