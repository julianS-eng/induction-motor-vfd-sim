"""Consistent Matplotlib styling for all figures of the project.

Colours follow a colour-blind-validated categorical order (blue, orange, aqua,
yellow, ...) that is assigned in fixed order and never cycled. Text always uses
neutral ink colours; identity is carried by the marks and the legend.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib as mpl
from cycler import cycler

mpl.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
REFERENCE = "#8a8984"  # neutral gray for reference / limit lines

#: Categorical series colours, in the fixed assignment order.
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7")

LINE_WIDTH = 1.6
MARKER_SIZE = 6.0


def apply_style() -> None:
    """Install the project-wide rcParams."""
    mpl.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "axes.edgecolor": GRID,
            "axes.labelcolor": TEXT_SECONDARY,
            "axes.titlecolor": TEXT_PRIMARY,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.titlelocation": "left",
            "axes.labelsize": 9.5,
            "axes.grid": True,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.prop_cycle": cycler(color=list(SERIES)),
            "grid.color": GRID,
            "grid.linewidth": 0.7,
            "grid.linestyle": "-",
            "xtick.color": TEXT_SECONDARY,
            "ytick.color": TEXT_SECONDARY,
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "text.color": TEXT_PRIMARY,
            "legend.frameon": False,
            "legend.fontsize": 8.5,
            "legend.labelcolor": TEXT_PRIMARY,
            "lines.linewidth": LINE_WIDTH,
            "lines.solid_capstyle": "round",
            "lines.solid_joinstyle": "round",
            "lines.markersize": MARKER_SIZE,
            "font.family": "DejaVu Sans",
            "figure.dpi": 110,
            "savefig.dpi": 130,
            "savefig.bbox": "tight",
        }
    )


def reference_line_kwargs() -> dict[str, Any]:
    """Style for reference/limit lines (neutral, thin, dashed)."""
    return {"color": REFERENCE, "linewidth": 1.0, "linestyle": (0, (4, 3))}


def marker_kwargs(color: str) -> dict[str, Any]:
    """Filled marker with a surface-coloured ring."""
    return {
        "marker": "o",
        "markersize": MARKER_SIZE,
        "markerfacecolor": color,
        "markeredgecolor": SURFACE,
        "markeredgewidth": 1.5,
        "linestyle": "none",
    }


def save(fig: Any, path: str | Path) -> Path:
    """Save a figure (creating parent directories) and close it."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(p)
    plt.close(fig)
    return p
