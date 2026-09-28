"""Container for simulation outputs."""

from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]

RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)


@dataclass
class SimulationResult:
    """Time series recorded at the control sampling instants.

    Every channel is a 1-D array aligned with ``t``. Controller signals are
    stored with the prefix ``"ctrl."``. Voltages ``v_alpha``/``v_beta`` and the
    duties are those applied during the period that *starts* at each sample.
    """

    signals: dict[str, FloatArray]
    metadata: dict[str, Any] = field(default_factory=dict)

    def __getitem__(self, name: str) -> FloatArray:
        return self.signals[name]

    def __contains__(self, name: object) -> bool:
        return name in self.signals

    def __iter__(self) -> Iterator[str]:
        return iter(self.signals)

    @property
    def t(self) -> FloatArray:
        """Sample times in s."""
        return self.signals["t"]

    @property
    def speed_rpm(self) -> FloatArray:
        """Mechanical speed in r/min."""
        return self.signals["omega_m"] * RAD_S_TO_RPM

    def window(self, t_start: float, t_stop: float | None = None) -> NDArray[np.bool_]:
        """Boolean mask of samples with ``t_start <= t < t_stop``."""
        t = self.t
        stop = t[-1] + 1.0 if t_stop is None else t_stop
        return np.asarray((t >= t_start) & (t < stop), dtype=np.bool_)

    def mean(self, name: str, t_start: float, t_stop: float | None = None) -> float:
        """Average of a channel over a time window."""
        return float(np.mean(self.signals[name][self.window(t_start, t_stop)]))

    def at(self, name: str, t: float) -> float:
        """Value of a channel at the last sample not later than ``t``."""
        idx = int(np.searchsorted(self.t, t, side="right")) - 1
        return float(self.signals[name][max(idx, 0)])

    def delta(self, name: str, t_start: float, t_stop: float) -> float:
        """Increment of an accumulator channel between two instants."""
        return self.at(name, t_stop) - self.at(name, t_start)

    def average_power(self, energy_channel: str, t_start: float, t_stop: float) -> float:
        """Average power from an energy accumulator over ``[t_start, t_stop]``."""
        i0 = int(np.searchsorted(self.t, t_start, side="left"))
        i1 = int(np.searchsorted(self.t, t_stop, side="left"))
        i1 = min(i1, len(self.t) - 1)
        e = self.signals[energy_channel]
        return float((e[i1] - e[i0]) / (self.t[i1] - self.t[i0]))

    def save_npz(self, path: str | Path) -> None:
        """Save all channels to a compressed ``.npz`` file."""
        arrays: dict[str, Any] = dict(self.signals)
        np.savez_compressed(Path(path), **arrays)
