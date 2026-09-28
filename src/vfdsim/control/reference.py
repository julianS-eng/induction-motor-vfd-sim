"""Reference (set-point) profiles."""

from __future__ import annotations

import bisect
import itertools
from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class PiecewiseConstant:
    """Piecewise-constant profile defined by ``(time, value)`` breakpoints.

    The value before the first breakpoint equals the first value.

    Examples
    --------
    >>> ref = PiecewiseConstant(((0.0, 1500.0), (2.0, 900.0)))
    >>> ref(1.0), ref(2.5)
    (1500.0, 900.0)
    """

    points: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        if not self.points:
            raise ValueError("at least one breakpoint is required")
        times = [p[0] for p in self.points]
        if any(b < a for a, b in itertools.pairwise(times)):
            raise ValueError("breakpoint times must be non-decreasing")

    @classmethod
    def from_pairs(cls, pairs: Sequence[Sequence[float]]) -> PiecewiseConstant:
        """Build from a list of ``[time, value]`` pairs (e.g. parsed YAML)."""
        return cls(tuple((float(p[0]), float(p[1])) for p in pairs))

    @classmethod
    def constant(cls, value: float) -> PiecewiseConstant:
        """A constant profile."""
        return cls(((0.0, float(value)),))

    def __call__(self, t: float) -> float:
        """Profile value at time ``t``."""
        times = [p[0] for p in self.points]
        i = bisect.bisect_right(times, t) - 1
        return self.points[max(i, 0)][1]
