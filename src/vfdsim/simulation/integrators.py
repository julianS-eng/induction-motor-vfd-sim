"""Fixed-step explicit integrators working on plain Python lists.

For state vectors of a dozen elements, list arithmetic on floats is several
times faster than NumPy (whose per-call overhead dominates at this size).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

Derivative = Callable[[float, Sequence[float]], list[float]]


def rk4_step(f: Derivative, t: float, x: Sequence[float], h: float) -> list[float]:
    """One classical fourth-order Runge–Kutta step."""
    hh = 0.5 * h
    k1 = f(t, x)
    k2 = f(t + hh, [a + hh * b for a, b in zip(x, k1, strict=True)])
    k3 = f(t + hh, [a + hh * b for a, b in zip(x, k2, strict=True)])
    k4 = f(t + h, [a + h * b for a, b in zip(x, k3, strict=True)])
    h6 = h / 6.0
    return [
        a + h6 * (b1 + 2.0 * b2 + 2.0 * b3 + b4)
        for a, b1, b2, b3, b4 in zip(x, k1, k2, k3, k4, strict=True)
    ]


def integrate(
    f: Derivative, t0: float, x: Sequence[float], duration: float, n_steps: int
) -> list[float]:
    """Integrate ``dx/dt = f(t, x)`` over ``duration`` with ``n_steps`` RK4 steps."""
    if n_steps < 1:
        raise ValueError("n_steps must be >= 1")
    h = duration / n_steps
    y = list(x)
    t = t0
    for _ in range(n_steps):
        y = rk4_step(f, t, y, h)
        t += h
    return y
