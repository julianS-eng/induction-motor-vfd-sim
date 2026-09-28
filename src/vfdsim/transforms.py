"""Reference-frame transformations (amplitude-invariant convention).

With the amplitude-invariant Clarke transform the length of the space vector
equals the peak value of the balanced phase quantities, and the three-phase
instantaneous power is ``p = 3/2 (v_α i_α + v_β i_β) = 3/2 (v_d i_d + v_q i_q)``.

The Park transform rotates the stationary αβ frame by the angle ``θ`` of the
rotating dq frame, with ``d`` aligned to ``θ``::

    [x_d]   [ cos θ   sin θ] [x_α]
    [x_q] = [-sin θ   cos θ] [x_β]

Scalar functions operate on Python floats (used inside the fixed-step control
loops, mirroring the embedded C implementation); the ``*_array`` variants are
vectorised with NumPy for post-processing.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

SQRT3 = math.sqrt(3.0)
INV_SQRT3 = 1.0 / SQRT3
TWO_PI = 2.0 * math.pi

FloatArray = NDArray[np.float64]


def clarke(a: float, b: float, c: float) -> tuple[float, float]:
    """Amplitude-invariant Clarke transform ``abc → αβ`` (zero sequence dropped)."""
    alpha = (2.0 * a - b - c) / 3.0
    beta = (b - c) * INV_SQRT3
    return alpha, beta


def inverse_clarke(alpha: float, beta: float) -> tuple[float, float, float]:
    """Inverse amplitude-invariant Clarke transform ``αβ → abc`` (no zero sequence)."""
    a = alpha
    b = -0.5 * alpha + 0.5 * SQRT3 * beta
    c = -0.5 * alpha - 0.5 * SQRT3 * beta
    return a, b, c


def park(alpha: float, beta: float, theta: float) -> tuple[float, float]:
    """Park transform ``αβ → dq`` for a frame at electrical angle ``theta`` (rad)."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return alpha * cos_t + beta * sin_t, -alpha * sin_t + beta * cos_t


def inverse_park(d: float, q: float, theta: float) -> tuple[float, float]:
    """Inverse Park transform ``dq → αβ`` for a frame at electrical angle ``theta``."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return d * cos_t - q * sin_t, d * sin_t + q * cos_t


def wrap_angle(theta: float) -> float:
    """Wrap an angle to the interval ``[-π, π)``."""
    return (theta + math.pi) % TWO_PI - math.pi


def clarke_array(a: FloatArray, b: FloatArray, c: FloatArray) -> tuple[FloatArray, FloatArray]:
    """Vectorised amplitude-invariant Clarke transform."""
    return (2.0 * a - b - c) / 3.0, (b - c) * INV_SQRT3


def inverse_clarke_array(
    alpha: FloatArray, beta: FloatArray
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Vectorised inverse Clarke transform."""
    return (
        alpha,
        -0.5 * alpha + 0.5 * SQRT3 * beta,
        -0.5 * alpha - 0.5 * SQRT3 * beta,
    )


def park_array(
    alpha: FloatArray, beta: FloatArray, theta: FloatArray
) -> tuple[FloatArray, FloatArray]:
    """Vectorised Park transform."""
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return alpha * cos_t + beta * sin_t, -alpha * sin_t + beta * cos_t
