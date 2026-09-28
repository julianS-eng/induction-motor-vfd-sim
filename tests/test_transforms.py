from __future__ import annotations

import math

import numpy as np
import pytest

from vfdsim.transforms import (
    clarke,
    clarke_array,
    inverse_clarke,
    inverse_clarke_array,
    inverse_park,
    park,
    park_array,
    wrap_angle,
)


@pytest.mark.parametrize("angle", np.linspace(0.0, 2 * math.pi, 13))
def test_clarke_is_amplitude_invariant(angle: float) -> None:
    amp = 3.7
    a = amp * math.cos(angle)
    b = amp * math.cos(angle - 2 * math.pi / 3)
    c = amp * math.cos(angle + 2 * math.pi / 3)
    alpha, beta = clarke(a, b, c)
    assert math.hypot(alpha, beta) == pytest.approx(amp, rel=1e-12)
    assert alpha == pytest.approx(amp * math.cos(angle), abs=1e-12)
    assert beta == pytest.approx(amp * math.sin(angle), abs=1e-12)


def test_clarke_roundtrip_and_zero_sequence_rejection() -> None:
    a, b, c = inverse_clarke(1.2, -0.7)
    assert a + b + c == pytest.approx(0.0, abs=1e-12)
    assert clarke(a, b, c) == pytest.approx((1.2, -0.7))
    # adding a common-mode component does not change alpha-beta
    assert clarke(a + 5.0, b + 5.0, c + 5.0) == pytest.approx((1.2, -0.7))


def test_power_is_three_halves_of_alpha_beta_product() -> None:
    rng = np.random.default_rng(1)
    va, vb = rng.normal(size=2)
    ia, ib = rng.normal(size=2)
    v_abc = inverse_clarke(va, vb)
    i_abc = inverse_clarke(ia, ib)
    p_abc = sum(v * i for v, i in zip(v_abc, i_abc, strict=True))
    assert p_abc == pytest.approx(1.5 * (va * ia + vb * ib), rel=1e-12)


def test_park_roundtrip_and_alignment() -> None:
    theta = 0.83
    d, q = park(math.cos(theta), math.sin(theta), theta)
    assert (d, q) == pytest.approx((1.0, 0.0), abs=1e-12)
    assert inverse_park(*park(0.3, -1.1, theta), theta) == pytest.approx((0.3, -1.1))


def test_array_versions_match_scalar() -> None:
    rng = np.random.default_rng(2)
    a, b, c = rng.normal(size=(3, 50))
    alpha, beta = clarke_array(a, b, c)
    for k in range(50):
        assert (alpha[k], beta[k]) == pytest.approx(clarke(a[k], b[k], c[k]))
    a2, b2, c2 = inverse_clarke_array(alpha, beta)
    np.testing.assert_allclose(a2 + b2 + c2, 0.0, atol=1e-12)
    theta = rng.uniform(-3, 3, size=50)
    d, q = park_array(alpha, beta, theta)
    for k in range(50):
        assert (d[k], q[k]) == pytest.approx(park(alpha[k], beta[k], theta[k]))


def test_wrap_angle_range() -> None:
    for x in np.linspace(-20, 20, 101):
        w = wrap_angle(float(x))
        assert -math.pi <= w < math.pi
        assert math.cos(w) == pytest.approx(math.cos(x))
