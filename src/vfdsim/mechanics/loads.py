"""Mechanical load models.

Every load contributes an inertia (added to the rotor inertia) and an opposing
torque ``T_L(t, ω_m)`` (positive values oppose positive rotation). Passive
loads (friction, constant-torque conveyors, fans) change sign with the speed;
the sign function is smoothed with ``tanh(ω/ω_ε)`` so that the ODE right-hand
side stays continuous at standstill. Active loads (hoists, test benches in
torque mode) act independently of the direction of rotation.

These components map directly onto the operating modes of a four-quadrant
dynamometer (constant torque, fan/quadratic, friction, flywheel), which is how
the load is emulated on the LabVolt/Festo bench.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

#: Default speed (rad/s) used to smooth the sign of passive loads.
DEFAULT_OMEGA_EPS = 0.5


def smooth_sign(omega: float, omega_eps: float = DEFAULT_OMEGA_EPS) -> float:
    """Continuous approximation of ``sign(ω)`` using ``tanh(ω/ω_ε)``."""
    return math.tanh(omega / omega_eps)


@runtime_checkable
class Load(Protocol):
    """Protocol implemented by every mechanical load."""

    @property
    def inertia(self) -> float:
        """Moment of inertia added to the shaft, kg·m²."""
        ...

    def torque(self, t: float, omega_m: float) -> float:
        """Opposing torque (N·m) at time ``t`` (s) and speed ``omega_m`` (rad/s)."""
        ...


@dataclass(frozen=True)
class InertiaLoad:
    """Pure inertia (flywheel, coupling, dynamometer rotor)."""

    inertia_kgm2: float

    def __post_init__(self) -> None:
        if self.inertia_kgm2 < 0.0:
            raise ValueError("inertia must be non-negative")

    @property
    def inertia(self) -> float:
        """Added inertia, kg·m²."""
        return self.inertia_kgm2

    def torque(self, t: float, omega_m: float) -> float:
        """A pure inertia exerts no steady torque."""
        return 0.0


@dataclass(frozen=True)
class FrictionLoad:
    """Viscous plus Coulomb friction: ``T = B ω + T_c sign(ω)``."""

    viscous_nm_s_per_rad: float = 0.0
    coulomb_nm: float = 0.0
    omega_eps: float = DEFAULT_OMEGA_EPS

    def __post_init__(self) -> None:
        if self.viscous_nm_s_per_rad < 0.0 or self.coulomb_nm < 0.0:
            raise ValueError("friction coefficients must be non-negative")

    @property
    def inertia(self) -> float:
        """Friction adds no inertia."""
        return 0.0

    def torque(self, t: float, omega_m: float) -> float:
        """Friction torque."""
        return self.viscous_nm_s_per_rad * omega_m + self.coulomb_nm * smooth_sign(
            omega_m, self.omega_eps
        )


@dataclass(frozen=True)
class ConstantTorqueLoad:
    """Constant-torque load (conveyor, positive-displacement pump, hoist).

    Parameters
    ----------
    torque_nm:
        Load torque magnitude.
    passive:
        If ``True`` the torque always opposes rotation (conveyor); if ``False``
        it is an active torque with a fixed direction (hoist).
    """

    torque_nm: float
    passive: bool = True
    omega_eps: float = DEFAULT_OMEGA_EPS

    @property
    def inertia(self) -> float:
        """No added inertia."""
        return 0.0

    def torque(self, t: float, omega_m: float) -> float:
        """Load torque."""
        if self.passive:
            return self.torque_nm * smooth_sign(omega_m, self.omega_eps)
        return self.torque_nm


@dataclass(frozen=True)
class QuadraticLoad:
    """Fan/centrifugal-pump load ``T = k ω |ω|`` defined by one operating point.

    Parameters
    ----------
    torque_at_speed_nm:
        Torque at the reference speed.
    speed_rpm:
        Reference speed in r/min.
    """

    torque_at_speed_nm: float
    speed_rpm: float

    def __post_init__(self) -> None:
        if self.speed_rpm <= 0.0:
            raise ValueError("reference speed must be positive")

    @property
    def coefficient(self) -> float:
        """Quadratic coefficient ``k`` in N·m·s²/rad²."""
        w = self.speed_rpm * 2.0 * math.pi / 60.0
        return self.torque_at_speed_nm / (w * w)

    @property
    def inertia(self) -> float:
        """No added inertia."""
        return 0.0

    def torque(self, t: float, omega_m: float) -> float:
        """Load torque."""
        return self.coefficient * omega_m * abs(omega_m)


@dataclass(frozen=True)
class StepLoad:
    """Load torque that steps from ``torque_before_nm`` to ``torque_after_nm`` at ``time_s``."""

    time_s: float
    torque_after_nm: float
    torque_before_nm: float = 0.0
    passive: bool = True
    omega_eps: float = DEFAULT_OMEGA_EPS

    @property
    def inertia(self) -> float:
        """No added inertia."""
        return 0.0

    def torque(self, t: float, omega_m: float) -> float:
        """Load torque."""
        value = self.torque_after_nm if t >= self.time_s else self.torque_before_nm
        if self.passive:
            return value * smooth_sign(omega_m, self.omega_eps)
        return value


@dataclass(frozen=True)
class CompositeLoad:
    """Sum of several load components."""

    components: tuple[Load, ...] = field(default_factory=tuple)

    @property
    def inertia(self) -> float:
        """Total added inertia."""
        return sum(c.inertia for c in self.components)

    def torque(self, t: float, omega_m: float) -> float:
        """Total opposing torque."""
        total = 0.0
        for c in self.components:
            total += c.torque(t, omega_m)
        return total

    def with_component(self, component: Load) -> CompositeLoad:
        """Return a copy with one more component."""
        return CompositeLoad((*self.components, component))


_FACTORIES: dict[str, tuple[type[Any], dict[str, str]]] = {
    # type name -> (class, {yaml key: constructor argument})
    "inertia": (InertiaLoad, {"inertia_kgm2": "inertia_kgm2"}),
    "friction": (
        FrictionLoad,
        {"viscous_nm_s_per_rad": "viscous_nm_s_per_rad", "coulomb_nm": "coulomb_nm"},
    ),
    "constant": (ConstantTorqueLoad, {"torque_nm": "torque_nm", "passive": "passive"}),
    "quadratic": (
        QuadraticLoad,
        {"torque_at_speed_nm": "torque_at_speed_nm", "speed_rpm": "speed_rpm"},
    ),
    "step": (
        StepLoad,
        {
            "time_s": "time_s",
            "torque_before_nm": "torque_before_nm",
            "torque_after_nm": "torque_after_nm",
            "passive": "passive",
        },
    ),
}


def load_from_spec(spec: Mapping[str, Any]) -> Load:
    """Build one load component from a mapping with a ``type`` key.

    Examples
    --------
    >>> load_from_spec({"type": "constant", "torque_nm": 0.5}).torque(0.0, 100.0)
    0.5
    """
    data = dict(spec)
    kind = data.pop("type", None)
    if kind not in _FACTORIES:
        raise ValueError(f"unknown load type {kind!r}; expected one of {sorted(_FACTORIES)}")
    cls, keys = _FACTORIES[kind]
    unknown = set(data) - set(keys)
    if unknown:
        raise ValueError(f"unknown keys for load type {kind!r}: {sorted(unknown)}")
    kwargs = {keys[k]: v for k, v in data.items()}
    load: Load = cls(**kwargs)
    return load


def composite_from_specs(specs: Iterable[Mapping[str, Any]]) -> CompositeLoad:
    """Build a :class:`CompositeLoad` from a list of component specifications."""
    return CompositeLoad(tuple(load_from_spec(s) for s in specs))


def total_inertia(motor_inertia: float, loads: Sequence[Load] | Load) -> float:
    """Total shaft inertia of motor + load(s)."""
    if isinstance(loads, Sequence):
        return motor_inertia + sum(ld.inertia for ld in loads)
    return motor_inertia + loads.inertia
