"""YAML configuration loading with strict validation.

A drive configuration file references a motor file (so motors can be swapped
without touching the scenario) and contains one section per subsystem. Unknown
keys are rejected to catch typos early. See ``configs/default.yaml``.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeVar

import yaml

from vfdsim.control.vf import VFSettings
from vfdsim.motor.parameters import MotorParameters, Nameplate

T = TypeVar("T")

#: Directory with the configuration files shipped with the repository.
DEFAULT_CONFIG_DIR = Path(__file__).resolve().parents[2] / "configs"


class ConfigError(ValueError):
    """Raised when a configuration file is invalid."""


def _check_keys(data: Mapping[str, Any], allowed: set[str], section: str) -> None:
    unknown = set(data) - allowed
    if unknown:
        raise ConfigError(f"unknown key(s) in '{section}': {sorted(unknown)}")


def _build(cls: type[T], data: Mapping[str, Any] | None, section: str) -> T:
    """Instantiate a dataclass from a mapping, rejecting unknown keys."""
    data = dict(data or {})
    names = {f.name for f in dataclasses.fields(cls)}  # type: ignore[arg-type]
    _check_keys(data, names, section)
    try:
        return cls(**data)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"invalid '{section}' section: {exc}") from exc


# ------------------------------------------------------------------- sections
@dataclass(frozen=True)
class DCLinkConfig:
    """DC-link settings (Phase 1: ideal bus)."""

    nominal_voltage_v: float = 285.0

    def __post_init__(self) -> None:
        if self.nominal_voltage_v <= 0.0:
            raise ValueError("nominal_voltage_v must be positive")


@dataclass(frozen=True)
class InverterConfig:
    """Inverter model selection."""

    model: str = "averaged"
    switching_frequency_hz: float = 10_000.0

    def __post_init__(self) -> None:
        if self.switching_frequency_hz <= 0.0:
            raise ValueError("switching_frequency_hz must be positive")

    @property
    def control_period_s(self) -> float:
        """Control period = PWM period (single update per carrier period)."""
        return 1.0 / self.switching_frequency_hz


@dataclass(frozen=True)
class ModulationConfig:
    """Modulation strategy."""

    method: str = "spwm"


@dataclass(frozen=True)
class SimulationConfig:
    """Numerical settings of the simulation."""

    t_end_s: float = 3.0
    plant_substeps: int = 1
    computation_delay: bool = True
    record_every: int = 1

    def __post_init__(self) -> None:
        if self.t_end_s <= 0.0:
            raise ValueError("t_end_s must be positive")
        if self.plant_substeps < 1 or self.record_every < 1:
            raise ValueError("plant_substeps and record_every must be >= 1")


@dataclass(frozen=True)
class SensorConfig:
    """Measurement chain."""

    current_noise_std_a: float = 0.0
    seed: int = 0


@dataclass(frozen=True)
class DriveConfig:
    """Complete drive scenario."""

    motor: MotorParameters
    dc_link: DCLinkConfig = field(default_factory=DCLinkConfig)
    inverter: InverterConfig = field(default_factory=InverterConfig)
    modulation: ModulationConfig = field(default_factory=ModulationConfig)
    vf_control: VFSettings = field(default_factory=VFSettings)
    load_components: tuple[Mapping[str, Any], ...] = ()
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    speed_reference_rpm: tuple[tuple[float, float], ...] = ((0.0, 0.0),)
    source: Path | None = None


# --------------------------------------------------------------------- motor
def motor_from_mapping(data: Mapping[str, Any]) -> MotorParameters:
    """Build :class:`MotorParameters` from the structure of a motor YAML file."""
    allowed = {"name", "provenance", "nameplate", "equivalent_circuit", "mechanical"}
    _check_keys(data, allowed, "motor")
    for key in ("nameplate", "equivalent_circuit", "mechanical"):
        if key not in data:
            raise ConfigError(f"motor file is missing the '{key}' section")
    nameplate = _build(Nameplate, data["nameplate"], "motor.nameplate")
    ec = dict(data["equivalent_circuit"])
    ec_keys = {
        "stator_resistance_ohm": "r_s",
        "rotor_resistance_ohm": "r_r",
        "stator_leakage_inductance_h": "l_ls",
        "rotor_leakage_inductance_h": "l_lr",
        "magnetizing_inductance_h": "l_m",
    }
    _check_keys(ec, set(ec_keys), "motor.equivalent_circuit")
    mech = dict(data["mechanical"])
    mech_keys = {
        "rotor_inertia_kgm2": "inertia",
        "viscous_friction_nm_s_per_rad": "viscous_friction",
    }
    _check_keys(mech, set(mech_keys), "motor.mechanical")
    missing = (set(ec_keys) - set(ec)) | (set(mech_keys) - set(mech))
    if missing:
        raise ConfigError(f"motor file is missing: {sorted(missing)}")
    provenance_raw = dict(data.get("provenance") or {})
    provenance = {k: str(v) for k, v in provenance_raw.items() if k != "notes"}
    try:
        return MotorParameters(
            **{ec_keys[k]: float(v) for k, v in ec.items()},
            **{mech_keys[k]: float(v) for k, v in mech.items()},
            nameplate=nameplate,
            name=str(data.get("name", "motor")),
            provenance=provenance,
        )
    except ValueError as exc:
        raise ConfigError(str(exc)) from exc


def _read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ConfigError(f"{path} does not contain a mapping")
    return data


def load_motor(path: str | Path) -> MotorParameters:
    """Load a motor YAML file."""
    return motor_from_mapping(_read_yaml(Path(path)))


def default_motor() -> MotorParameters:
    """The assumed educational motor shipped in ``configs/motors``."""
    return load_motor(DEFAULT_CONFIG_DIR / "motors" / "educational_175w_4pole.yaml")


# --------------------------------------------------------------------- drive
def config_from_mapping(data: Mapping[str, Any], base_dir: Path | None = None) -> DriveConfig:
    """Build a :class:`DriveConfig` from a parsed YAML mapping."""
    allowed = {
        "motor",
        "dc_link",
        "inverter",
        "modulation",
        "vf_control",
        "load",
        "simulation",
        "sensors",
        "reference",
    }
    _check_keys(data, allowed, "root")
    motor_entry = data.get("motor")
    if isinstance(motor_entry, str):
        motor_path = Path(motor_entry)
        if not motor_path.is_absolute() and base_dir is not None:
            motor_path = base_dir / motor_path
        motor = load_motor(motor_path)
    elif isinstance(motor_entry, Mapping):
        motor = motor_from_mapping(motor_entry)
    else:
        raise ConfigError("'motor' must be a path to a motor file or an inline mapping")

    load_section = dict(data.get("load") or {})
    _check_keys(load_section, {"components"}, "load")
    components = tuple(dict(c) for c in load_section.get("components") or ())

    ref_section = dict(data.get("reference") or {})
    _check_keys(ref_section, {"speed_rpm"}, "reference")
    ref_pairs = ref_section.get("speed_rpm") or [[0.0, 0.0]]
    try:
        speed_ref = tuple((float(p[0]), float(p[1])) for p in ref_pairs)
    except (TypeError, IndexError, ValueError) as exc:
        raise ConfigError("reference.speed_rpm must be a list of [time_s, rpm] pairs") from exc

    cfg = DriveConfig(
        motor=motor,
        dc_link=_build(DCLinkConfig, data.get("dc_link"), "dc_link"),
        inverter=_build(InverterConfig, data.get("inverter"), "inverter"),
        modulation=_build(ModulationConfig, data.get("modulation"), "modulation"),
        vf_control=_build(VFSettings, data.get("vf_control"), "vf_control"),
        load_components=components,
        simulation=_build(SimulationConfig, data.get("simulation"), "simulation"),
        sensors=_build(SensorConfig, data.get("sensors"), "sensors"),
        speed_reference_rpm=speed_ref,
    )
    # Validate load specs eagerly.
    from vfdsim.mechanics.loads import composite_from_specs

    try:
        composite_from_specs(cfg.load_components)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"invalid 'load' section: {exc}") from exc
    return cfg


def load_config(path: str | Path) -> DriveConfig:
    """Load a drive configuration YAML file."""
    p = Path(path)
    cfg = config_from_mapping(_read_yaml(p), base_dir=p.resolve().parent)
    return dataclasses.replace(cfg, source=p)


def default_config() -> DriveConfig:
    """The default scenario ``configs/default.yaml``."""
    return load_config(DEFAULT_CONFIG_DIR / "default.yaml")
