from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from vfdsim.config import (
    DEFAULT_CONFIG_DIR,
    ConfigError,
    DriveConfig,
    config_from_mapping,
    load_config,
    motor_from_mapping,
)
from vfdsim.motor import MotorParameters


def _motor_yaml() -> dict[str, object]:
    with (DEFAULT_CONFIG_DIR / "motors" / "educational_175w_4pole.yaml").open() as fh:
        data: dict[str, object] = yaml.safe_load(fh)
    return data


def test_default_config_loads(cfg: DriveConfig) -> None:
    assert cfg.motor.nameplate.poles == 4
    assert cfg.inverter.control_period_s == pytest.approx(1e-4)
    assert cfg.modulation.method == "spwm"
    assert cfg.speed_reference_rpm[0] == (0.0, 1500.0)
    assert cfg.source is not None


def test_motor_parameters_are_flagged_as_assumed(motor: MotorParameters) -> None:
    assert motor.is_assumed
    assert set(motor.provenance) == {"nameplate", "equivalent_circuit", "mechanical"}


def test_derived_nameplate_quantities(motor: MotorParameters) -> None:
    npl = motor.nameplate
    assert npl.pole_pairs == 2
    assert npl.synchronous_speed_rpm == pytest.approx(1800.0)
    assert npl.rated_slip == pytest.approx(130.0 / 1800.0)
    assert npl.rated_torque_nm == pytest.approx(175.0 / (1670.0 * 2 * 3.141592653589793 / 60))
    assert motor.l_s == pytest.approx(motor.l_ls + motor.l_m)
    assert 0.0 < motor.sigma < 1.0


def test_unknown_key_is_rejected() -> None:
    data = _motor_yaml()
    data["nameplate"] = {**data["nameplate"], "rated_torgue": 1.0}  # type: ignore[dict-item]
    with pytest.raises(ConfigError, match="unknown key"):
        motor_from_mapping(data)


def test_invalid_value_is_rejected() -> None:
    data = _motor_yaml()
    ec = dict(data["equivalent_circuit"])  # type: ignore[call-overload]
    ec["stator_resistance_ohm"] = -1.0
    data["equivalent_circuit"] = ec
    with pytest.raises(ConfigError, match="positive"):
        motor_from_mapping(data)


def test_rated_speed_above_synchronous_is_rejected() -> None:
    data = _motor_yaml()
    data["nameplate"] = {**data["nameplate"], "rated_speed_rpm": 1850.0}  # type: ignore[dict-item]
    with pytest.raises(ConfigError, match="below synchronous"):
        motor_from_mapping(data)


def test_inline_motor_and_bad_load(tmp_path: Path) -> None:
    data = {"motor": _motor_yaml(), "load": {"components": [{"type": "flywheel"}]}}
    with pytest.raises(ConfigError, match="load"):
        config_from_mapping(data)
    ok = config_from_mapping({"motor": _motor_yaml()})
    assert ok.load_components == ()


def test_config_file_with_relative_motor_path(tmp_path: Path) -> None:
    motor_file = tmp_path / "m.yaml"
    motor_file.write_text(yaml.safe_dump(_motor_yaml()))
    cfg_file = tmp_path / "drive.yaml"
    cfg_file.write_text(
        yaml.safe_dump(
            {
                "motor": "m.yaml",
                "vf_control": {"boost_fraction": 0.1, "current_limit_a": None},
                "reference": {"speed_rpm": [[0.0, 900.0], [1.0, 600.0]]},
            }
        )
    )
    cfg = load_config(cfg_file)
    assert cfg.vf_control.boost_fraction == 0.1
    assert cfg.vf_control.current_limit_a is None
    assert cfg.speed_reference_rpm == ((0.0, 900.0), (1.0, 600.0))


def test_bad_section_values() -> None:
    with pytest.raises(ConfigError, match="vf_control"):
        config_from_mapping({"motor": _motor_yaml(), "vf_control": {"profile": "cubic"}})
    with pytest.raises(ConfigError, match="reference"):
        config_from_mapping({"motor": _motor_yaml(), "reference": {"speed_rpm": [1.0]}})
    with pytest.raises(ConfigError, match="motor"):
        config_from_mapping({"motor": 3})
