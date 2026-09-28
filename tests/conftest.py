from __future__ import annotations

import dataclasses

import pytest

from vfdsim.config import DriveConfig, default_config
from vfdsim.motor import EquivalentCircuit, InductionMotorModel, MotorParameters


@pytest.fixture(scope="session")
def cfg() -> DriveConfig:
    return default_config()


@pytest.fixture(scope="session")
def motor(cfg: DriveConfig) -> MotorParameters:
    return cfg.motor


@pytest.fixture(scope="session")
def frictionless_motor(motor: MotorParameters) -> MotorParameters:
    return dataclasses.replace(motor, viscous_friction=0.0)


@pytest.fixture(scope="session")
def ec(motor: MotorParameters) -> EquivalentCircuit:
    return EquivalentCircuit(motor)


@pytest.fixture(scope="session")
def dq(motor: MotorParameters) -> InductionMotorModel:
    return InductionMotorModel(motor)
