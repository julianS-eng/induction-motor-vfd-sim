"""Power stage: sources and inverter models."""

from vfdsim.power.inverter import (
    AveragedInverter,
    PowerStage,
    Segment,
    dc_link_current,
    pole_to_alpha_beta,
)
from vfdsim.power.sources import GridSource

__all__ = [
    "AveragedInverter",
    "GridSource",
    "PowerStage",
    "Segment",
    "dc_link_current",
    "make_power_stage",
    "pole_to_alpha_beta",
]


def make_power_stage(model: str) -> PowerStage:
    """Instantiate an inverter model by name (``"averaged"``)."""
    if model == "averaged":
        return AveragedInverter()
    raise ValueError(f"unknown inverter model {model!r}")
