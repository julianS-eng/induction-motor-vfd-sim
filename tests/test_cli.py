from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from vfdsim import __version__
from vfdsim.cli import build_parser, main


def test_info(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["info"]) == 0
    out = capsys.readouterr().out
    assert "WARNING: assumed" in out
    assert "n_sync=1800" in out


def test_simulate_saves_outputs(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    npz = tmp_path / "run.npz"
    png = tmp_path / "run.png"
    assert main(["simulate", "--t-end", "0.05", "--out", str(npz), "--plot", str(png)]) == 0
    assert "final speed" in capsys.readouterr().out
    data = np.load(npz)
    assert data["t"].shape == (500,)
    assert png.stat().st_size > 0


def test_version(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--version"])
    assert __version__ in capsys.readouterr().out


@pytest.mark.slow
def test_phase1_experiments_quick(tmp_path: Path) -> None:
    assert main(["experiments", "phase1", "--quick", "--out", str(tmp_path)]) == 0
    assert (tmp_path / "results" / "phase1.md").exists()
    assert (tmp_path / "results" / "phase1_metrics.json").exists()
    assert len(list((tmp_path / "img").glob("phase1_*.png"))) == 5
