# Copyright 2026 PickNik Inc.
# All rights reserved.

"""Offline evidence-script argument validation."""

from pathlib import Path
import os
import subprocess
import sys


def test_sequence_smoke_rejects_nan_frame_rate(tmp_path: Path) -> None:
    """Synthetic timestamps cannot be generated from a non-finite frame rate."""
    left = tmp_path / "image_left"
    right = tmp_path / "image_right"
    left.mkdir()
    right.mkdir()
    (left / "000000_left.png").touch()
    (right / "000000_right.png").touch()
    script = Path(__file__).parents[1] / "scripts" / "run_sequence_smoke.py"

    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--runtime",
            "eager",
            "--dataset-root",
            str(tmp_path),
            "--output",
            str(tmp_path / "result.json"),
            "--fx",
            "320",
            "--fy",
            "320",
            "--cx",
            "320",
            "--cy",
            "240",
            "--baseline-m",
            "0.25",
            "--frame-rate-hz",
            "nan",
        ],
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "PYTHONPATH": str(Path(__file__).parents[1])},
    )

    assert completed.returncode != 0
    assert "frame-rate-hz must be finite and positive" in completed.stderr
