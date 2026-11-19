# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Save paired camera trajectories and a finish close-up as a headless SVG."""

from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from trajectory_metrics import QualificationResult


_SURFACE = "#fcfcfb"
_INK = "#172b3a"
_SERIES = (("Ground truth", "#2a78d6", False), ("Stereo VO", "#eb6834", True))


def save_plot(result: QualificationResult, path: Path) -> None:
    """Save the scorer's world-XY paths, without fitting or overwriting a file."""
    if path.suffix.lower() != ".svg":
        raise ValueError("--plot must name an .svg file")
    paths = [
        np.array([pose.translation[:2] for pose in samples])
        for samples in (result.truth_world, result.vo_world)
    ]
    root = ET.Element(
        "svg", xmlns="http://www.w3.org/2000/svg", viewBox="0 0 1100 800", role="img"
    )
    ET.SubElement(root, "title").text = "Stereo VO against camera ground truth"
    ET.SubElement(root, "desc").text = (
        "Simulation world XY in metres. Both paths track the left camera. "
        "VO is aligned at the first motion pose only; no scale or trajectory fitting."
    )
    ET.SubElement(root, "rect", width="1100", height="800", fill=_SURFACE)

    def text(x: float, y: float, value: str, size: int = 16) -> None:
        ET.SubElement(
            root,
            "text",
            x=str(x),
            y=str(y),
            fill=_INK,
            **{"font-family": "sans-serif", "font-size": str(size)},
        ).text = value

    def panel(trajectories: list[np.ndarray], x: int, y: int, size: int) -> None:
        points = np.concatenate(trajectories)
        span = max(float(np.ptp(points, axis=0).max()), 0.1) * 1.2
        lower = (points.min(axis=0) + points.max(axis=0)) / 2 - span / 2
        for index in range(5):
            fraction = index / 4
            for attributes in (
                {
                    "x1": x + fraction * size,
                    "y1": y,
                    "x2": x + fraction * size,
                    "y2": y + size,
                },
                {
                    "x1": x,
                    "y1": y + fraction * size,
                    "x2": x + size,
                    "y2": y + fraction * size,
                },
            ):
                ET.SubElement(
                    root,
                    "line",
                    **{key: str(value) for key, value in attributes.items()},
                    stroke="#e2e5e7",
                )
            text(
                x + fraction * size - 16,
                y + size + 22,
                f"{lower[0] + fraction * span:.2f}",
                12,
            )
            text(
                x - 48,
                y + fraction * size + 4,
                f"{lower[1] + (1 - fraction) * span:.2f}",
                12,
            )
        for xy, (_, color, dashed) in zip(trajectories, _SERIES, strict=True):
            pixels = (xy - lower) / span * size
            pixels[:, 0] += x
            pixels[:, 1] = y + size - pixels[:, 1]
            attributes = {"stroke-dasharray": "7 5"} if dashed else {}
            ET.SubElement(
                root,
                "polyline",
                points=" ".join(f"{px:.3f},{py:.3f}" for px, py in pixels),
                fill="none",
                stroke=color,
                **{"stroke-width": "2", "stroke-linejoin": "round"},
                **attributes,
            )
            px, py = pixels[-1]
            ET.SubElement(
                root,
                "circle",
                cx=str(px),
                cy=str(py),
                r="4",
                fill=color,
                stroke=_SURFACE,
                **{"stroke-width": "2"},
            )
        text(x + size / 2 - 15, y + size + 44, "X (m)", 12)
        text(x - 48, y - 12, "Y (m)", 12)

    text(70, 36, "Stereo VO against camera ground truth", 24)
    text(70, 65, "Start-pose alignment only; no scale or trajectory fitting.")
    for index, (label, color, dashed) in enumerate(_SERIES):
        x = 70 + index * 220
        attributes = {"stroke-dasharray": "7 5"} if dashed else {}
        ET.SubElement(
            root,
            "line",
            x1=str(x),
            y1="90",
            x2=str(x + 30),
            y2="90",
            stroke=color,
            **{"stroke-width": "2"},
            **attributes,
        )
        text(x + 40, 95, label)
    panel(paths, 70, 135, 600)
    text(780, 155, f"RMSE: {result.metrics['ate_rmse_m']:.3f} m", 20)
    text(780, 195, f"Final error: {result.metrics['final_drift_m']:.3f} m", 20)
    text(780, 225, "Position errors are three-dimensional.", 13)
    text(780, 295, "Close-up at finish", 18)
    panel([xy[-min(18, len(xy)) :] for xy in paths], 780, 335, 250)
    text(780, 675, "Solid: ground truth", 14)
    text(780, 700, "Dashed: stereo VO", 14)
    text(
        70,
        795,
        "Both paths show the left camera in mj_world; no terrain rendering is included.",
        13,
    )
    with path.open("xb") as output:
        ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
        output.write(b"\n")
