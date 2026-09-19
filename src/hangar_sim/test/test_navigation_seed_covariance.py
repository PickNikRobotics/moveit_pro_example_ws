# Copyright 2025 PickNik Inc.
"""Every navigation Objective must re-seed the localizer with a converged covariance.

``SetInitialPose`` is the first action in both navigation Objectives. It re-asserts the pose the
filter already holds, but it overwrites the *covariance*, and when the ports are omitted it takes
the behavior's own defaults of ``xy_variance`` 0.25 m^2 and ``yaw_variance`` 0.0685 rad^2 --
sigma 0.5 m and sigma 15 deg.

The converged filter on this configuration holds sigma_yaw 1.5-1.8 deg and sigma_xy
0.066-0.078 m, measured both from AMCL's own published covariance and independently from the
particle cloud's spread, across two separate sessions. The bounds asserted here are 0.0027 rad^2
(sigma 3.0 deg) and 0.023 m^2 (sigma 0.15 m): roughly twice the measured converged spread, so
they admit the committed values (0.00085 / 0.0052) while rejecting the behavior defaults
(0.0685 / 0.25).

The purpose is that a future edit dropping back to those defaults fails here instead of silently
reintroducing a six-fold heading doubt at the start of every navigation Objective.

The Objective XML is a machine-consumed declarative artifact -- the behavior tree the moveit_pro
agent loads -- so it is parsed into elements and attributes and asserted on by meaning, never
grepped for text.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

MAX_YAW_VARIANCE = 0.0027
MAX_XY_VARIANCE = 0.023

OBJECTIVES_DIR = Path(__file__).resolve().parent.parent / "objectives"
NAVIGATION_OBJECTIVES = [
    "navigate_to_clicked_point.xml",
    "navigate_to_clicked_point_with_replanning.xml",
]


def set_initial_pose_actions(objective_path: Path) -> list[ET.Element]:
    root = ET.parse(objective_path).getroot()
    return [
        action
        for action in root.iter("Action")
        if action.get("ID") == "SetInitialPose"
    ]


@pytest.mark.parametrize("objective", NAVIGATION_OBJECTIVES)
def test_navigation_objective_seeds_with_a_converged_covariance(objective: str) -> None:
    objective_path = OBJECTIVES_DIR / objective
    assert objective_path.is_file(), f"{objective_path} does not exist"

    actions = set_initial_pose_actions(objective_path)
    assert actions, f"{objective} has no SetInitialPose action"

    for action in actions:
        xy_variance = action.get("xy_variance")
        yaw_variance = action.get("yaw_variance")

        assert xy_variance is not None, (
            f"{objective}: SetInitialPose omits xy_variance, so it falls back to the behavior "
            f"default of 0.25 m^2 (sigma 0.5 m)"
        )
        assert yaw_variance is not None, (
            f"{objective}: SetInitialPose omits yaw_variance, so it falls back to the behavior "
            f"default of 0.0685 rad^2 (sigma 15 deg)"
        )

        assert 0.0 < float(xy_variance) <= MAX_XY_VARIANCE, (
            f"{objective}: xy_variance={xy_variance} is not a converged spread "
            f"(expected <= {MAX_XY_VARIANCE} m^2, sigma 0.15 m)"
        )
        assert 0.0 < float(yaw_variance) <= MAX_YAW_VARIANCE, (
            f"{objective}: yaw_variance={yaw_variance} is not a converged spread "
            f"(expected <= {MAX_YAW_VARIANCE} rad^2, sigma 3.0 deg)"
        )
