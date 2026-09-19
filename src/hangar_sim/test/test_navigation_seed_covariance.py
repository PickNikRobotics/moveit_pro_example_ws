# Copyright 2025 PickNik Inc.
"""Every navigation Objective must re-seed the localizer with a converged covariance.

``SetInitialPose`` is the first action in both navigation Objectives. It re-asserts the pose the
filter already holds, but it overwrites the *covariance*, and when the ports are omitted it takes
the behavior's own defaults of ``xy_variance`` 0.25 m^2 and ``yaw_variance`` 0.0685 rad^2 --
sigma 0.5 m and sigma 15 deg.

The bound comes from the settled **particle cloud's own spread**, because that is the quantity a
re-seed actually replaces. Two instruments are available and they do not agree: AMCL's published
covariance reads tighter (sigma_yaw 1.49-1.84 deg, sigma_xy 0.066-0.078 m) than the particle set
it summarises, being a weighted summary of it. The cloud spread is used here rather than treating
the two as corroborating each other.

Settled samples pooled across two independent sessions (n=4199; stopped, more than 15 s after any
re-seed, after driving) give a 90th-percentile cloud spread of r95 = 0.324 m and yaw_r95 =
5.55 deg. Converting with ``settled.py``'s own relations (sigma_xy = r95/2.448, sigma_yaw =
yaw_r95/1.96) gives sigma_xy 0.132 m and sigma_yaw 2.83 deg -- the committed
``xy_variance`` 0.0175 and ``yaw_variance`` 0.00245. The 90th percentile is used for *both*
quantities, since the rule being applied is that a seed must not assert a belief tighter than what
the filter actually holds when converged.

The rule is bounded in **both** directions, because a seed can be wrong either way:

* Ceiling — 0.0027 rad^2 (sigma 3.0 deg) and 0.023 m^2 (sigma 0.15 m), just above the p90 settled
  spread. Too wide manufactures doubt the filter did not have; the behavior defaults
  (0.0685 / 0.25) fail here.
* Floor — 0.00088 rad^2 (sigma 1.70 deg) and 0.0049 m^2 (sigma 0.070 m), the **median** settled
  spread (pooled medians r95 0.171 m and yaw_r95 3.32 deg over the same n=4199). A seed at or
  below the median converged spread asserts more confidence than the filter typically holds, and
  that over-confident cloud cannot pull a genuinely offset pose back — the mirror image of the
  failure being fixed. The superseded 0.0052 / 0.00085 fails here.

The committed values sit at the p90, i.e. at the ceiling end of that band.

The purpose is that a future edit fails here instead of silently reintroducing either a five-fold
heading doubt or an over-confident cloud at the start of every navigation Objective.

The Objective XML is a machine-consumed declarative artifact -- the behavior tree the moveit_pro
agent loads -- so it is parsed into elements and attributes and asserted on by meaning, never
grepped for text.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

MAX_YAW_VARIANCE = 0.0027
MAX_XY_VARIANCE = 0.023
MIN_YAW_VARIANCE = 0.00088
MIN_XY_VARIANCE = 0.0049

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

        assert float(xy_variance) <= MAX_XY_VARIANCE, (
            f"{objective}: xy_variance={xy_variance} is wider than the converged spread "
            f"(expected <= {MAX_XY_VARIANCE} m^2, sigma 0.15 m) -- a seed this wide "
            f"manufactures position doubt the filter did not have"
        )
        assert float(xy_variance) >= MIN_XY_VARIANCE, (
            f"{objective}: xy_variance={xy_variance} is tighter than the median settled spread "
            f"(expected >= {MIN_XY_VARIANCE} m^2, sigma 0.070 m) -- an over-confident seed "
            f"collapses the cloud onto a pose it cannot then correct"
        )
        assert float(yaw_variance) <= MAX_YAW_VARIANCE, (
            f"{objective}: yaw_variance={yaw_variance} is wider than the converged spread "
            f"(expected <= {MAX_YAW_VARIANCE} rad^2, sigma 3.0 deg) -- a seed this wide puts the "
            f"rotationally-ambiguous flipped hypothesis back in play"
        )
        assert float(yaw_variance) >= MIN_YAW_VARIANCE, (
            f"{objective}: yaw_variance={yaw_variance} is tighter than the median settled spread "
            f"(expected >= {MIN_YAW_VARIANCE} rad^2, sigma 1.70 deg) -- an over-confident seed "
            f"collapses the cloud onto a heading it cannot then correct"
        )
