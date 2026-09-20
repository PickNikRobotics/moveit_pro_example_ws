# Copyright 2025 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
re-seed, after driving) give a **median** cloud spread of r95 = 0.171 m and yaw_r95 = 3.32 deg.
Converting with ``settled.py``'s own relations (sigma_xy = r95/2.448, sigma_yaw = yaw_r95/1.96)
gives sigma_xy 0.070 m and sigma_yaw 1.70 deg -- the committed ``xy_variance`` 0.0049 and
``yaw_variance`` 0.00088.

The **median** is used, not a higher percentile, and that is a measured choice rather than a
stylistic one. The rule has two halves: a seed must not assert more confidence than the filter has,
and it must not widen the filter's belief either. An earlier revision took the 90th percentile of
the same settled spread (sigma 2.83 deg) on the first half alone; measured over 31 re-seeds that
widened the cloud's heading spread every single time (3.49 -> 5.25 deg), which is the defect this
change exists to remove. Only a seed at or below the median settled spread satisfies both halves.

The rule is bounded in **both** directions, because a seed can be wrong either way. Measured over
367 re-seeds spanning four seed widths, a seed produces a cloud of post_yaw_r95 = 1.88 * sigma_yaw
and post_r95 = 2.41 * sigma_xy, so both bounds are the settled cloud itself expressed as a seed:

* Ceiling — 0.00095 rad^2 (sigma 1.77 deg) and 0.0050 m^2 (sigma 0.071 m): the widest seed whose
  resulting cloud is still no wider than the pooled median settled spread (yaw_r95 3.32 deg,
  r95 0.171 m). Anything wider widens the filter's belief, which is the defect being fixed; the
  behavior defaults (0.0685 / 0.25) fail here by a factor of seventy.
* Floor — 0.00087 rad^2 (sigma 1.69 deg) and 0.0046 m^2 (sigma 0.068 m): the tightest settled
  median any single session exhibited. Below it the seed asserts confidence no session ever showed,
  and that over-confident cloud cannot pull a genuinely offset pose back — the mirror image of the
  failure being fixed.

The committed values (0.00088 / 0.0049) sit just inside that band, and the band is narrow because
the evidence determines the value tightly. A future re-derivation landing outside it should be
re-examined rather than waved through, which is what this test is for.

The purpose is that a future edit fails here instead of silently reintroducing either a five-fold
heading doubt or an over-confident cloud at the start of every navigation Objective.

The Objective XML is a machine-consumed declarative artifact -- the behavior tree the moveit_pro
agent loads -- so it is parsed into elements and attributes and asserted on by meaning, never
grepped for text.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

# The bounds are the settled cloud itself, expressed as a seed. Measured over 367 re-seeds across
# four seed widths, a seed produces a cloud of post_yaw_r95 = 1.88 * sigma_yaw and
# post_r95 = 2.41 * sigma_xy, so a seed may be no wider than the value whose cloud equals the
# POOLED MEDIAN settled spread (3.32 deg / 0.171 m) -- wider than that widens the belief -- and no
# tighter than the value matching the most converged session's own median (1.69 deg / 0.068 m),
# which would assert confidence no session ever exhibited.
MAX_YAW_VARIANCE = (
    0.00095  # sigma 1.77 deg: its cloud is exactly the median settled spread
)
MAX_XY_VARIANCE = 0.0050  # sigma 0.071 m: likewise
MIN_YAW_VARIANCE = 0.00087  # sigma 1.69 deg: the tightest settled median observed
MIN_XY_VARIANCE = 0.0046  # sigma 0.068 m: likewise

OBJECTIVES_DIR = Path(__file__).resolve().parent.parent / "objectives"
NAVIGATION_OBJECTIVES = [
    "navigate_to_clicked_point.xml",
    "navigate_to_clicked_point_with_replanning.xml",
]


def set_initial_pose_actions(objective_path: Path) -> list[ET.Element]:
    root = ET.parse(objective_path).getroot()
    return [
        action for action in root.iter("Action") if action.get("ID") == "SetInitialPose"
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
            f"(expected <= {MAX_XY_VARIANCE} m^2, sigma 0.071 m) -- a seed this wide "
            f"manufactures position doubt the filter did not have"
        )
        assert float(xy_variance) >= MIN_XY_VARIANCE, (
            f"{objective}: xy_variance={xy_variance} is tighter than the median settled spread "
            f"(expected >= {MIN_XY_VARIANCE} m^2, sigma 0.068 m) -- an over-confident seed "
            f"collapses the cloud onto a pose it cannot then correct"
        )
        assert float(yaw_variance) <= MAX_YAW_VARIANCE, (
            f"{objective}: yaw_variance={yaw_variance} is wider than the converged spread "
            f"(expected <= {MAX_YAW_VARIANCE} rad^2, sigma 1.77 deg) -- a seed this wide puts the "
            f"rotationally-ambiguous flipped hypothesis back in play"
        )
        assert float(yaw_variance) >= MIN_YAW_VARIANCE, (
            f"{objective}: yaw_variance={yaw_variance} is tighter than the median settled spread "
            f"(expected >= {MIN_YAW_VARIANCE} rad^2, sigma 1.69 deg) -- an over-confident seed "
            f"collapses the cloud onto a heading it cannot then correct"
        )
