# Copyright 2026 PickNik Inc.
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

"""The Objectives the integration test will run, checked without starting the stack."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import objectives_integration_test as ci
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
GOAL_PORTS = {
    "NavigateToPoseAction": "pose_stamped",
    "Drive Base using Nav2": "goal_pose",
    "PushBackVector": "element",
}


def _ci_objectives() -> dict[str, Path]:
    """Objective ID -> XML path for every Objective the integration test executes."""
    params = ci.get_objective_pytest_params(
        "market_sim", ci.cancel_objectives, ci.skip_objectives
    )
    return {p.values[0]: Path(p.id) for p in params if not p.marks and not p.values[1]}


def _route_length(root: ET.ElementTree) -> float:
    """Straight-line length through every navigation goal, from the keyframe's origin."""
    created: dict[str, tuple[float, float]] = {}
    here, length = (0.0, 0.0), 0.0
    for node in root.iter():
        xyz = node.get("position_xyz", "{")
        if node.get("ID") == "CreatePoseStamped" and not xyz.startswith("{"):
            x, y = map(float, xyz.split(";")[:2])
            created[node.get("pose_stamped")] = (x, y)
        goal = created.get(node.get(GOAL_PORTS.get(node.get("ID"), ""), ""))
        if goal is not None:
            length += math.dist(here, goal)
            here = goal
    return length


def test_ci_objectives_fit_their_budget_at_the_speed_limit() -> None:
    """A run Objective's route from the start pose must fit inside its execute timeout."""
    params = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())
    speed = params["controller_server"]["ros__parameters"]["FollowPath"]["vx_max"]
    for objective_id, path in _ci_objectives().items():
        budget = ci.EXECUTE_TIMEOUT_OVERRIDES_S.get(
            objective_id, ci.DEFAULT_OBJECTIVE_WAIT_S
        )
        nominal = _route_length(ET.parse(path)) / speed
        assert nominal < budget, (
            f"{objective_id} needs at least {nominal:.0f} s at {speed} m/s "
            f"but has {budget:.0f} s"
        )


def test_ci_objectives_do_not_need_a_docked_start() -> None:
    """Every test resets to the keyframe, which is not at the D4 dock."""
    for objective_id, path in _ci_objectives().items():
        assert (
            ET.parse(path).find(".//SubTree[@ID='Undock From D4']") is None
        ), objective_id
