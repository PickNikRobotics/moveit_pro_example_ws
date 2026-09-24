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

"""Keep named gripper commands consistent with the configured Objectives."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


@pytest.mark.parametrize("side", ["left", "right"])
def test_named_gripper_states_match_objectives(side: str) -> None:
    """Named opening must open the jaws, and closing must move the opposite way."""
    package = Path(__file__).resolve().parents[1]
    srdf = ET.parse(package / "config/moveit/picknik_kinova_gen3_base.srdf")
    named = {}
    objective = {}
    for command in ("open", "close"):
        joint = srdf.find(
            f".//group_state[@name='{command}'][@group='gripper']/"
            f"joint[@name='robotiq_85_{side}_knuckle_joint']"
        )
        assert joint is not None
        named[command] = float(joint.attrib["value"])
        tree = ET.parse(package / "objectives" / f"{command}_gripper.xml")
        action = tree.find(".//Action[@ID='MoveGripperAction']")
        assert action is not None
        objective[command] = float(action.attrib["position"])

    assert named["open"] == pytest.approx(objective["open"])
    # Closing an empty gripper can target a different amount than grasping;
    # both commands must still move away from the open state in the same direction.
    assert (named["close"] - named["open"]) * (
        objective["close"] - objective["open"]
    ) > 0
