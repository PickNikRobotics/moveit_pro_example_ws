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

"""Verify planar pose selection is wired into lunar_sim navigation."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

OBJECTIVES = Path(__file__).resolve().parents[1] / "objectives"


@pytest.mark.parametrize(
    ("filename", "consumer", "input_port"),
    [
        ("navigate_to_clicked_point.xml", "ComputePathToPoseAction", "goal"),
        (
            "navigate_to_clicked_point_with_replanning.xml",
            "NavigateToPoseAction",
            "pose_stamped",
        ),
    ],
)
def test_navigation_uses_confirmed_planar_goal(
    filename: str, consumer: str, input_port: str
) -> None:
    """The shared picker supplies a map-frame pose to the existing navigation path."""
    tree = ET.parse(OBJECTIVES / filename)
    picker = tree.find(".//Action[@ID='GetPlanarPoseFromUser']")
    assert (
        picker is not None
    ), "Navigation must request an editable position and heading."
    assert picker.get("reference_frame") == "map"
    assert picker.get("view_name") == "Visualization"
    assert picker.get("user_pose") == "{goal}"
    navigation = tree.find(f".//Action[@ID='{consumer}']")
    assert navigation is not None
    assert navigation.get(input_port) == picker.get("user_pose")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    children = list(sequence)
    assert children.index(picker) < children.index(navigation)
    assert tree.find(".//Action[@ID='GetPoseFromUser']") is None


def test_navigation_path_approval_precedes_motion() -> None:
    """The confirmed goal is planned and approved before its path is followed."""
    tree = ET.parse(OBJECTIVES / "navigate_to_clicked_point.xml")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    children = list(sequence)
    plan = sequence.find("./Action[@ID='ComputePathToPoseAction']")
    approval = sequence.find("./Action[@ID='WaitForUserPathApproval']")
    follow = sequence.find("./Action[@ID='FollowPathAction']")
    assert plan is not None
    assert approval is not None
    assert follow is not None
    assert children.index(plan) < children.index(approval) < children.index(follow)
    assert approval.get("path") == follow.get("path") == plan.get("path")
