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

"""Drive Base using Nav2 and the driver launch file must agree on each planner's Nav2 tree.

Both paths the SubTree names are absolute, because BehaviorTree.CPP refuses a relative tree
path: the default planner gets Nav2's own installed tree, and the Smac planner gets the copy
the launch file writes at startup with that planner selected. These checks run without a
simulator, so a mismatch between the Objective, the launch file and the planner list fails the
build rather than a drive.
"""

import ast
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "sim" / "robot_drivers_to_persist_sim.launch.py"
SCRIPT = re.compile(
    r"nav_tree := planner_id == '(?P<default>\w+)' \? "
    r"'(?P<stock>[^']+)' : '(?P<smac>[^']+)'$"
)


def script_parts():
    tree = ET.parse(PACKAGE / "objectives" / "drive_base_using_nav2.xml")
    codes = [a.get("code") for a in tree.iter("Action") if a.get("ID") == "Script"]
    assert len(codes) == 1
    match = SCRIPT.match(codes[0])
    assert match, codes[0]
    return match.groupdict()


def launch_constant(name):
    module = ast.parse(LAUNCH.read_text())
    for node in module.body:
        if (
            isinstance(node, ast.Assign)
            and getattr(node.targets[0], "id", None) == name
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} is not set in the launch file")


def planners():
    params = yaml.safe_load((PACKAGE / "params" / "nav2_params.yaml").read_text())
    return params["planner_server"]["ros__parameters"]["planner_plugins"]


def test_both_tree_paths_are_absolute():
    """A relative path reaches BehaviorTree.CPP and fails there, not at launch."""
    parts = script_parts()
    for key in ("stock", "smac"):
        assert Path(parts[key]).is_absolute(), f"{key} tree path is relative"


def test_objective_and_launch_file_name_the_same_trees():
    parts = script_parts()
    assert parts["stock"] == launch_constant("STOCK_NAV_TO_POSE_TREE")
    assert parts["smac"] == launch_constant("SMAC_NAV_TO_POSE_TREE")


def test_every_registered_planner_is_reachable():
    """Each planner in the params must be selectable through the SubTree's planner_id."""
    parts = script_parts()
    selectors = {
        parts["default"]: parts["stock"],
        launch_constant("SMAC_SELECTOR").split('"')[1]: parts["smac"],
    }
    for planner in planners():
        assert planner in selectors, f"{planner} has no tree the Objective can name"


def test_launch_file_writes_the_smac_tree_from_the_stock_one():
    """The generated tree must differ from the stock tree only in the planner selector."""
    stock_path = Path(launch_constant("STOCK_NAV_TO_POSE_TREE"))
    navfn, smac = launch_constant("NAVFN_SELECTOR"), launch_constant("SMAC_SELECTOR")
    source = LAUNCH.read_text()
    assert "def write_smac_nav_tree(" in source
    assert f"tree.replace({'NAVFN_SELECTOR'}, {'SMAC_SELECTOR'})" in source
    if stock_path.is_file():
        stock = stock_path.read_text()
        assert (
            stock.count(navfn) == 1
        ), "the stock tree no longer selects NavFn by default"
        assert smac.split('"')[1] in str(planners())
