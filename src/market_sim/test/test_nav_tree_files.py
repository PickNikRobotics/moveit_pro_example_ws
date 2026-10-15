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

"""Drive Base using Nav2 and the launch file must agree on where each planner's Nav2 tree is.

The SubTree's Script names the tree file for each planner: Nav2's installed stock tree for the
default planner, and a file relative to bt_navigator's working directory for any other. The launch
file sets that working directory; CMakeLists.txt installs the files there. Needs the installed
package, so it runs after colcon build.
"""

import ast
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "sim" / "robot_drivers_to_persist_sim.launch.py"
SCRIPT = re.compile(
    r"nav_tree := planner_id == '(?P<default>\w+)' \? '(?P<stock>[^']+)' : "
    r"'(?P<prefix>[^']*)' \.\. planner_id \.\. '(?P<suffix>[^']*)'$"
)


def script_parts():
    tree = ET.parse(PACKAGE / "objectives" / "drive_base_using_nav2.xml")
    codes = [a.get("code") for a in tree.iter("Action") if a.get("ID") == "Script"]
    assert len(codes) == 1
    match = SCRIPT.match(codes[0])
    assert match, codes[0]
    return match.groupdict()


def nav_tree_dir():
    module = ast.parse(LAUNCH.read_text())
    for node in module.body:
        if isinstance(node, ast.Assign) and node.targets[0].id == "NAV_TREE_DIR":
            return node.value.value
    raise AssertionError("NAV_TREE_DIR is not set in the launch file")


def planners():
    params = yaml.safe_load((PACKAGE / "params" / "nav2_params.yaml").read_text())
    return params["planner_server"]["ros__parameters"]["planner_plugins"]


def test_launch_runs_bt_navigator_in_the_tree_directory():
    source = LAUNCH.read_text()
    call = re.search(
        r'nav2_node\(\s*"nav2_bt_navigator",\s*"bt_navigator",'
        r"\s*cwd=os\.path\.join\(share, NAV_TREE_DIR\)\s*\)",
        source,
    )
    assert call, "bt_navigator is not started in the tree folder through nav2_node"


def test_every_planner_has_a_tree_file():
    parts = script_parts()
    folder = Path(get_package_share_directory("market_sim")) / nav_tree_dir()
    for planner in planners():
        if planner == parts["default"]:
            path = Path(parts["stock"])
        else:
            path = folder / f"{parts['prefix']}{planner}{parts['suffix']}"
        assert path.is_file(), f"{planner}: {path}"


def test_copies_differ_from_the_stock_tree_only_in_the_planner():
    parts = script_parts()
    stock = Path(parts["stock"]).read_text()
    folder = Path(get_package_share_directory("market_sim")) / nav_tree_dir()
    selector = f'default_planner="{parts["default"]}"'
    assert stock.count(selector) == 1
    for planner in planners():
        if planner == parts["default"]:
            continue
        copy = (folder / f"{parts['prefix']}{planner}{parts['suffix']}").read_text()
        assert copy == stock.replace(selector, f'default_planner="{planner}"')
