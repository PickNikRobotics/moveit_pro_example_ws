#!/usr/bin/env python3

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

"""The hangar must stay rigid to the same root the base joints hang off.

`world` is two things at once: MoveIt's planning root, and the link the hangar's
collision meshes are welded to. Because the base's three joints hang off that same
root, the robot moves *through* a fixed environment instead of the environment
moving with the robot — which is the only reason the arm can plan against the
aircraft at `link_padding` 0.0 while navigation runs on a localization estimate.

Re-parenting the sim's tree to the hardware shape (`map -> odom -> base_link`)
would delete that property in one move: the planning root would follow the
estimate and drag all 66 meshes with it, and nothing would fail loudly, because
the arm planner's world would have drifted identically to the arm. The
`odom -> world` bridge in `launch/sim/robot_drivers_to_persist_sim.launch.py`
exists to keep the estimate above `world` instead. These checks pin the shape that
argument rests on, so a re-parent has to be deliberate.

These parse the declarative artifacts into their own semantic models — the URDF
link/joint graph, the launch file's `Node(...)` calls — and assert meaning in that
model, not the presence of text. They follow `test_base_geometry.py`, which reads
the same descriptions directly rather than expanding the xacro, because expansion
would need the launch's full argument set and tie these assertions to launch
configuration they do not care about. No simulator, no ROS.
"""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parent.parent
ROBOT_XACRO = PACKAGE / "description" / "ur5e_ridgeback.xacro"
HANGAR_XACRO = PACKAGE / "description" / "hangar_urdf.xacro"
CONTROL_XACRO = PACKAGE / "description" / "picknik_ur_mujoco_ros2_control.xacro"
CONTROL_YAML = PACKAGE / "config" / "control" / "picknik_ur.ros2_control.yaml"
DRIVERS_LAUNCH = PACKAGE / "launch" / "sim" / "robot_drivers_to_persist_sim.launch.py"


# The two description files spell the xacro namespace differently
# (wiki.ros.org vs ros.org/wiki), so tags are matched on local name only.
def _local_name(tag: str) -> str:
    return tag.rpartition("}")[2]


def _find_all(element: ET.Element, name: str) -> list[ET.Element]:
    return [node for node in element.iter() if _local_name(node.tag) == name]


# The frame the planning root is expected to be, named here so a rename has to
# travel through this file rather than quietly passing every check below.
PLANNING_ROOT = "world"

# The joint that hangs the whole robot off the planning root.
BASE_ROOT_JOINT = "linear_x_joint"

# Every collision mesh the arm plans against. The number is quoted in the comments
# this file backs, so it is checked rather than assumed.
HANGAR_COLLISION_MESH_COUNT = 66

# Both mecanum controller instances publish odometry messages; neither may publish
# the odom -> base TF edge, or it would bypass the bridge.
MECANUM_CONTROLLERS = (
    "platform_velocity_controller",
    "platform_velocity_controller_nav2",
)


def _robot_root() -> ET.Element:
    return ET.parse(ROBOT_XACRO).getroot()


def _joint_parent(root: ET.Element, joint_name: str) -> str:
    for joint in root.iter("joint"):
        if joint.get("name") == joint_name:
            parent = joint.find("parent")
            assert parent is not None, f"{joint_name} declares no <parent>"
            return parent.get("link", "")
    raise AssertionError(f"{ROBOT_XACRO.name} has no joint named {joint_name!r}")


def _hangar_attachment_parent(root: ET.Element) -> str:
    calls = [
        element
        for element in _find_all(root, "hangar_urdf")
        if element.get("parent") is not None
    ]
    assert len(calls) == 1, (
        f"{ROBOT_XACRO.name} instantiates the hangar {len(calls)} times; this file "
        f"reasons about exactly one environment attachment."
    )
    return calls[0].get("parent", "")


def _hangar_macro() -> ET.Element:
    macros = _find_all(ET.parse(HANGAR_XACRO).getroot(), "macro")
    assert len(macros) == 1, (
        f"{HANGAR_XACRO.name} defines {len(macros)} macros; this file reasons about "
        f"the single hangar_urdf macro."
    )
    return macros[0]


def _hangar_joints() -> list[ET.Element]:
    return _find_all(_hangar_macro(), "joint")


def test_base_joints_hang_off_the_planning_root() -> None:
    """The base's pose is joint state below `world`, not a transform above it."""
    parent = _joint_parent(_robot_root(), BASE_ROOT_JOINT)
    assert parent == PLANNING_ROOT, (
        f"{BASE_ROOT_JOINT} now hangs off {parent!r} rather than {PLANNING_ROOT!r}. "
        f"The base pose is three joints under the planning root; re-parenting it "
        f"moves the base pose into TF above the root, where the hangar cannot follow."
    )


def test_the_hangar_hangs_off_the_same_root() -> None:
    """One root for both, which is the whole constraint."""
    root = _robot_root()
    base_parent = _joint_parent(root, BASE_ROOT_JOINT)
    hangar_parent = _hangar_attachment_parent(root)
    assert hangar_parent == base_parent, (
        f"the hangar attaches to {hangar_parent!r} while {BASE_ROOT_JOINT} hangs off "
        f"{base_parent!r}. They share a root, and the odom -> world bridge exists "
        f"because of it: splitting them changes what that bridge is for."
    )


def test_the_hangar_is_rigid_to_that_root() -> None:
    """No joint in the environment may articulate, or 'rigid' stops being true."""
    movable = {
        joint.get("name", "?"): joint.get("type", "?")
        for joint in _hangar_joints()
        if joint.get("type") != "fixed"
    }
    assert not movable, (
        f"the hangar now carries non-fixed joints {movable}. Every mesh is welded to "
        f"the planning root; an articulated one would move independently of it."
    )


def test_every_hangar_mesh_reaches_the_root_through_fixed_joints() -> None:
    """A mesh that does not reach `${parent}` is not held still by holding it still."""
    parent_of: dict[str, str] = {}
    for joint in _hangar_joints():
        parent = joint.find("parent")
        child = joint.find("child")
        assert (
            parent is not None and child is not None
        ), f"hangar joint {joint.get('name', '?')!r} is missing a parent or child"
        parent_of[child.get("link", "")] = parent.get("link", "")

    collision_links = [
        link.get("name", "")
        for link in _find_all(_hangar_macro(), "link")
        if link.find("collision") is not None
    ]
    assert len(collision_links) == HANGAR_COLLISION_MESH_COUNT, (
        f"the hangar now contributes {len(collision_links)} collision meshes, not "
        f"{HANGAR_COLLISION_MESH_COUNT}. That count is quoted in the comments "
        f"explaining why the odom -> world bridge exists; update them together."
    )

    for link in collision_links:
        seen, current = [link], link
        while current in parent_of:
            current = parent_of[current]
            assert current not in seen, f"cycle in the hangar chain at {current!r}"
            seen.append(current)
        assert current == "${parent}", (
            f"collision link {link!r} walks up to {current!r}, not to the macro's "
            f"parent. Only links that reach the planning root are held still by it."
        )


def _launch_module() -> ast.Module:
    return ast.parse(DRIVERS_LAUNCH.read_text(), filename=str(DRIVERS_LAUNCH))


def _static_transform_publishers(module: ast.Module) -> dict[str, list[str]]:
    """Every tf2_ros static_transform_publisher in the launch, by variable name."""
    found: dict[str, list[str]] = {}
    for node in ast.walk(module):
        if not isinstance(node, ast.Assign) or not isinstance(node.value, ast.Call):
            continue
        call = node.value
        if not (isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords if kw.arg}
        executable = kwargs.get("executable")
        if not (
            isinstance(executable, ast.Constant)
            and executable.value == "static_transform_publisher"
        ):
            continue
        arguments = kwargs.get("arguments")
        assert isinstance(arguments, ast.List), (
            f"{DRIVERS_LAUNCH.name}: a static_transform_publisher's arguments are not "
            f"a literal list, so this check can no longer read its frames."
        )
        values = [
            element.value
            for element in arguments.elts
            if isinstance(element, ast.Constant)
        ]
        for target in node.targets:
            if isinstance(target, ast.Name):
                found[target.id] = values
    return found


def _added_to_launch_description(module: ast.Module) -> set[str]:
    added: set[str] = set()
    for node in ast.walk(module):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "add_action" or len(node.args) != 1:
            continue
        if isinstance(node.args[0], ast.Name):
            added.add(node.args[0].id)
    return added


def test_the_odom_to_world_bridge_is_launched() -> None:
    """The edge that keeps the estimate above the planning root."""
    module = _launch_module()
    publishers = _static_transform_publishers(module)
    bridges = {
        name: args
        for name, args in publishers.items()
        if args[-2:] == ["odom", PLANNING_ROOT]
    }
    assert len(bridges) == 1, (
        f"expected exactly one odom -> {PLANNING_ROOT} static transform in "
        f"{DRIVERS_LAUNCH.name}, found {sorted(bridges)} among {sorted(publishers)}. "
        f"Without it the planning root has no parent and nav2 goals in "
        f"{PLANNING_ROOT!r} stop resolving to 'map'."
    )
    name = next(iter(bridges))
    assert name in _added_to_launch_description(module), (
        f"{name} is constructed but never added to the LaunchDescription, so the "
        f"bridge would not actually run."
    )


def test_nothing_else_in_this_launch_parents_the_planning_root() -> None:
    """Two parents for `world` is a broken tree, not a redundant one.

    Scoped to this launch file's static transform publishers, which is where every
    frame above `world` is declared; it cannot see a parent broadcast at runtime.
    """
    parents = {
        name: args[-2]
        for name, args in _static_transform_publishers(_launch_module()).items()
        if args[-1:] == [PLANNING_ROOT]
    }
    assert len(parents) == 1, (
        f"{len(parents)} static transforms in {DRIVERS_LAUNCH.name} publish "
        f"{PLANNING_ROOT!r} as a child ({parents}). A frame may have exactly one "
        f"parent, and the odom -> world bridge is meant to be it."
    )


def test_no_simulator_or_controller_publishes_odom_to_base() -> None:
    """An odom -> base edge would route around the bridge and re-split the tree.

    `world -> ... -> ridgeback_base_link` is robot_state_publisher's alone. If
    anything also broadcast `odom -> ridgeback_base_link`, navigation would read the
    base from that edge instead of through `world`, and `world` would stop being the
    frame that holds the environment still relative to the robot.
    """
    hardware = _find_all(ET.parse(CONTROL_XACRO).getroot(), "hardware")
    assert len(hardware) == 1, (
        f"{CONTROL_XACRO.name} declares {len(hardware)} <hardware> blocks; this check "
        f"reads the MuJoCo plugin's parameters from the single one."
    )
    params = {
        param.get("name"): (param.text or "").strip()
        for param in _find_all(hardware[0], "param")
    }
    assert params.get("odom_publish_tf", "").lower() == "false", (
        f"{CONTROL_XACRO.name} sets odom_publish_tf = "
        f"{params.get('odom_publish_tf')!r}; it must stay false, or MuJoCo broadcasts "
        f"odom -> ridgeback_base_link alongside robot_state_publisher."
    )

    controllers = yaml.safe_load(CONTROL_YAML.read_text())
    for controller in MECANUM_CONTROLLERS:
        parameters = controllers[controller]["ros__parameters"]
        assert parameters.get("enable_odom_tf") is False, (
            f"{controller} sets enable_odom_tf = "
            f"{parameters.get('enable_odom_tf')!r}; it must stay false so the "
            f"odometry it publishes stays a message and never becomes a TF edge."
        )
