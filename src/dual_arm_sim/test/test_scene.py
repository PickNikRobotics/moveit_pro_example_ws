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

"""Checks of the MuJoCo scene, the Nav2 map made from it, and the spawn pose's two edges."""

import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE / "scripts"))

import generate_map  # noqa: E402

INSTALLED_SCENE = (
    Path(get_package_share_directory("dual_arm_sim"))
    / "description"
    / "mujoco"
    / "scene.xml"
)

# Half the side of the 5 cm sort blocks, scene.xml.
BLOCK_HALF_WIDTH = 0.025

# Fixed so the reach test is deterministic; the boundary it guards is sharp, not marginal.
IK_SEED = 1000
IK_RESTARTS = 30
IK_ITERATIONS = 200
IK_TOLERANCE = 2e-3
IK_DAMPING = 0.08


def test_keyframe_qpos_matches_model_dof_count():
    """A keyframe qpos whose length differs from nq crashes ros2_control_node at startup."""
    mujoco = pytest.importorskip("mujoco")
    # MoveIt Pro loads the same provider, which resolves the package:// mesh URIs.
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_path(str(INSTALLED_SCENE))
    for key in (
        ET.parse(PACKAGE / "description" / "mujoco" / "scene.xml").getroot().iter("key")
    ):
        assert len(key.get("qpos").split()) == model.nq, key.get("name")
        assert len(key.get("ctrl").split()) == model.nu, key.get("name")


def test_map_matches_the_generator(tmp_path):
    generate_map.write(tmp_path)
    for name in ("scene.pgm", "scene.yaml"):
        assert (tmp_path / name).read_bytes() == (
            PACKAGE / "maps" / name
        ).read_bytes(), f"maps/{name} is stale: run scripts/generate_map.py"


def _spawn_pose():
    """Planar x, y and yaw the keyframe starts the base at."""
    key = (
        ET.parse(PACKAGE / "description" / "mujoco" / "scene.xml")
        .getroot()
        .find("keyframe/key")
    )
    qpos = [float(v) for v in key.get("qpos").split()]
    return qpos[0], qpos[1], qpos[2]


def _footprint_at(points, x, y, yaw):
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    return [
        (x + px * cos_t - py * sin_t, y + px * sin_t + py * cos_t) for px, py in points
    ]


def _separated(poly, rect):
    """True when some axis separates the two, which proves they do not overlap."""
    x_min, x_max, y_min, y_max = rect
    corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    axes = [(1.0, 0.0), (0.0, 1.0)]
    for i, (px, py) in enumerate(poly):
        qx, qy = poly[(i + 1) % len(poly)]
        edge = (qx - px, qy - py)
        length = math.hypot(*edge)
        if length > 1e-12:
            axes.append((-edge[1] / length, edge[0] / length))
    for ax, ay in axes:
        pa = [px * ax + py * ay for px, py in poly]
        ra = [cx * ax + cy * ay for cx, cy in corners]
        if max(pa) <= min(ra) + 1e-12 or max(ra) <= min(pa) + 1e-12:
            return True
    return False


def _occupied_cells():
    """Non-border occupied cells of the generated map, as world rectangles."""
    rows = generate_map.occupancy(generate_map.table_boxes())
    size = len(rows)
    step, half = generate_map.RESOLUTION, generate_map.HALF_EXTENT
    cells = []
    for row, values in enumerate(rows):
        for col, value in enumerate(values):
            if value != generate_map.OCCUPIED:
                continue
            if col in (0, size - 1) or row in (0, size - 1):
                continue
            cells.append(
                (
                    -half + col * step,
                    -half + (col + 1) * step,
                    half - (row + 1) * step,
                    half - row * step,
                )
            )
    return cells


def test_footprint_clears_the_map_at_the_spawn_pose():
    """Nav2 refuses to plan from a pose whose footprint covers an occupied cell.

    One of the two edges of the spawn window; see AGENTS.md. Moving the base further back
    satisfies this test and eventually fails the drawing-reach one below.
    """
    x, y, yaw = _spawn_pose()
    params = yaml.safe_load((PACKAGE / "params" / "nav2_params.yaml").read_text())
    cells = _occupied_cells()
    assert cells, "the generated map has no table cells"
    for costmap in ("local_costmap", "global_costmap"):
        points = yaml.safe_load(
            params[costmap][costmap]["ros__parameters"]["footprint"]
        )
        poly = _footprint_at(points, x, y, yaw)
        for rect in cells:
            assert _separated(poly, rect), (
                "the clearance edge of the spawn window was crossed: the robot spawns "
                "on an occupied map cell, so Nav2 will refuse to plan and "
                '"Move Base to Ready" fails. Move the base further back, or shrink what '
                "the map projects.",
                costmap,
                (x, y, yaw),
                rect,
            )


def _centre_pose():
    """The drawing frame Writing Demo publishes as `local`, which every path pose is relative to."""
    text = (PACKAGE / "objectives" / "writing_demo.xml").read_text()
    position = [
        float(v) for v in re.search(r'position_xyz="([^"]+)"', text).group(1).split(";")
    ]
    quaternion = [
        float(v)
        for v in re.search(r'orientation_xyzw="([^"]+)"', text).group(1).split(";")
    ]
    return position, quaternion


def _tip_offset():
    text = (PACKAGE / "objectives" / "draw.xml").read_text()
    values = re.search(r'tip_offset="([^"]+)"', text).group(1).split(";")
    return [float(v) for v in values[:3]]


def _drawing_arms():
    """{path file: tip link} pairing each drawn path with the arm that draws it."""
    wipe_text = (PACKAGE / "objectives" / "draw_and_wipe.xml").read_text()
    demo_text = (PACKAGE / "objectives" / "writing_demo.xml").read_text()
    out = {}
    for call in re.findall(r'<SubTree\s+ID="Draw".*?/>', wipe_text, re.S):
        tip = re.search(r'tip_link="([^"]+)"', call).group(1)
        path = re.search(r'file_path="([^"]+)"', call).group(1)
        if path.startswith("{"):
            # The written text is whichever files writing_demo.xml passes down.
            for name in re.findall(r'file_path="([^"]+\.yaml)"', demo_text):
                out[name] = tip
        else:
            out[path] = tip
    return out


def _path_poses(name):
    poses = []
    for block in (PACKAGE / "objectives" / name).read_text().split("---"):
        position = re.search(
            r"position:\s*\n\s*x:\s*(\S+)\s*\n\s*y:\s*(\S+)\s*\n\s*z:\s*(\S+)", block
        )
        orientation = re.search(
            r"orientation:\s*\n\s*x:\s*(\S+)\s*\n\s*y:\s*(\S+)\s*\n\s*z:\s*(\S+)\s*\n\s*w:\s*(\S+)",
            block,
        )
        if position and orientation:
            poses.append(
                (
                    [float(v) for v in position.groups()],
                    [float(v) for v in orientation.groups()],
                )
            )
    return poses


def _matrix(quaternion_xyzw):
    import numpy

    x, y, z, w = numpy.asarray(quaternion_xyzw, float) / numpy.linalg.norm(
        quaternion_xyzw
    )
    return numpy.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _set_arm(mujoco, model, data, base, side, angles):
    data.qpos[:] = model.key_qpos[0]
    for joint, value in zip(("planar_x", "planar_y", "planar_theta"), base):
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        data.qpos[model.jnt_qposadr[index]] = value
    for number, value in enumerate(angles, start=1):
        index = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_fr3v2_joint{number}"
        )
        data.qpos[model.jnt_qposadr[index]] = value
    mujoco.mj_kinematics(model, data)
    # mj_jacBody reads data.cdof, which mj_kinematics does not fill.
    mujoco.mj_comPos(model, data)


def _reaches(mujoco, model, data, base, side, link, target_position, target_matrix):
    """Damped-least-squares IK on that arm's seven joints. True when a pose is found."""
    import numpy

    joints = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_fr3v2_joint{i}")
        for i in range(1, 8)
    ]
    limits = numpy.array([model.jnt_range[j] for j in joints])
    dofs = numpy.array([model.jnt_dofadr[j] for j in joints])
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link)
    jacp = numpy.zeros((3, model.nv))
    jacr = numpy.zeros((3, model.nv))
    generator = numpy.random.default_rng(IK_SEED)
    for _ in range(IK_RESTARTS):
        angles = generator.uniform(limits[:, 0], limits[:, 1])
        _set_arm(mujoco, model, data, base, side, angles)
        error = numpy.ones(6)
        for _ in range(IK_ITERATIONS):
            rotation = data.xmat[body].reshape(3, 3)
            delta = target_matrix @ rotation.T
            axis = numpy.array(
                [
                    delta[2, 1] - delta[1, 2],
                    delta[0, 2] - delta[2, 0],
                    delta[1, 0] - delta[0, 1],
                ]
            )
            sine = numpy.linalg.norm(axis)
            angle = math.atan2(sine, delta.trace() - 1.0)
            error = numpy.concatenate(
                [
                    target_position - data.xpos[body],
                    numpy.zeros(3) if sine < 1e-12 else axis / sine * angle,
                ]
            )
            if numpy.linalg.norm(error) < IK_TOLERANCE:
                return True
            mujoco.mj_jacBody(model, data, jacp, jacr, body)
            jacobian = numpy.vstack([jacp[:, dofs], jacr[:, dofs]])
            step = jacobian.T @ numpy.linalg.solve(
                jacobian @ jacobian.T + IK_DAMPING**2 * numpy.eye(6), error
            )
            angles = numpy.clip(
                data.qpos[[model.jnt_qposadr[j] for j in joints]]
                + numpy.clip(step, -0.3, 0.3),
                limits[:, 0],
                limits[:, 1],
            )
            _set_arm(mujoco, model, data, base, side, angles)
        if numpy.linalg.norm(error) < IK_TOLERANCE:
            return True
    return False


def test_writing_demo_paths_are_reachable_at_the_spawn_pose():
    """The drawing edge of the spawn window; see AGENTS.md.

    Proves only that inverse kinematics exists for the furthest commanded pose of each drawn
    path, which is the pose that binds. It does NOT run MoveIt: it checks no path continuity,
    no joint-limit continuity along the stroke and no collisions, so it can pass where
    PlanCartesianPath would still fail. A faithful check needs a licensed stack running
    Writing Demo itself.
    """
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("numpy")
    import numpy

    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_path(str(INSTALLED_SCENE))
    data = mujoco.MjData(model)

    base = _spawn_pose()
    centre_position, centre_quaternion = _centre_pose()
    centre_matrix = _matrix(centre_quaternion)
    offset = numpy.asarray(_tip_offset(), float)

    for name, link in sorted(_drawing_arms().items()):
        side = link.split("_")[0]
        targets = []
        for position, quaternion in _path_poses(name):
            matrix = centre_matrix @ _matrix(quaternion)
            targets.append(
                (
                    numpy.asarray(centre_position, float)
                    + centre_matrix @ numpy.asarray(position, float)
                    - matrix @ offset,
                    matrix,
                )
            )
        assert targets, f"no poses parsed from {name}"
        _set_arm(mujoco, model, data, base, side, [0.0] * 7)
        shoulder = data.xpos[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_fr3v2_link0")
        ].copy()
        furthest = max(targets, key=lambda t: numpy.linalg.norm(t[0] - shoulder))
        assert _reaches(mujoco, model, data, base, side, link, *furthest), (
            "the drawing edge of the spawn window was crossed: the base sits too far from "
            f"the front table for the {side} arm to reach the furthest pose of {name}, so "
            "Writing Demo can no longer plan its Cartesian path. Move the base towards the "
            "table, which eventually fails the clearance test above.",
            name,
            link,
            base,
            float(numpy.linalg.norm(furthest[0] - shoulder)),
        )


def test_place_waypoints_stay_over_their_tables():
    """The side tables are the sort destinations, so a moved table or base must keep them under the hand."""
    mujoco = pytest.importorskip("mujoco")
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_path(str(INSTALLED_SCENE))
    data = mujoco.MjData(model)
    waypoints = {
        entry["name"]: entry
        for entry in yaml.safe_load(
            (PACKAGE / "waypoints" / "waypoints.yaml").read_text()
        )
    }
    spawn = _spawn_pose()
    for name, link, body in (
        ("Place Left", "left_grasp_link", "table_left"),
        ("Place Right", "right_grasp_link", "table_right"),
    ):
        state = waypoints[name]["joint_state"]
        data.qpos[:] = model.key_qpos[0]
        for joint, value in zip(state["name"], state["position"]):
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            data.qpos[model.jnt_qposadr[index]] = value
        for joint, value in zip(("planar_x", "planar_y", "planar_theta"), spawn):
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            data.qpos[model.jnt_qposadr[index]] = value
        mujoco.mj_kinematics(model, data)
        tcp = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link)]
        table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        top = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, body)
        centre = model.body_pos[table] + model.geom_pos[top]
        half = model.geom_size[top]
        # The block hangs below the hand, so it needs room inside the edge, not just the edge.
        for axis, label in ((0, "x"), (1, "y")):
            inset = half[axis] - abs(tcp[axis] - centre[axis])
            assert inset > BLOCK_HALF_WIDTH, (name, body, label, inset)
