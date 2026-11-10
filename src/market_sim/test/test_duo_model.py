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

"""The compiled Duo model, URDF, SRDF, controller and stowed navigation geometry contract."""

import itertools
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
PLANAR_JOINTS = ("planar_x", "planar_y", "planar_theta")


@pytest.fixture(scope="module")
def model():
    """Compile the actual store including the Duo and its full collision geometry."""
    from ament_index_python.packages import get_package_prefix

    # The Duo's franka_description meshes are package:// URIs; CTest runs this file in its own
    # process, so load MoveIt Pro's resolver here rather than rely on another test having done it.
    plugin = Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin"
    mujoco.mj_loadPluginLibrary(
        str(plugin / "libmujoco_package_uri_resource_provider.so")
    )
    return mujoco.MjModel.from_xml_path(str(PACKAGE / "mjcf/scene.xml"))


@pytest.fixture(scope="module")
def urdf():
    """Expand the installed Franka description rather than infer the xacro's output."""
    result = subprocess.run(
        ["xacro", str(PACKAGE / "description/mobile_fr3_duo.urdf.xacro")],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def stowed_data(model):
    """Set both arms and spine to the exact waypoint used before navigation."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    waypoint = next(
        item
        for item in yaml.safe_load((PACKAGE / "waypoints/waypoints.yaml").read_text())
        if item["name"] == "Stow"
    )
    for name, value in zip(
        waypoint["joint_state"]["name"], waypoint["joint_state"]["position"]
    ):
        data.qpos[model.joint(name).qposadr[0]] = value
        data.ctrl[model.actuator(name).id] = value
    mujoco.mj_forward(model, data)
    return data


def robot_geoms(model):
    """Return geoms descended from the robot base, excluding the store and products."""
    base = model.body("base_link").id
    bodies = {base}
    for body in range(base + 1, model.nbody):
        if model.body_parentid[body] in bodies:
            bodies.add(body)
    return [geom for geom in range(model.ngeom) if model.geom_bodyid[geom] in bodies]


def geom_points(model, data, geom):
    """World vertices for meshes or a conservative bounding box for analytic geometry."""
    kind = model.geom_type[geom]
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        local = model.mesh_vert[start : start + count]
    else:
        size = model.geom_size[geom].copy()
        if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            size[:] = size[0]
        elif kind in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
            radius, half_length = size[:2]
            size = np.array(
                [
                    radius,
                    radius,
                    half_length
                    + (radius if kind == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0),
                ]
            )
        else:
            assert kind in (
                mujoco.mjtGeom.mjGEOM_BOX,
                mujoco.mjtGeom.mjGEOM_ELLIPSOID,
            ), (geom, kind)
        local = np.array(list(itertools.product((-1, 1), repeat=3))) * size
    return local @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]


def inside_polygon(points, polygon):
    """Ray-cast arbitrary footprint polygons, including the Duo's concave rear shoulders."""
    inside = np.zeros(len(points), dtype=bool)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        crossed = (a[1] > points[:, 1]) != (b[1] > points[:, 1])
        if a[1] != b[1]:
            boundary_x = (b[0] - a[0]) * (points[:, 1] - a[1]) / (b[1] - a[1]) + a[0]
            inside ^= crossed & (points[:, 0] < boundary_x)
    return inside


def test_stowed_robot_collision_geometry_fits_navigation_footprint(model) -> None:
    """Both arms, grippers, spine and chassis collision geometry stay inside Nav2's padded polygon."""
    data = stowed_data(model)
    params = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())[
        "local_costmap"
    ]["local_costmap"]["ros__parameters"]
    polygon = np.array(yaml.safe_load(params["footprint"]))
    polygon += np.sign(polygon) * params.get("footprint_padding", 0.01)
    outside = []
    for geom in robot_geoms(model):
        if not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        points = geom_points(model, data, geom)
        base_points = (points - data.xpos[model.body("base_link").id]) @ data.xmat[
            model.body("base_link").id
        ].reshape(3, 3)
        if not inside_polygon(base_points[:, :2], polygon).all():
            outside.append(
                (
                    model.geom(geom).name,
                    base_points[:, :2].min(axis=0).tolist(),
                    base_points[:, :2].max(axis=0).tolist(),
                )
            )
    assert (
        not outside
    ), f"Stowed collision geometry outside the Nav2 footprint: {outside}"


def test_planar_and_arm_joint_interfaces_match_description(model, urdf) -> None:
    """Actuator names, joint types, directions and limits match the planning model."""
    joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    for actuator in range(model.nu):
        name = model.actuator(actuator).name
        assert name in joints, name
        joint = model.joint(name)
        declared = joints[name]
        assert model.jnt_type[joint.id] == (
            mujoco.mjtJoint.mjJNT_SLIDE
            if declared.get("type") == "prismatic"
            else mujoco.mjtJoint.mjJNT_HINGE
        )
        np.testing.assert_allclose(
            joint.axis,
            [float(v) for v in declared.find("axis").get("xyz").split()],
            atol=1e-5,
            err_msg=name,
        )
        limit = declared.find("limit")
        if declared.get("type") == "continuous":
            assert not model.jnt_limited[joint.id], name
        else:
            np.testing.assert_allclose(
                joint.range,
                [float(limit.get("lower")), float(limit.get("upper"))],
                atol=1e-4,
                err_msg=name,
            )
        if not name.endswith("finger_joint1"):
            assert model.actuator_trnid[actuator, 0] == joint.id, name
    control = urdf.find("ros2_control")
    assert control.find("hardware/plugin").text == "picknik_mujoco_ros/MujocoSystem"
    params = {p.get("name"): p.text for p in control.findall("hardware/param")}
    assert params["base_link_name"] == "base_link"
    assert params["publish_mj_world_tf"] == "false"
    assert params["odom_publish_tf"] == "false"
    assert params["odom_topic"] == "/ground_truth/odom"


def test_semantic_groups_and_stow_waypoint_resolve(urdf) -> None:
    """The restored arm/spine/gripper and independent mobile-base groups resolve actual joints."""
    links = {link.get("name") for link in urdf.findall("link")}
    joints = {joint.get("name") for joint in urdf.findall("joint")}
    srdf = ET.parse(PACKAGE / "config/moveit/mobile_fr3_duo.srdf")
    groups = {group.get("name"): group for group in srdf.findall("group")}
    assert {
        "manipulator",
        "mobile_base",
        "spine",
        "left_gripper",
        "right_gripper",
    } <= set(groups)
    assert [j.get("name") for j in groups["mobile_base"].findall("joint")] == list(
        PLANAR_JOINTS
    )
    for group in groups.values():
        for joint in group.findall("joint"):
            assert joint.get("name") in joints
        for chain in group.findall("chain"):
            assert chain.get("base_link") in links
            assert chain.get("tip_link") in links
        for subgroup in group.findall("group"):
            assert subgroup.get("name") in groups
    for end in srdf.findall("end_effector"):
        assert end.get("parent_link") in links
        assert end.get("group") in groups
        assert end.get("parent_group") in groups
    for waypoint in yaml.safe_load((PACKAGE / "waypoints/waypoints.yaml").read_text()):
        assert set(waypoint["joint_state"]["name"]) <= joints, waypoint["name"]
        assert set(waypoint["joint_group_names"]) <= set(groups), waypoint["name"]


def test_raw_planar_state_is_private_and_only_fused_joints_reach_rsp() -> None:
    """Encoder feedback cannot race the fused planning/TF state publisher."""
    control = yaml.safe_load(
        (PACKAGE / "config/control/mobile_fr3_duo.ros2_control.yaml").read_text()
    )
    base = control["base_state_broadcaster"]["ros__parameters"]
    assert base["use_local_topics"]
    assert base["joints"] == list(PLANAR_JOINTS)
    assert base["interfaces"] == ["position", "velocity"]
    general = control["joint_state_broadcaster"]["ros__parameters"]
    assert not set(PLANAR_JOINTS).intersection(general["joints"])
    assert control["base_jgvc"]["ros__parameters"]["joints"] == list(PLANAR_JOINTS)


def test_lidar_mounts_match_real_description_and_horizontal_ring(model, urdf) -> None:
    """Duo lidar extrinsics agree across the compiled model and upstream Franka description."""
    data = stowed_data(model)
    for side in ("front", "rear"):
        link = f"lidar_{side}_mounting_point"
        joint = next(
            j for j in urdf.findall("joint") if j.find("child").get("link") == link
        )
        origin = joint.find("origin")
        body = model.body(link).id
        np.testing.assert_allclose(
            model.body_pos[body],
            [float(v) for v in origin.get("xyz").split()],
            atol=1e-6,
        )
        camera = model.camera(f"lidar_{side}").id
        site = model.site(f"lidar_{side}_optical_frame").id
        np.testing.assert_allclose(
            data.cam_xpos[camera], data.site_xpos[site], atol=1e-6
        )
        np.testing.assert_allclose(
            data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1]),
            data.site_xmat[site].reshape(3, 3),
            atol=1e-6,
        )
        # Optical X and Z span a horizontal scan; optical Y is vertical.
        assert abs(data.site_xmat[site].reshape(3, 3)[2, 1]) == pytest.approx(
            1.0, abs=1e-6
        )
        assert model.cam_user[camera, 1] == 275.0
        assert data.cam_xpos[camera, 2] == pytest.approx(0.19065)
