# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
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

"""Render the installed robot with MuJoCo's default mask, retaining its collision model."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
import numpy as np
import pytest


@pytest.fixture(scope="module")
def model():
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    scene = (
        Path(get_package_share_directory("dual_arm_sim"))
        / "description/mujoco/scene.xml"
    )
    return mujoco.MjModel.from_xml_path(str(scene))


def robot_geoms(model):
    """Geometry attached below the robot root, not the tables or blocks."""
    root = model.body("base").id
    result = []
    for geom, body in enumerate(model.geom_bodyid):
        while body and body != root:
            body = model.body_parentid[body]
        if body == root:
            result.append(geom)
    return np.asarray(result)


@pytest.mark.parametrize("keyframe", ["default", "nav_demo"])
def test_regression_normal_views_render_visuals_not_robot_collisions(model, keyframe):
    """RGB must retain visible robot parts while segmentation proves no collision geom owns a pixel."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(keyframe).id)
    mujoco.mj_forward(model, data)
    robot = robot_geoms(model)
    collides = (model.geom_contype | model.geom_conaffinity) != 0
    collision = robot[collides[robot]]
    visual = robot[~collides[robot]]
    assert len(collision) and len(visual)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = data.xpos[model.body("base_link").id] + [0, 0, 0.8]
    camera.distance = 3
    camera.elevation = -15
    option = mujoco.MjvOption()
    visible_pixels = 0
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        for direction in (0, 90, 180, 270):
            camera.azimuth = direction
            renderer.update_scene(data, camera=camera, scene_option=option)
            rgb = renderer.render().copy()
            assert np.ptp(rgb) > 0
            renderer.enable_segmentation_rendering()
            pixels = renderer.render().copy()
            renderer.disable_segmentation_rendering()
            geoms = pixels[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM
            ids = pixels[:, :, 0][geoms]
            assert not np.isin(ids, collision).any(), (keyframe, direction)
            visible_pixels += int(np.isin(ids, visual).sum())
        # Hidden collision geoms must still exist and be available in the debug collision view.
        debug = mujoco.MjvOption()
        debug.geomgroup[:] = 1
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=camera, scene_option=debug)
        pixels = renderer.render()
        ids = pixels[:, :, 0][pixels[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM]
        assert np.isin(ids, collision).any()
    assert visible_pixels > 0


def test_compiled_mesh_assets_have_consumers(model):
    """Every compiled mesh is owned by geometry, not a leftover unused asset declaration."""
    used = set(model.geom_dataid[model.geom_type == mujoco.mjtGeom.mjGEOM_MESH])
    unused = [
        model.mesh(index).name for index in range(model.nmesh) if index not in used
    ]
    assert not unused, unused
