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

"""Checks of the MuJoCo scene, the Nav2 map made from it, and the footprint at the start pose."""

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


def test_table_legs_clear_the_footprint_at_the_start_pose():
    """Nav2 must not see the robot in collision at the map origin, where the keyframe puts it."""
    params = yaml.safe_load((PACKAGE / "params" / "nav2_params.yaml").read_text())
    for costmap in ("local_costmap", "global_costmap"):
        footprint = yaml.safe_load(
            params[costmap][costmap]["ros__parameters"]["footprint"]
        )
        xs = [p[0] for p in footprint]
        ys = [p[1] for p in footprint]
        for x_min, x_max, y_min, y_max in generate_map.leg_boxes():
            overlaps_x = x_min < max(xs) + generate_map.RESOLUTION and x_max > min(xs)
            overlaps_y = y_min < max(ys) + generate_map.RESOLUTION and y_max > min(ys)
            assert not (overlaps_x and overlaps_y), (
                costmap,
                (x_min, x_max, y_min, y_max),
            )
