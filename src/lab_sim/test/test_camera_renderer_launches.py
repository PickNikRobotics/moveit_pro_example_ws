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

"""Keep every MuJoCo config's Nyx launch cameras aligned with its robot description."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import pytest

SOURCE_ROOT = Path(__file__).resolve().parents[2]
SHARED_MUJOCO_ASSETS = SOURCE_ROOT / "picknik_accessories/mujoco_assets"
LAB_DESCRIPTION = SOURCE_ROOT / "lab_sim/description/picknik_ur.xacro"


@dataclass(frozen=True)
class RendererConfig:
    """The source files that declare one config's renderer-selected RGB cameras."""

    package: str
    directory: Path
    description: Path
    scene_relative: str
    scene: Path | None
    nyx_cameras_override: str | None = None
    direct_sensor_names: bool = False


CONFIGS = (
    RendererConfig(
        "april_tag_sim",
        SOURCE_ROOT / "april_tag_sim",
        LAB_DESCRIPTION,
        "description/scene.xml",
        SOURCE_ROOT / "april_tag_sim/description/scene.xml",
    ),
    RendererConfig(
        "dual_arm_sim",
        SOURCE_ROOT / "dual_arm_sim",
        SOURCE_ROOT / "dual_arm_sim/description/franka.urdf.xacro",
        "description/mujoco/scene.xml",
        SOURCE_ROOT / "dual_arm_sim/description/mujoco/scene.xml",
    ),
    RendererConfig(
        "factory_sim",
        SOURCE_ROOT / "factory_sim",
        SOURCE_ROOT / "factory_sim/description/fanuc_lrmate200id.ros2_control.xacro",
        "description/scene.xml",
        SOURCE_ROOT / "factory_sim/description/scene.xml",
    ),
    RendererConfig(
        "grinding_sim",
        SOURCE_ROOT / "grinding_sim",
        SOURCE_ROOT / "grinding_sim/description/ur20.urdf.xacro",
        "description/scene.xml",
        SOURCE_ROOT / "grinding_sim/description/scene.xml",
    ),
    RendererConfig(
        "hand_eye_calibration_sim",
        SOURCE_ROOT / "hand_eye_calibration_sim",
        LAB_DESCRIPTION,
        "description/scene.xml",
        SOURCE_ROOT / "hand_eye_calibration_sim/description/scene.xml",
        "scene_camera_narrow scene_camera_wide wrist_camera",
    ),
    RendererConfig(
        "hangar_sim",
        SOURCE_ROOT / "hangar_sim",
        SOURCE_ROOT / "hangar_sim/description/picknik_ur_mujoco_ros2_control.xacro",
        "description/hangar_scene.xml",
        SOURCE_ROOT / "hangar_sim/description/hangar_scene.xml",
    ),
    RendererConfig(
        "kitchen_sim",
        SOURCE_ROOT / "kitchen_sim",
        SOURCE_ROOT / "kitchen_sim/description/franka.urdf.xacro",
        "description/mujoco/scene.xml",
        SOURCE_ROOT / "kitchen_sim/description/mujoco/scene.xml",
    ),
    RendererConfig(
        "lab_sim",
        SOURCE_ROOT / "lab_sim",
        LAB_DESCRIPTION,
        "description/scene.xml",
        SOURCE_ROOT / "lab_sim/description/scene.xml",
    ),
    RendererConfig(
        "lunar_sim",
        SOURCE_ROOT / "lunar_sim",
        SOURCE_ROOT / "lunar_sim/description/husky_a300_mujoco.xacro",
        "description/husky_scene.xml",
        SOURCE_ROOT / "lunar_sim/description/husky_scene.xml",
    ),
    RendererConfig(
        "kinova_sim",
        SOURCE_ROOT / "moveit_pro_kinova_configs/kinova_sim",
        SOURCE_ROOT
        / "moveit_pro_kinova_configs/kinova_sim/description/picknik_kinova_gen3.xacro",
        "description/mujoco/rafti_fingers_scene.xml",
        SOURCE_ROOT
        / "moveit_pro_kinova_configs/kinova_sim/description/mujoco/rafti_fingers_scene.xml",
    ),
    RendererConfig(
        "space_satellite_sim",
        SOURCE_ROOT / "moveit_pro_kinova_configs/space_satellite_sim",
        SOURCE_ROOT
        / "moveit_pro_kinova_configs/space_satellite_sim/description/space_satellite_sim.xacro",
        "description/mujoco/scene.xml",
        SOURCE_ROOT
        / "moveit_pro_kinova_configs/space_satellite_sim/description/mujoco/scene.xml",
    ),
    RendererConfig(
        "vla_sim",
        SOURCE_ROOT / "vla_sim",
        SOURCE_ROOT / "vla_sim/description/picknik_kinova_gen3.xacro",
        "description/mujoco/cube_stack_scene.xml",
        SOURCE_ROOT / "vla_sim/description/mujoco/cube_stack_scene.xml",
    ),
    RendererConfig(
        "rebot_bench_sim",
        SOURCE_ROOT / "rebot_bench_sim",
        SOURCE_ROOT / "rebot_bench_sim/description/rebot_bench_sim.urdf.xacro",
        "description/scene.xml",
        None,
        direct_sensor_names=True,
    ),
)


def _local_name(element: ET.Element) -> str:
    """The XML local name, independent of whether xacro's namespace is expanded."""
    return element.tag.rsplit("}", maxsplit=1)[-1]


def _xacro_arg_default(root: ET.Element, name: str) -> str:
    """The default of one xacro argument, failing if a selector was renamed or removed."""
    args = [
        element
        for element in root.iter()
        if _local_name(element) == "arg" and element.get("name") == name
    ]
    assert len(args) == 1
    default = args[0].get("default")
    assert default is not None
    return default


def _renderer_sensors(config: RendererConfig) -> set[str]:
    """The cameras whose MuJoCo sensor overrides the description selects."""
    root = ET.parse(config.description).getroot()
    if config.direct_sensor_names:
        return {
            sensor.get("name")
            for sensor in root.iter()
            if _local_name(sensor) == "sensor" and sensor.get("name")
        }

    macros = [
        element
        for element in root.iter()
        if _local_name(element) == "camera_renderer_sensors"
    ]
    assert len(macros) == 1
    cameras = macros[0].get("cameras")
    assert cameras is not None
    if cameras == "$(arg nyx_cameras)":
        cameras = config.nyx_cameras_override or _xacro_arg_default(root, "nyx_cameras")
    return set(cameras.split())


def _scene_cameras(path: Path, seen: set[Path] | None = None) -> set[str]:
    """The camera names in a scene and its local or shared-asset includes."""
    seen = set() if seen is None else seen
    path = path.resolve()
    if path in seen:
        return set()
    seen.add(path)
    root = ET.parse(path).getroot()
    cameras = {
        camera.get("name") for camera in root.iter("camera") if camera.get("name")
    }
    for include in root.iter("include"):
        include_path = include.get("file")
        if not include_path or include_path.startswith("package:"):
            continue
        candidate = path.parent / include_path
        if not candidate.is_file():
            candidate = SHARED_MUJOCO_ASSETS / include_path
        if candidate.is_file():
            cameras |= _scene_cameras(candidate, seen)
    return cameras


def _nyx_launches(config: RendererConfig) -> tuple[ET.Element, list[ET.Element]]:
    """The root and Nyx camera includes in one config's runtime launch file."""
    root = ET.parse(config.directory / "launch/runtime.launch.xml").getroot()
    launches = [
        include
        for include in root.iter("include")
        if include.get("file", "").endswith("nyx_camera.launch.py")
    ]
    return root, launches


@pytest.mark.parametrize("config", CONFIGS, ids=lambda config: config.package)
def test_runtime_launch_cameras_match_the_renderer_description(
    config: RendererConfig,
) -> None:
    """A renamed camera cannot leave description, launch or scene wiring out of sync."""
    config_yaml = (config.directory / "config/config.yaml").read_text()
    runtime_match = re.search(
        r"runtime_launch_file:\s+package:\s*[\"']?([^\s\"']+)[\"']?\s+path:\s*[\"']?([^\s\"']+)[\"']?",
        config_yaml,
    )
    assert runtime_match is not None
    assert runtime_match.group(1) == config.package
    assert runtime_match.group(2) == "launch/runtime.launch.xml"

    root, launches = _nyx_launches(config)
    launch_args = [
        {arg.get("name"): arg.get("value") for arg in launch.findall("arg")}
        for launch in launches
    ]
    cameras = {launch["camera"] for launch in launch_args}
    assert cameras == _renderer_sensors(config)
    expected_mjcf = f"$(find-pkg-share {config.package})/{config.scene_relative}"
    expected_scene_dir = (
        "$(env USER_WS $(env HOME)/user_ws)/.nyx_scenes/" + config.package
    )
    if not config.direct_sensor_names:
        assert [(let.get("name"), let.get("value")) for let in root.findall("let")] == [
            ("camera_renderer", "$(env MOVEIT_PRO_CAMERA_RENDERER '')"),
            (
                "nyx_enabled",
                '$(eval \'"$(var camera_renderer)" in ("nyx", "both")\')',
            ),
            (
                "nyx_namespace",
                '$(eval \'"nyx" if "$(var camera_renderer)" == "both" else ""\')',
            ),
        ]
        assert launches
        assert len(launches) == len(_renderer_sensors(config))
        for include, launch in zip(launches, launch_args, strict=True):
            assert include.get("if") == "$(var nyx_enabled)"
            assert launch["mjcf"] == expected_mjcf
            assert launch["scene_dir"] == expected_scene_dir
            assert launch["topic_namespace"] == "$(var nyx_namespace)"
    if config.scene is not None:
        assert cameras <= _scene_cameras(config.scene)


def test_renderer_is_environment_only() -> None:
    """Launch selection and description selection use the same environment variable."""
    for config in CONFIGS:
        config_yaml = (config.directory / "config/config.yaml").read_text()
        assert not re.search(r"^\s*-\s*camera_renderer:\s*", config_yaml, re.MULTILINE)


def test_all_nyx_runtime_launches_are_covered_by_the_drift_test() -> None:
    """A new Nyx launch file cannot silently omit this cross-config contract."""
    expected = {config.directory / "launch/runtime.launch.xml" for config in CONFIGS}
    actual = {
        launch
        for launch in SOURCE_ROOT.glob("**/launch/runtime.launch.xml")
        if "nyx_camera.launch.py" in launch.read_text()
    }
    assert actual == expected
