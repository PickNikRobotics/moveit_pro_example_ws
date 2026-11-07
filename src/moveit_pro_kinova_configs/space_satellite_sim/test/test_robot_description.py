# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The robot description expands the way the Runtime expands it at startup."""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.create_urdf import create_xml_from_xacro
from moveit_studio_utils_py.system_config import load_system_config


def _robot_description(tmp_path) -> ET.Element:
    """Expand the description the way the Runtime expands it at startup."""
    config, _, _ = load_system_config("space_satellite_sim", tmp_path)
    description = config.hardware.robot_description
    return ET.fromstring(
        create_xml_from_xacro(description.urdf, description.urdf_params)
    )


def test_robot_description_expands(tmp_path) -> None:
    assert _robot_description(tmp_path).tag == "robot"


def test_tag_detector_frames_and_sizes_match_the_scene_and_fuse_measurements() -> None:
    """Keep detector, static-transform and fuse tag-frame names in sync."""
    package = Path(get_package_share_directory("space_satellite_sim"))
    detector_launches = ("launch/36h11.launch.yml", "launch/36h11_wrist.launch.yml")
    detector_configs = ("tags_36h11.yaml", "tags_36h11_wrist.yaml")
    detector_tags = [
        yaml.safe_load((package / "launch" / config).read_text())["/**"][
            "ros__parameters"
        ]["tag"]
        for config in detector_configs
    ]
    fuse_launch_spec = importlib.util.spec_from_file_location(
        "fuse_launch", package / "launch/fuse.launch.py"
    )
    fuse_launch = importlib.util.module_from_spec(fuse_launch_spec)
    fuse_launch_spec.loader.exec_module(fuse_launch)
    sample_objective = ET.parse(package / "objectives/sample_april_tag.xml").getroot()
    fuse_frames = yaml.safe_load((package / "config/fuse/fuse.yaml").read_text())[
        "state_estimator"
    ]["ros__parameters"]["transform_sensor"]["transforms"]
    scene = ET.parse(package / "description/mujoco/scene.xml").getroot()
    marker_bodies = [
        body
        for body in scene.iter("body")
        if body.get("name", "").startswith("ar_marker_")
    ]
    marker_sizes = [
        geom.get("size").split()
        for body in marker_bodies
        for geom in body.findall("geom")
    ]
    assert [
        next(
            entry["node_container"]
            for entry in yaml.safe_load((package / launch).read_text())["launch"]
            if "node_container" in entry
        )["composable_node"][0]["param"][0]["from"]
        for launch in detector_launches
    ] == [
        f"$(find-pkg-share space_satellite_sim)/launch/{config}"
        for config in detector_configs
    ]
    assert [tag["frames"] for tag in detector_tags] == [
        [f"satellite_tag_{index}{suffix}" for index in range(3)]
        for suffix in ("", "_wrist")
    ]
    expected_tag_sizes = [
        0.8 * 2 * float(marker_size[0]) for marker_size in marker_sizes
    ]
    expected_tag_positions = [
        [float(value) for value in body.get("pos").split()[:2]]
        for body in marker_bodies
    ]
    for detector_tag in detector_tags:
        assert detector_tag["sizes"] == pytest.approx(expected_tag_sizes)
    assert (
        sample_objective.find(".//Action[@ID='DetectAprilTags']").get("tag_size")
        == "0.2"
    )
    static_transform_frames = {
        (
            f"satellite_tag_{index}{suffix}",
            f"target_frame_satellite_tag_{index}{suffix}",
        )
        for suffix in fuse_launch.TAG_SUFFIXES
        for index, _ in enumerate(fuse_launch.TAG_OFFSETS)
    }
    for offset, position in zip(
        fuse_launch.TAG_OFFSETS, expected_tag_positions, strict=True
    ):
        assert [float(value) for value in offset] == pytest.approx(position)
    expected_tag_frames = {
        f"satellite_tag_{index}{suffix}"
        for suffix in ("", "_wrist")
        for index in range(3)
    }
    assert static_transform_frames == {
        (tag_frame, f"target_frame_{tag_frame}") for tag_frame in expected_tag_frames
    }
    assert set(fuse_frames) == {
        f"satellite_tag_{index}{suffix}"
        for suffix in ("", "_wrist")
        for index in range(3)
    }
    assert marker_sizes == [["0.125", "0.125", "0.005"]] * 3


def test_calibrated_servicer_camera_has_platform_collision_exemptions(
    tmp_path,
) -> None:
    """The calibrated fixed camera's platform collision pairs stay exempted."""
    srdf = ET.parse(
        get_package_share_directory("space_satellite_sim")
        + "/config/moveit/space_satellite_sim.srdf"
    ).getroot()
    disabled = {
        frozenset((entry.get("link1"), entry.get("link2")))
        for entry in srdf.findall("disable_collisions")
    }
    camera_pairs = {
        frozenset(("servicer_camera_link", platform))
        for platform in (
            "servicer_platform",
            "epastar",
            "interface",
            "rpo_suite",
            "pole",
        )
    }
    assert camera_pairs <= disabled
    robot = _robot_description(tmp_path)
    assert {
        "servicer_camera_link",
        "servicer_platform",
        "epastar",
        "interface",
        "rpo_suite",
        "pole",
    } <= {link.get("name") for link in robot.findall("link")}
