"""Check color-stream ownership without changing MuJoCo depth or lidar sensors."""

import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

DESCRIPTION = (
    Path(__file__).resolve().parents[1] / "description/husky_a300_mujoco.xacro"
)


@pytest.mark.parametrize("renderer", ["", "opengl", "nyx", "both"])
def test_color_stream_has_one_renderer(renderer: str) -> None:
    """Keep OpenGL defaults or hand both color streams to the selected renderer."""
    result = subprocess.run(
        ["xacro", str(DESCRIPTION)],
        env={**os.environ, "MOVEIT_PRO_CAMERA_RENDERER": renderer},
        check=True,
        capture_output=True,
        text=True,
    )
    control = ET.fromstring(result.stdout).find("ros2_control")
    for camera in ("scene_camera", "oakd_color"):
        sensors = control.findall(f"sensor[@name='{camera}']")
        if renderer in ("", "opengl"):
            assert not sensors
            continue
        assert len(sensors) == 1
        params = {p.attrib["name"]: p.text for p in sensors[0]}
        if renderer == "nyx":
            assert params == {"color_enabled": "false"}
        else:
            assert params == {
                "rgb_topic": f"{camera}/opengl/color",
                "camera_info_topic": f"{camera}/opengl/camera_info",
            }
    for lidar in ("lidar_front", "lidar_rear"):
        assert (
            control.find(f"sensor[@name='{lidar}']/param[@name='points_topic']").text
            == f"/{lidar}/points"
        )


def test_unknown_renderer_fails_description() -> None:
    """Reject typos before starting a configuration with missing color streams."""
    result = subprocess.run(
        ["xacro", str(DESCRIPTION)],
        env={**os.environ, "MOVEIT_PRO_CAMERA_RENDERER": "invalid"},
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "MOVEIT_PRO_CAMERA_RENDERER" in result.stderr
    assert "opengl, nyx or both" in result.stderr
