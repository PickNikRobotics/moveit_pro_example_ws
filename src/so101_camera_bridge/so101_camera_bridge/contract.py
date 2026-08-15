"""Pure validation and camera-model helpers for the RTSP bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StreamSpec:
    name: str
    url: str
    image_topic: str
    camera_info_topic: str
    frame_id: str


def validate_stream_spec(spec: StreamSpec) -> None:
    if not spec.name.strip():
        raise ValueError("stream name must not be empty")
    if not spec.url.startswith(("rtsp://", "rtsps://")):
        raise ValueError(f"{spec.name} URL must use rtsp:// or rtsps://")
    for label, topic in (
        ("image", spec.image_topic),
        ("camera_info", spec.camera_info_topic),
    ):
        if not topic.startswith("/") or topic.endswith("/"):
            raise ValueError(f"{spec.name} {label} topic must be an absolute ROS topic")
    if not spec.frame_id.strip() or spec.frame_id.startswith("/"):
        raise ValueError(
            f"{spec.name} frame_id must be non-empty and have no leading slash"
        )


def pinhole_projection(
    width: int, height: int, vertical_fov_degrees: float
) -> tuple[list[float], list[float]]:
    """Return approximate K and P matrices when physical calibration is absent."""
    if width <= 0 or height <= 0:
        raise ValueError("image width and height must be greater than zero")
    if (
        not math.isfinite(vertical_fov_degrees)
        or not 1.0 < vertical_fov_degrees < 179.0
    ):
        raise ValueError("vertical FOV must be finite and between 1 and 179 degrees")
    fy = (height / 2.0) / math.tan(math.radians(vertical_fov_degrees) / 2.0)
    fx = fy
    cx = (width - 1.0) / 2.0
    cy = (height - 1.0) / 2.0
    k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return k, p
