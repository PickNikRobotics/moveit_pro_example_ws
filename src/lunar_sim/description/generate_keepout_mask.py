#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#    * Neither the name of the PickNik Inc. nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Generate lunar_sim's semantic Nav2 keep-out mask from fixed scene geometry.

The mask is independent of the localization occupancy map. It marks every fixed
moon-base structure, the scene-camera mast, and the inside-foot safety strip of
the authored berm and access-road walls. Natural terrain and rocks stay free.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent
SCENE = HERE / "husky_scene.xml"
MOON_BASE = HERE / "moon_base_geoms.xml"
NAV2_PARAMS = PACKAGE / "params" / "nav2_params.yaml"
DEFAULT_OUTPUT = PACKAGE / "maps"
RESOLUTION_M = 0.05
WORLD_BOUNDS_M = (-20.0, -20.0, 20.0, 24.0)
BERM_SAFETY_M = 0.30
# The garage remains enterable: only its three walls are fixed hazards.
GARAGE_GEOMS = {
    "moon_base_garage_wall_left",
    "moon_base_garage_wall_right",
    "moon_base_garage_wall_back",
}


def _values(element, attribute, default):
    """Parse a space-separated MJCF vector."""
    return np.array([float(value) for value in element.get(attribute, default).split()])


def _yaw(element):
    """Return yaw; roll about local X does not change a box's XY axes."""
    if element.get("euler"):
        _, pitch, yaw = _values(element, "euler", "0 0 0")
        if abs(pitch) > 1e-9:
            raise ValueError(f"{element.get('name')}: geom pitch is unsupported")
        return yaw
    return 0.0


def _rotate(points, yaw):
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    return points @ rotation.T


def _body_geom_polygons(body, selected=None):
    """Project a fixed body's selected primitive geoms into world XY."""
    body_pos = _values(body, "pos", "0 0 0")[:2]
    body_yaw = _yaw(body)
    polygons = []
    for geom in body.findall("geom"):
        if selected is not None and geom.get("name") not in selected:
            continue
        geom_pos = _values(geom, "pos", "0 0 0")[:2]
        geom_yaw = _yaw(geom)
        kind = geom.get("type", "sphere")
        size = _values(geom, "size", "0")
        if kind == "box":
            hx, hy = size[:2]
            polygon = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
            polygon = _rotate(polygon, geom_yaw)
        elif kind in {"cylinder", "capsule", "sphere"}:
            radius = size[0]
            angles = np.linspace(0.0, 2.0 * math.pi, 96, endpoint=False)
            polygon = np.column_stack((np.cos(angles), np.sin(angles))) * radius
            if geom.get("fromto"):
                values = _values(geom, "fromto", "0 0 0 0 0 0")
                a, b = values[:2], values[3:5]
                direction = b - a
                length = np.linalg.norm(direction)
                if length > 1e-9:
                    normal = radius * np.array([-direction[1], direction[0]]) / length
                    polygon = np.array([a + normal, b + normal, b - normal, a - normal])
                    geom_pos = np.zeros(2)
        else:
            raise ValueError(f"{geom.get('name')}: unsupported fixed geom type {kind}")
        polygons.append(_rotate(polygon + geom_pos, body_yaw) + body_pos)
    return polygons


def fixed_structure_polygons():
    """World polygons for all fixed non-traversable structures."""
    root = ET.parse(MOON_BASE).getroot()
    polygons = []
    for body in root.findall("body"):
        selected = GARAGE_GEOMS if body.get("name") == "moon_base_garage" else None
        polygons.extend(_body_geom_polygons(body, selected))
    scene = ET.parse(SCENE).getroot()
    mast = scene.find(".//body[@name='scene_camera_mast']")
    if mast is None:
        raise ValueError("husky_scene.xml has no scene_camera_mast body")
    polygons.extend(_body_geom_polygons(mast))
    return polygons


def _footprint_clearance_m():
    """All-yaw rover clearance from the Nav2 footprint and padding."""
    params = yaml.safe_load(NAV2_PARAMS.read_text())
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = np.array(yaml.safe_load(local["footprint"]), dtype=float)
    return float(np.linalg.norm(footprint, axis=1).max() + local["footprint_padding"])


def _spawn_pose():
    """World pose of map origin from the selected default keyframe."""
    key = ET.parse(SCENE).getroot().find("keyframe/key[@name='default']")
    if key is None:
        raise ValueError("husky_scene.xml has no default keyframe")
    x, y, _, qw, qx, qy, qz = (float(value) for value in key.get("qpos").split()[:7])
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return np.array([x, y]), yaw


def _world_grid():
    xmin, ymin, xmax, ymax = WORLD_BOUNDS_M
    width = round((xmax - xmin) / RESOLUTION_M)
    height = round((ymax - ymin) / RESOLUTION_M)
    xs = xmin + (np.arange(width) + 0.5) * RESOLUTION_M
    ys = ymin + (np.arange(height) + 0.5) * RESOLUTION_M
    return xs, ys


def _inside_polygon(x, y, polygon):
    """Vectorized point-in-polygon for cell centres."""
    inside = np.zeros(x.shape, dtype=bool)
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crosses = (yi > y) != (yj > y)
        at_x = (xj - xi) * (y - yi) / (yj - yi + np.finfo(float).eps) + xi
        inside ^= crosses & (x < at_x)
        j = i
    return inside


def _dilate(mask, radius_m):
    """Conservatively grow occupied cells by a circular radius."""
    radius = math.ceil((radius_m + RESOLUTION_M / math.sqrt(2.0)) / RESOLUTION_M)
    out = np.zeros_like(mask)
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if math.hypot(dx, dy) * RESOLUTION_M <= radius_m + RESOLUTION_M / math.sqrt(
                2.0
            ):
                src_x = slice(max(0, -dx), min(mask.shape[1], mask.shape[1] - dx))
                src_y = slice(max(0, -dy), min(mask.shape[0], mask.shape[0] - dy))
                dst_x = slice(max(0, dx), min(mask.shape[1], mask.shape[1] + dx))
                dst_y = slice(max(0, dy), min(mask.shape[0], mask.shape[0] + dy))
                out[dst_y, dst_x] |= mask[src_y, src_x]
    return out


def _berm_safety_mask(world_x, world_y):
    """Cells just inside the authored berm and road-wall feet, with lane/end open.

    The exact seeded foot wobbles up to 0.6 m outward. Marking that complete authored range keeps the
    strip conservative without importing the SciPy-based terrain generator into this package.
    """
    outer_m = 15.0
    berm_width_m = 2.0
    wobble_m = 0.6
    road_x_m = -4.5
    road_lane_half_m = 1.5
    road_end_m = 19.0
    foot_min = outer_m - berm_width_m
    square_distance = np.maximum(np.abs(world_x), np.abs(world_y))
    square_strip = (square_distance >= foot_min - BERM_SAFETY_M) & (
        square_distance <= foot_min + wobble_m
    )
    lane_lo = foot_min - 1.0
    in_opening = (np.abs(world_x - road_x_m) < road_lane_half_m) & (world_y >= lane_lo)
    square_strip &= ~in_opening

    left_foot = road_x_m - road_lane_half_m
    right_foot = road_x_m + road_lane_half_m
    road_span = (world_y >= lane_lo) & (world_y <= road_end_m)
    left_strip = (world_x >= left_foot) & (world_x <= left_foot + BERM_SAFETY_M)
    right_strip = (world_x <= right_foot) & (world_x >= right_foot - BERM_SAFETY_M)
    return square_strip | (road_span & (left_strip | right_strip))


def generate():
    """Return the PGM raster and YAML metadata."""
    xs, ys = _world_grid()
    world_x, world_y = np.meshgrid(xs, ys)
    occupied = np.zeros(world_x.shape, dtype=bool)
    for polygon in fixed_structure_polygons():
        lo, hi = polygon.min(axis=0), polygon.max(axis=0)
        selected = (
            (world_x >= lo[0])
            & (world_x <= hi[0])
            & (world_y >= lo[1])
            & (world_y <= hi[1])
        )
        if selected.any():
            occupied[selected] |= _inside_polygon(
                world_x[selected], world_y[selected], polygon
            )
    occupied = _dilate(occupied, _footprint_clearance_m())
    occupied |= _berm_safety_mask(world_x, world_y)

    spawn, yaw = _spawn_pose()
    if abs(yaw) > 1e-9:
        raise ValueError(
            "nonzero default-keyframe yaw is not supported by the axis-aligned mask"
        )
    xmin, ymin, _, _ = WORLD_BOUNDS_M
    image = np.where(np.flipud(occupied), 0, 254).astype(np.uint8)
    metadata = {
        "image": "lunar_keepout_mask.pgm",
        "mode": "trinary",
        "resolution": RESOLUTION_M,
        "origin": [float(xmin - spawn[0]), float(ymin - spawn[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    return image, metadata


def write(output):
    """Write the deterministic PGM/YAML pair."""
    output.mkdir(parents=True, exist_ok=True)
    image, metadata = generate()
    image_path = output / metadata["image"]
    Image.fromarray(image, mode="L").save(image_path)
    # pre-commit's generic end-of-file check requires a trailing newline even for binary PGM.
    with image_path.open("ab") as stream:
        stream.write(b"\n")
    (output / "lunar_keepout_mask.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False)
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    write(args.output)


if __name__ == "__main__":
    main()
