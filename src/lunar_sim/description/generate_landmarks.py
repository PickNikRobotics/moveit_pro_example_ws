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

"""Offline, seeded generator for lunar_sim's lidar landmark rocks.

Not run at simulation time; run manually and commit its outputs:
  - assets/rocks/landmark_<name>.stl  angular rock meshes (convex hulls of fractured point clouds)
  - landmarks_assets_generated.xml    <mesh> entries
  - landmarks_geoms_generated.xml     <geom> placements

Lunar rocks are impact ejecta: angular, flat-faced blocks, a few weathered sub-rounded ones, partly
buried, with sizes falling off as a power law. These are scattered at random over the cratered centre,
with a short blocky rim around one fresh crater, plus one large fractured block (a small mesa) and one
block per empty corner of the drivable band. They double as slam_toolbox and AMCL landmarks: tall rocks
reach above the front lidar's scan plane (0.46 m) by at least 0.1 m, and every top stays at least
0.1 m from both scan planes (0.46 m and 0.79 m) so pitch does not make a rock flicker in and out of a
scan. The rest stay below the front plane, as most real rocks would.

Placement keeps the field drivable: rocks taller than the rover can roll over stay LANE_M apart and
from the base structures, the Dead Reckon Square, the spawn and the integration test's Nav2 goal.

Usage: python3 generate_landmarks.py [--seed 47]

Afterwards run the repo's prettier on the files it wrote (`pre-commit run prettier`), as
generate_terrain.py's generated XML needs.
"""
import argparse
import ast
import importlib.util
import math
import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
ROCKS_DIR = ASSETS / "rocks"


def _band():
    spec = importlib.util.spec_from_file_location(
        "band", HERE / "generate_band_terrain.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _spawn_xy():
    """The default keyframe's chassis x, y from husky_scene.xml."""
    key = (
        ET.parse(HERE / "husky_scene.xml")
        .getroot()
        .find("keyframe/key[@name='default']")
    )
    x, y = (float(v) for v in key.get("qpos").split()[:2])
    return x, y


def _scan_planes_m():
    """Front and rear lidar heights above the ground: chassis_link's height plus each lidar link's."""
    root = ET.parse(HERE / "husky_a300.xml").getroot()
    chassis = float(root.find(".//body[@name='chassis_link']").get("pos").split()[2])
    return tuple(
        chassis + float(root.find(f".//body[@name='{name}']").get("pos").split()[2])
        for name in ("lidar_front_link", "lidar_rear_link")
    )


def _square_side_m():
    """The Dead Reckon Square Objective's side length."""
    tree = ET.parse(HERE.parent / "objectives" / "dead_reckon_square.xml").getroot()
    return float(tree.find(".//Action[@ID='DriveDistance']").get("distance"))


def _nav_goal_map():
    """objectives_integration_test.py's NAV_GOAL_XY, without importing the ROS test module."""
    source = (HERE.parent / "test" / "objectives_integration_test.py").read_text()
    return ast.literal_eval(
        re.search(r"^NAV_GOAL_XY = (\(.*\))$", source, re.M).group(1)
    )


BAND = _band()
INNER_M = BAND.INNER_M  # the cratered centre's half-extent; scattered rocks stay on it
SPAWN_XY = _spawn_xy()
SIDE = _square_side_m()
# Dead Reckon Square corners and the integration test's goal, in the map frame (origin at the spawn).
SQUARE_MAP = [(0.0, 0.0), (SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]
NAV_GOAL_MAP = _nav_goal_map()
ROVER_RADIUS_M = (
    0.6  # a disc containing the A300 footprint, bumpers and wheels included
)
LANE_M = (
    1.2  # clear gap between tall obstacles: the 0.7 m rover plus margin either side
)
# The A300's belly is 0.136 m up and its wheels 0.165 m in radius, so anything taller than this stops
# it. Rocks up to here are left for the wheels; anything taller must reach a lidar plane.
ROLL_OVER_M = 0.10
SCAN_PLANES_M = _scan_planes_m()
SCAN_MARGIN_M = 0.1
FRESH_CRATER_WORLD = (
    3.19,
    -3.28,
)  # a 1.18 m crater from generate_terrain.py's default seed
FRESH_CRATER_RADIUS_M = 0.6
N_RIM, N_TALL, N_SMALL = 6, 16, 18
# The large fractured block (griz's mesa): world x, y, yaw, half-width, exposed height. Hand placed in
# open ground well clear of the base.
BIG_BLOCK = ("landmark_mesa", 5.0, -12.6, 0.4, 1.7, 1.35)
# Band landmarks: name, mesh index, world x, y, yaw, half-width, exposed height. Each corner gets a
# different pair of shapes and sizes, so no two corners look alike to a scan matcher, and the long
# slab breaks up the otherwise empty north band.
BAND_BLOCKS = [
    ("landmark_band_nw", 0, -12.6, 12.4, 0.9, 0.85, 1.05),
    ("landmark_band_nw_low", 3, -9.7, 12.0, 2.4, 0.5, 0.62),
    ("landmark_band_ne", 2, 12.5, 12.6, 2.1, 0.6, 0.62),
    ("landmark_band_n_slab", 1, 3.0, 12.6, 0.05, 1.3, 0.64),
    ("landmark_band_sw", 5, -12.4, -12.3, 0.3, 0.95, 1.1),
    ("landmark_band_sw_low", 6, -10.0, -13.4, 1.0, 0.45, 0.63),
    ("landmark_band_e", 4, 12.8, -3.6, 1.4, 0.7, 1.0),
]


def fractured_block(rng, aspect, n=26):
    """A box-ish point cloud with a few planar fracture cuts shaved off; its hull has broad faces."""
    points = rng.uniform(-1, 1, (n * 6, 3)) * aspect
    for _ in range(rng.integers(3, 6)):
        normal = rng.standard_normal(3)
        normal /= np.linalg.norm(normal)
        offset = rng.uniform(0.35, 0.75) * np.abs(aspect) @ np.abs(normal)
        points = points[points @ normal < offset]
    points = points[ConvexHull(points).vertices]
    points[:, 2] -= points[:, 2].min()
    return points


def weathered(rng):
    points = fractured_block(
        rng, np.array([1.0, rng.uniform(0.7, 1.0), rng.uniform(0.6, 0.85)]), n=40
    )
    centre = points.mean(0)
    points = centre + (points - centre) * (1 - 0.18 * rng.random((len(points), 1)))
    points[:, 2] -= points[:, 2].min()
    return points


def big_block(rng):
    """Irregular outline, near-vertical broken sides and a flat, slightly stepped cap."""
    k = 9
    angle = np.sort(rng.uniform(0, 2 * np.pi, k))
    radius = rng.uniform(0.7, 1.0, k)
    rings = []
    for z, shrink in (
        (0.0, 1.0),
        (0.45, rng.uniform(0.96, 1.0)),
        (0.85, 0.94),
        (1.0, 0.88),
    ):
        jitter = rng.uniform(0.95, 1.05, k)
        rings.append(
            np.c_[
                radius * shrink * jitter * np.cos(angle),
                radius * shrink * jitter * np.sin(angle),
                np.full(k, z) + rng.uniform(-0.02, 0.02, k) + 0.05 * np.cos(angle),
            ]
        )
    return np.vstack(rings)


def write_stl(path, points):
    hull = ConvexHull(points)
    centre = points.mean(0)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(hull.simplices)))
        for tri in hull.simplices:
            a, b, c = points[tri]
            normal = np.cross(b - a, c - a)
            if normal @ (a - centre) < 0:
                b, c = c, b
                normal = -normal
            normal /= np.linalg.norm(normal)
            f.write(struct.pack("<12fH", *normal, *a, *b, *c, 0))
    lo, hi = points.min(0), points.max(0)
    return hi - lo


def mesh_library(rng):
    shapes = [
        fractured_block(rng, np.array([1.0, 0.8, 0.7])),
        fractured_block(rng, np.array([1.0, 0.65, 0.5])),
        fractured_block(rng, np.array([0.9, 0.9, 0.8])),
        fractured_block(rng, np.array([1.0, 0.75, 0.45])),
        fractured_block(rng, np.array([0.85, 0.7, 0.75])),
        weathered(rng),
        weathered(rng),
    ]
    return shapes, big_block(rng)


class Ground:
    """Collision ground height (centre plus band) from the committed heightmaps and husky_scene.xml."""

    def __init__(self):
        root = ET.parse(HERE / "husky_scene.xml").getroot()
        sizes = {
            h.get("name"): (h.get("file"), [float(v) for v in h.get("size").split()])
            for h in root.iter("hfield")
        }
        geoms = list(root.iter("geom")) + list(
            ET.parse(HERE / "band_geoms_generated.xml").getroot().iter("geom")
        )
        self.fields = []
        for geom in geoms:
            if geom.get("type") != "hfield" or geom.get("contype") == "0":
                continue
            file, (hx, hy, elevation_z, _) = sizes[geom.get("hfield")]
            gx, gy, gz = (float(v) for v in geom.get("pos").split())
            with Image.open(ASSETS / file) as image:
                raw = np.asarray(image).astype(np.float64)
            raw = raw[..., 0] if raw.ndim == 3 else raw
            full = 65535.0 if raw.max() > 255 else 255.0
            self.fields.append(
                (
                    gx - hx,
                    gx + hx,
                    gy - hy,
                    gy + hy,
                    gz + np.flipud(raw) / full * elevation_z,
                )
            )

    def __call__(self, x, y):
        for x0, x1, y0, y1, z in self.fields:
            if x0 <= x <= x1 and y0 <= y <= y1:
                fx = (x - x0) / (x1 - x0) * (z.shape[1] - 1)
                fy = (y - y0) / (y1 - y0) * (z.shape[0] - 1)
                j, i = min(int(fx), z.shape[1] - 2), min(int(fy), z.shape[0] - 2)
                tx, ty = fx - j, fy - i
                return float(
                    z[i, j] * (1 - tx) * (1 - ty)
                    + z[i, j + 1] * tx * (1 - ty)
                    + z[i + 1, j] * (1 - tx) * ty
                    + z[i + 1, j + 1] * tx * ty
                )
        raise ValueError(f"({x:.2f}, {y:.2f}) is outside the drivable ground")

    def lowest(self, x, y, r):
        return min(self(x + dx, y + dy) for dx in (-r, 0, r) for dy in (-r, 0, r))


def structure_boxes():
    """World-aligned XY boxes around each moon base body's geoms, from moon_base_geoms.xml."""
    boxes = []
    for body in ET.parse(HERE / "moon_base_geoms.xml").getroot().iter("body"):
        bx, by, _ = (float(v) for v in body.get("pos").split())
        yaw = float((body.get("euler") or "0 0 0").split()[2])
        c, s = math.cos(yaw), math.sin(yaw)
        corners = []
        for geom in body.iter("geom"):
            size = [float(v) for v in geom.get("size", "0").split()]
            if geom.get("fromto"):
                f = [float(v) for v in geom.get("fromto").split()]
                pts = [(f[0], f[1]), (f[3], f[4])]
                r = size[0]
                pts = [
                    (px + dx, py + dy)
                    for px, py in pts
                    for dx in (-r, r)
                    for dy in (-r, r)
                ]
            else:
                px, py, _ = (float(v) for v in (geom.get("pos") or "0 0 0").split())
                rx = size[0]
                ry = (
                    size[1]
                    if len(size) > 1 and geom.get("type") in ("box", "ellipsoid")
                    else size[0]
                )
                reach = math.hypot(rx, ry)
                pts = [
                    (px + dx, py + dy)
                    for dx in (-reach, reach)
                    for dy in (-reach, reach)
                ]
            corners += [(bx + c * px - s * py, by + s * px + c * py) for px, py in pts]
        corners = np.array(corners)
        boxes.append((corners.min(0), corners.max(0)))
    return boxes


def segment_distance(p, a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip((p - a) @ ab / (ab @ ab), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


# Horizontal reach of a scaled landmark mesh in units of its nominal half-width: the hulls reach up
# to 1.27x their x half-extent, and placement stretches y by up to 1.1x. Spacing uses the full reach.
MESH_REACH = 1.35
# Exposed heights for scan-visible rocks: inside the gap between the two lidar planes' margins, or
# clear above the rear one.
TALL_BANDS_M = (
    (SCAN_PLANES_M[0] + SCAN_MARGIN_M + 0.01, SCAN_PLANES_M[1] - SCAN_MARGIN_M - 0.01),
    (SCAN_PLANES_M[1] + SCAN_MARGIN_M + 0.03, SCAN_PLANES_M[1] + SCAN_MARGIN_M + 0.26),
)


def check_exposed(name, top):
    """Every landmark either rolls under the rover or stands clear of both scan planes' margins."""
    if top > ROLL_OVER_M and any(
        abs(top - plane) < SCAN_MARGIN_M for plane in SCAN_PLANES_M
    ):
        raise SystemExit(
            f"{name} stands {top:.3f} m, within {SCAN_MARGIN_M} m of a lidar plane"
        )
    if ROLL_OVER_M < top < SCAN_PLANES_M[0] + SCAN_MARGIN_M:
        raise SystemExit(
            f"{name} stands {top:.3f} m: too tall to roll over, too short for a lidar to see"
        )


def place(seed):
    """Scattered rocks as (world xy, half-width, exposed height). Exposed height is what stands above
    the ground under the rock's centre; main() scales each mesh so that is what it ends up as.
    """
    rng = np.random.default_rng(seed)
    spawn = np.array(SPAWN_XY)
    structures = structure_boxes()
    rocks = []

    def clear(w, size, top):
        reach = MESH_REACH * size
        if np.abs(w).max() > INNER_M - reach - 0.1:
            return False
        local = w - spawn
        square = min(
            segment_distance(local, a, b) for a, b in zip(SQUARE_MAP, SQUARE_MAP[1:])
        )
        if square < reach + ROVER_RADIUS_M + 0.8:
            return False
        if (
            np.linalg.norm(local - NAV_GOAL_MAP) < reach + LANE_M
            or np.linalg.norm(local) < reach + 1.5
        ):
            return False
        if any(
            np.linalg.norm(np.clip(w, lo, hi) - w) < reach + LANE_M
            for lo, hi in structures
        ):
            return False
        for q, q_size, q_top in rocks:
            gap = LANE_M if min(top, q_top) > ROLL_OVER_M else 0.3
            if np.linalg.norm(w - q) < reach + MESH_REACH * q_size + gap:
                return False
        rocks.append((w, size, top))
        return True

    def tall(scale=1.0):
        band = TALL_BANDS_M[int(rng.random() < 0.5)]
        return rng.uniform(0.35, 0.6) * scale, rng.uniform(*band)

    def small():
        size = min(0.14 * (1 - rng.random()) ** (-1 / 2.0), 0.3)
        return size, min(size * rng.uniform(0.4, 0.8), ROLL_OVER_M - 0.02)

    def fill(want, make, where, label, attempts):
        placed = 0
        for _ in range(attempts):
            if placed == want:
                return
            placed += clear(where(), *make())
        raise SystemExit(
            f"placed only {placed} of {want} {label}; loosen the spacing or ask for fewer"
        )

    def on_rim():
        angle = rng.uniform(0, 2 * math.pi)
        r = FRESH_CRATER_RADIUS_M * rng.uniform(1.2, 2.6)
        return np.array(FRESH_CRATER_WORLD) + r * np.array(
            [math.cos(angle), math.sin(angle)]
        )

    def anywhere():
        return np.array([rng.uniform(-9.6, 9.6), rng.uniform(-9.6, 9.6)])

    fill(
        N_RIM,
        lambda: tall(0.9) if rng.random() < 0.6 else small(),
        on_rim,
        "rim blocks",
        4000,
    )
    fill(N_TALL, tall, anywhere, "tall rocks", 30000)
    fill(N_SMALL, small, anywhere, "small rocks", 30000)
    return rocks, rng


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=47)
    args = parser.parse_args()

    shapes, mesa = mesh_library(np.random.default_rng(2026))
    ROCKS_DIR.mkdir(parents=True, exist_ok=True)
    extents = [
        write_stl(ROCKS_DIR / f"landmark_{i}.stl", p) for i, p in enumerate(shapes)
    ]
    mesa_extent = write_stl(ROCKS_DIR / "landmark_mesa.stl", mesa)

    rocks, rng = place(args.seed)
    ground = Ground()
    assets, geoms = ["<mujocoinclude>"], ["<mujocoinclude>"]

    def add(name, mesh, extent, x, y, yaw, half_width, height, bury):
        """`height` is what stands above the ground under the rock's centre; `bury` is the
        fraction of the mesh below it, so the mesh is scaled to height / (1 - bury)."""
        sx = 2 * half_width / extent[0]
        sy = (
            sx * rng.uniform(0.75, 1.1)
            if name.startswith("landmark_rock")
            else 2 * half_width / extent[1]
        )
        full = height / (1.0 - bury)
        sz = full / extent[2]
        z = ground(x, y) - bury * full
        assets.append(
            f'  <mesh name="{name}" file="rocks/{mesh}.stl" scale="{sx:.4f} {sy:.4f} {sz:.4f}" />'
        )
        geoms.append(
            f'  <geom name="{name}" type="mesh" mesh="{name}" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'quat="{math.cos(yaw / 2):.6f} 0 0 {math.sin(yaw / 2):.6f}" material="regolith_ground" '
            f'contype="1" conaffinity="1" />'
        )

    for i, (w, size, top) in enumerate(rocks):
        k = int(rng.integers(len(shapes)))
        add(
            f"landmark_rock_{i:02d}",
            f"landmark_{k}",
            extents[k],
            w[0],
            w[1],
            rng.uniform(0, math.pi),
            size,
            top,
            0.2,
        )
    name, x, y, yaw, half_width, height = BIG_BLOCK
    add(name, "landmark_mesa", mesa_extent, x, y, yaw, half_width, height, 0.09)
    for name, k, x, y, yaw, half_width, height in BAND_BLOCKS:
        add(name, f"landmark_{k}", extents[k], x, y, yaw, half_width, height, 0.2)
    assets.append("</mujocoinclude>\n")
    geoms.append("</mujocoinclude>\n")
    (HERE / "landmarks_assets_generated.xml").write_text("\n".join(assets))
    (HERE / "landmarks_geoms_generated.xml").write_text("\n".join(geoms))
    tall_count = sum(1 for _, _, top in rocks if top > SCAN_PLANES_M[0] + SCAN_MARGIN_M)
    for i, (_, _, top) in enumerate(rocks):
        check_exposed(f"landmark_rock_{i:02d}", top)
    check_exposed(BIG_BLOCK[0], BIG_BLOCK[-1])
    for block in BAND_BLOCKS:
        check_exposed(block[0], block[-1])
    print(
        f"landmarks: {len(rocks)} rocks ({tall_count} above the front scan plane), 1 mesa, {len(BAND_BLOCKS)} band blocks"
    )


if __name__ == "__main__":
    main()
