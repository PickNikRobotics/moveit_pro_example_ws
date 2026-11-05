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
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
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


def _require(element, what):
    if element is None:
        raise SystemExit(f"generate_landmarks.py reads {what}, which is missing")
    return element


def _spawn_xy():
    """The default keyframe's chassis x, y from husky_scene.xml."""
    root = ET.parse(HERE / "husky_scene.xml").getroot()
    key = _require(
        root.find("keyframe/key[@name='default']"),
        "husky_scene.xml's keyframe 'default'",
    )
    x, y = (float(v) for v in key.get("qpos").split()[:2])
    return x, y


def _square_side_m():
    """The Dead Reckon Square Objective's side length; every side must be the same."""
    tree = ET.parse(HERE.parent / "objectives" / "dead_reckon_square.xml").getroot()
    sides = {
        float(a.get("distance"))
        for a in tree.iter("Action")
        if a.get("ID") == "DriveDistance"
    }
    if len(sides) != 1:
        raise SystemExit(
            f"dead_reckon_square.xml's DriveDistance sides are {sorted(sides)}; expected one length"
        )
    return sides.pop()


def _nav_goal_map():
    """objectives_integration_test.py's NAV_GOAL_XY, parsed without importing the ROS test module."""
    path = HERE.parent / "test" / "objectives_integration_test.py"
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", None) == "NAV_GOAL_XY" for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise SystemExit(f"{path.name} has no NAV_GOAL_XY")


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
LANE_M = BAND.LANE_M
# The A300's belly is 0.136 m up and its wheels 0.165 m in radius, so anything taller than this stops
# it. Rocks up to here are left for the wheels; anything taller must reach a lidar plane.
ROLL_OVER_M = BAND.TERRAIN.ROLL_OVER_M
SEAT_M = 0.01  # how far below the ground every base vertex sits, so no rock floats on a slope
SCAN_PLANES_M = BAND.SCAN_PLANES_M
SCAN_MARGIN_M = BAND.SCAN_MARGIN_M
BIG_BLOCK = BAND.BIG_BLOCK
BAND_BLOCKS = BAND.BAND_BLOCKS
MESH_REACH = BAND.MESH_REACH
# A 1.18 m crater from generate_terrain.py's default seed; main() checks it is still there.
FRESH_CRATER_WORLD = (3.19, -3.28)
FRESH_CRATER_RADIUS_M = 0.6
N_RIM, N_TALL, N_SMALL = 6, 16, 18


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
    points = np.vstack(rings)
    points[:, 2] -= points[
        :, 2
    ].min()  # base at z = 0, like the other meshes, for placement
    return points


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
            self.fields.append(
                (
                    gx - hx,
                    gx + hx,
                    gy - hy,
                    gy + hy,
                    BAND.hfield_png_to_z(ASSETS / file, elevation_z, gz),
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

    def around(self, x, y, r):
        """Ground at the centre and on a ring of radius r around it."""
        angles = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        return [self(x, y)] + [
            self(x + r * np.cos(a), y + r * np.sin(a)) for a in angles
        ]


def segment_distance(p, a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip((p - a) @ ab / (ab @ ab), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


# Exposed heights for scan-visible rocks: inside the gap between the two lidar planes' margins, or
# clear above the rear one.
TALL_BANDS_M = (
    (SCAN_PLANES_M[0] + SCAN_MARGIN_M + 0.01, SCAN_PLANES_M[1] - SCAN_MARGIN_M - 0.01),
    (SCAN_PLANES_M[1] + SCAN_MARGIN_M + 0.03, SCAN_PLANES_M[1] + SCAN_MARGIN_M + 0.26),
)


def exposure_fault(name, tops):
    """Why a landmark fails to either roll under the rover or stand clear of both scan planes'
    margins seen from any side, or None: `tops` are its height above the ground at its centre and
    around its reach.
    """
    for top in tops:
        if top > ROLL_OVER_M and any(
            abs(top - plane) < SCAN_MARGIN_M for plane in SCAN_PLANES_M
        ):
            return f"{name} stands {top:.3f} m on some side, within {SCAN_MARGIN_M} m of a lidar plane"
        if ROLL_OVER_M < top < SCAN_PLANES_M[0] + SCAN_MARGIN_M:
            return f"{name} stands {top:.3f} m on some side: too tall to roll over, too short to see"
    return None


def place(seed, ground):
    """Scattered rocks as (world xy, half-width, exposed height). Exposed height is what stands above
    the ground under the rock's centre; main() scales each mesh so that is what it ends up as.
    """
    rng = np.random.default_rng(seed)
    spawn = np.array(SPAWN_XY)
    structures = BAND.structure_boxes()
    # The hand-placed blocks are fixed obstacles the scatter must keep its lanes from, like the base.
    hand_placed = [(BIG_BLOCK[1], BIG_BLOCK[2], BIG_BLOCK[4])] + [
        (b[2], b[3], b[5]) for b in BAND_BLOCKS
    ]
    for x, y, half_width in hand_placed:
        r = MESH_REACH * half_width
        structures.append((np.array([x - r, y - r]), np.array([x + r, y + r])))
    rocks = []

    def clear(w, size, top):
        reach = MESH_REACH * size
        try:
            centre, *ring = ground.around(w[0], w[1], reach)
        except ValueError:
            return False
        if exposure_fault(
            "candidate", [top + centre - min(ring), top + centre - max(ring)]
        ):
            return False
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
            placed += clear(where(), *make())
            if placed == want:
                return
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

    if not any(
        math.dist(FRESH_CRATER_WORLD, (cx, cy)) < 0.05
        for _, cx, cy, *_ in BAND.replay_centre_craters()
    ):
        raise SystemExit(
            f"generate_terrain.py no longer has a crater at {FRESH_CRATER_WORLD}; update FRESH_CRATER_WORLD"
        )
    shapes, mesa = mesh_library(np.random.default_rng(2026))
    extents = [p.max(0) - p.min(0) for p in shapes]
    mesa_extent = mesa.max(0) - mesa.min(0)

    ground = Ground()
    rocks, rng = place(args.seed, ground)
    assets, geoms = ["<mujocoinclude>"], ["<mujocoinclude>"]
    exposed = {}  # name -> (x, y, half-width, world top z), for the final check

    def add(name, mesh, extent, x, y, yaw, half_width, height, bury):
        """Place a rock whose top stands `height` above the ground under its centre."""
        sx = 2 * half_width / extent[0]
        sy = (
            sx * rng.uniform(0.75, 1.1)
            if name.startswith("landmark_rock")
            else 2 * half_width / extent[1]
        )
        # The top stands `height` above the ground under the centre. The base goes `bury` of the rock's
        # height below that ground, or deeper where a slope would leave its downhill edge in the air:
        # every point within the rock's reach then has the base at least SEAT_M below it.
        centre, *ring = ground.around(x, y, MESH_REACH * half_width)
        top = centre + height
        base = min(min(ring + [centre]) - SEAT_M, (centre - bury * top) / (1.0 - bury))
        sz = (top - base) / extent[2]
        z = base
        exposed[name] = (x, y, half_width, top)
        assets.append(
            f'  <mesh name="{name}" file="rocks/{mesh}.stl" scale="{sx:.4f} {sy:.4f} {sz:.4f}" />'
        )
        geoms.append(
            f'  <geom name="{name}" type="mesh" mesh="{name}" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'quat="{math.cos(yaw / 2):.6f} 0 0 {math.sin(yaw / 2):.6f}" material="regolith_ground" '
            'contype="1" conaffinity="1" />'
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
    # Every landmark as placed, from every side, before anything is written.
    for name, (x, y, half_width, top_z) in exposed.items():
        centre, *ring = ground.around(x, y, MESH_REACH * half_width)
        fault = exposure_fault(
            name, [top_z - centre, top_z - min(ring), top_z - max(ring)]
        )
        if fault:
            raise SystemExit(fault)
    assets.append("</mujocoinclude>\n")
    geoms.append("</mujocoinclude>\n")
    ROCKS_DIR.mkdir(parents=True, exist_ok=True)
    for i, points in enumerate(shapes):
        write_stl(ROCKS_DIR / f"landmark_{i}.stl", points)
    write_stl(ROCKS_DIR / "landmark_mesa.stl", mesa)
    (HERE / "landmarks_assets_generated.xml").write_text("\n".join(assets))
    (HERE / "landmarks_geoms_generated.xml").write_text("\n".join(geoms))
    tall_count = sum(1 for _, _, top in rocks if top > SCAN_PLANES_M[0] + SCAN_MARGIN_M)
    print(
        f"landmarks: {len(rocks)} rocks ({tall_count} above the front scan plane), 1 mesa, {len(BAND_BLOCKS)} band blocks"
    )


if __name__ == "__main__":
    main()
