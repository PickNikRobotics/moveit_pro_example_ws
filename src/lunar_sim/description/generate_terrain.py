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

"""Offline, seedable generator for lunar_sim's cratered ground.

Not run at simulation time. Run manually (or in CI as an asset-refresh step)
and commit its outputs:
  - assets/lunar_hfield.png     grayscale MuJoCo <hfield> heightmap
  - assets/rocks/rock_N.stl     a small library of procedural rock meshes
  - rocks_assets_generated.xml  <mesh> asset entries for the rock library
  - rocks_geoms_generated.xml   scattered <geom> placements referencing them

Recipe (OmniLRS-style): a lunar crater size-frequency distribution (SFD)
stamped onto a low-frequency noise base, plus a rock library scattered with
Poisson-ish spacing. No terrain or placement is computed at MuJoCo load time
- both XML fragments and the PNG are ordinary committed assets, included by
husky_scene.xml via <include>.

Usage: python3 generate_terrain.py [--seed 869]

After regenerating, re-run the repo's prettier XML check/fix on the two
generated *_generated.xml files (`pre-commit run prettier`, see
.pre-commit-config.yaml), and if elevation_z or z_min changed, copy this
script's printed "hfield size attr" / "ground geom pos attr" lines into
husky_scene.xml's <hfield> and ground_plane <geom>.
"""
import argparse
import struct
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
ROCKS_DIR = ASSETS / "rocks"

# Plane geometry: matches the existing 20x20 m ground_plane (size="10 10 0.1").
PLANE_HALF_M = 10.0
CELL_M = 0.02  # ~2 cm cells per the task spec
N = round(2 * PLANE_HALF_M / CELL_M)  # 1000

# Crater SFD: truncated power law, N(>D) ~ D^-B, typical small-crater slope.
CRATER_B = 2.0
CRATER_DMIN_M = 0.05
CRATER_DMAX_M = 2.0
CRATER_COUNT = 600
CRATER_DEPTH_RATIO = (0.08, 0.15)  # depth/diameter, degraded regolith craterlets
CRATER_RIM_RATIO = (0.02, 0.06)  # rim height/diameter
CRATER_RIM_WIDTH = 0.3  # extra radius (x crater radius) the rim falloff extends over

# Low-frequency base undulation (metres of amplitude), several octaves.
NOISE_OCTAVES = [(4.0, 0.02), (1.5, 0.01), (0.5, 0.005)]  # (wavelength_m, amplitude_m)

# Rocks: excluded from the Dead Reckon Square footprint (~1.8 m square near the
# origin) plus margin, so the calibration recheck isolates the heightfield's
# effect and isn't confounded by a wheel snagging a rock.
ROCK_KEEPOUT_XMIN, ROCK_KEEPOUT_XMAX = -1.0, 3.0
ROCK_KEEPOUT_YMIN, ROCK_KEEPOUT_YMAX = -1.0, 3.0
ROCK_LIBRARY_SIZE = 6
ROCK_COUNT = 200
ROCK_RADIUS_RANGE_M = (0.03, 0.15)
ROCK_MIN_SPACING_M = 0.35
# MuJoCo has no per-geom mesh scale (only per-<mesh>-asset scale), so continuous
# per-rock radii are quantized to a handful of discrete asset scales, each shared
# by many geom instances - real instancing (one mesh in memory) rather than one
# unique STL per rock.
ROCK_SIZE_BUCKETS = np.geomspace(*ROCK_RADIUS_RANGE_M, 8)


def build_noise(rng: np.random.Generator) -> np.ndarray:
    """Sum of Gaussian-blurred white noise octaves -> smooth non-periodic undulation."""
    field = np.zeros((N, N), dtype=np.float64)
    for wavelength_m, amplitude_m in NOISE_OCTAVES:
        sigma_px = (wavelength_m / CELL_M) / (2 * np.pi)
        white = rng.standard_normal((N, N))
        smooth = gaussian_filter(white, sigma=sigma_px, mode="wrap")
        smooth /= smooth.std()
        field += amplitude_m * smooth
    return field


def sample_crater_diameters(rng: np.random.Generator, count: int) -> np.ndarray:
    u = rng.random(count)
    dmin_b, dmax_b = CRATER_DMIN_M**-CRATER_B, CRATER_DMAX_M**-CRATER_B
    return (u * (dmin_b - dmax_b) + dmax_b) ** (-1.0 / CRATER_B)


def stamp_craters(height_m: np.ndarray, rng: np.random.Generator) -> None:
    diameters = sample_crater_diameters(rng, CRATER_COUNT)
    cx_all = rng.uniform(-PLANE_HALF_M, PLANE_HALF_M, CRATER_COUNT)
    cy_all = rng.uniform(-PLANE_HALF_M, PLANE_HALF_M, CRATER_COUNT)

    for d, cx, cy in zip(diameters, cx_all, cy_all):
        radius_m = d / 2.0
        depth_m = rng.uniform(*CRATER_DEPTH_RATIO) * d
        rim_h_m = rng.uniform(*CRATER_RIM_RATIO) * d
        outer_m = radius_m * (1.0 + CRATER_RIM_WIDTH)

        px = round((cx + PLANE_HALF_M) / CELL_M)
        py = round((cy + PLANE_HALF_M) / CELL_M)
        span = int(np.ceil(outer_m / CELL_M)) + 1
        x0, x1 = max(0, px - span), min(N, px + span)
        y0, y1 = max(0, py - span), min(N, py + span)
        if x0 >= x1 or y0 >= y1:
            continue

        xs = (np.arange(x0, x1) * CELL_M - PLANE_HALF_M) - cx
        ys = (np.arange(y0, y1) * CELL_M - PLANE_HALF_M) - cy
        xx, yy = np.meshgrid(xs, ys, indexing="ij")
        r = np.sqrt(xx**2 + yy**2)
        u = r / radius_m

        # Reference elevation each crater cuts/builds from: the pre-existing
        # local terrain at its own center, not a global datum, so craters
        # stamped later correctly cut across earlier rims instead of just
        # summing without bound.
        e0 = height_m[min(px, N - 1), min(py, N - 1)]

        patch = height_m[x0:x1, y0:y1]
        bowl_mask = u <= 1.0
        bowl = e0 + depth_m * (u**2 - 1.0)
        patch[bowl_mask] = np.minimum(patch[bowl_mask], bowl[bowl_mask])

        # The bowl reaches e0 at u=1 while the rim starts at its full crest there, so each
        # crater ends in a one-cell vertical step of rim_h_m. Accepted, previously measured
        # artifact: worst case over the committed seed-869 field is a 0.102 m step across one
        # 2 cm cell. Part of what the Dead Reckon Square calibration in README.md measures.
        rim_mask = (u > 1.0) & (u <= 1.0 + CRATER_RIM_WIDTH)
        t = (u[rim_mask] - 1.0) / CRATER_RIM_WIDTH
        rim = e0 + rim_h_m * np.cos(t * np.pi / 2.0) ** 2
        patch[rim_mask] = np.maximum(patch[rim_mask], rim)


def build_heightfield(seed: int) -> tuple[np.ndarray, float, float]:
    """Returns (height_m [x,y]-indexed in metres, elevation_z range, z_min)."""
    rng = np.random.default_rng(seed)
    height_m = build_noise(rng)
    stamp_craters(height_m, rng)
    z_min, z_max = height_m.min(), height_m.max()
    return height_m, float(z_max - z_min), float(z_min)


def height_to_png(height_m: np.ndarray, elevation_z: float, z_min: float) -> np.ndarray:
    values01 = (height_m - z_min) / elevation_z
    png = np.clip(np.round(values01 * 255), 0, 255).astype(np.uint8)
    # An hfield's rows run along +y and its columns along +x, but MuJoCo flips a
    # PNG's rows on load, so image row 0 lands at *maximum* y. Transpose for
    # image (row=y, col=x) and reverse the rows, so world (x, y) reads back as
    # height_m[x_index, y_index] - the convention height_at() samples rock z at.
    return png.T[::-1, :]


# --- Procedural rock meshes (no mesh-library/trimesh dependency needed) ---

_PHI = (1 + 5**0.5) / 2
_ICO_VERTS = np.array(
    [
        (-1, _PHI, 0),
        (1, _PHI, 0),
        (-1, -_PHI, 0),
        (1, -_PHI, 0),
        (0, -1, _PHI),
        (0, 1, _PHI),
        (0, -1, -_PHI),
        (0, 1, -_PHI),
        (_PHI, 0, -1),
        (_PHI, 0, 1),
        (-_PHI, 0, -1),
        (-_PHI, 0, 1),
    ],
    dtype=np.float64,
)
_ICO_VERTS /= np.linalg.norm(_ICO_VERTS[0])
_ICO_FACES = [
    (0, 11, 5),
    (0, 5, 1),
    (0, 1, 7),
    (0, 7, 10),
    (0, 10, 11),
    (1, 5, 9),
    (5, 11, 4),
    (11, 10, 2),
    (10, 7, 6),
    (7, 1, 8),
    (3, 9, 4),
    (3, 4, 2),
    (3, 2, 6),
    (3, 6, 8),
    (3, 8, 9),
    (4, 9, 5),
    (2, 4, 11),
    (6, 2, 10),
    (8, 6, 7),
    (9, 8, 1),
]


def icosphere(subdiv: int) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    verts = list(map(tuple, _ICO_VERTS))
    faces = list(_ICO_FACES)
    for _ in range(subdiv):
        cache: dict[tuple[int, int], int] = {}

        def midpoint(a: int, b: int) -> int:
            key = (min(a, b), max(a, b))
            if key in cache:
                return cache[key]
            m = np.array(verts[a]) + np.array(verts[b])
            m /= np.linalg.norm(m)
            verts.append(tuple(m))
            idx = len(verts) - 1
            cache[key] = idx
            return idx

        new_faces = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    return np.array(verts), faces


def make_rock_mesh(
    rng: np.random.Generator,
) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    verts, faces = icosphere(subdiv=2)
    # A few random low-order "bumps" (dot products with random axes) plus
    # per-vertex jitter approximate an irregular, non-spherical rock without
    # needing a real spherical-harmonics or mesh-library dependency.
    radius = np.ones(len(verts))
    for _ in range(3):
        axis = rng.standard_normal(3)
        axis /= np.linalg.norm(axis)
        amp = rng.uniform(0.08, 0.22)
        radius += amp * (verts @ axis)
    radius += rng.uniform(0.03, 0.08, len(verts))
    radius = np.clip(radius, 0.35, None)
    verts = verts * radius[:, None]

    # Anisotropic stretch so rocks aren't all near-spherical.
    scale = rng.uniform(0.6, 1.3, 3)
    verts = verts * scale
    verts /= np.abs(verts).max()  # normalize back to a unit-ish bounding box
    return verts, faces


def write_stl(path: Path, verts: np.ndarray, faces: list[tuple[int, int, int]]) -> None:
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            v0, v1, v2 = verts[a], verts[b], verts[c]
            normal = np.cross(v1 - v0, v2 - v0)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            f.write(struct.pack("<3f", *normal))
            for v in (v0, v1, v2):
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


def in_rock_keepout(x: float, y: float) -> bool:
    return (
        ROCK_KEEPOUT_XMIN <= x <= ROCK_KEEPOUT_XMAX
        and ROCK_KEEPOUT_YMIN <= y <= ROCK_KEEPOUT_YMAX
    )


def scatter_rocks(rng: np.random.Generator) -> list[dict]:
    placements = []
    accepted_xy = []
    attempts = 0
    while len(placements) < ROCK_COUNT and attempts < ROCK_COUNT * 50:
        attempts += 1
        x = rng.uniform(-PLANE_HALF_M + 0.3, PLANE_HALF_M - 0.3)
        y = rng.uniform(-PLANE_HALF_M + 0.3, PLANE_HALF_M - 0.3)
        if in_rock_keepout(x, y):
            continue
        if any(
            (x - ax) ** 2 + (y - ay) ** 2 < ROCK_MIN_SPACING_M**2
            for ax, ay in accepted_xy
        ):
            continue
        accepted_xy.append((x, y))
        radius = rng.uniform(*ROCK_RADIUS_RANGE_M)
        bucket = int(np.argmin(np.abs(ROCK_SIZE_BUCKETS - radius)))
        yaw = rng.uniform(0, 2 * np.pi)
        variant = rng.integers(0, ROCK_LIBRARY_SIZE)
        placements.append(
            {
                "x": x,
                "y": y,
                "radius": float(ROCK_SIZE_BUCKETS[bucket]),
                "bucket": bucket,
                "yaw": yaw,
                "variant": int(variant),
            }
        )
    return placements


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))


def height_at(height_m: np.ndarray, x: float, y: float) -> float:
    """Nearest-cell sample of the (untransposed, [x,y]-indexed) height array in world
    metres. World z reconstructs height_m directly once the hfield geom's pos.z is set
    to z_min (see main()), so no extra offset is added here."""
    fx = (x + PLANE_HALF_M) / CELL_M
    fy = (y + PLANE_HALF_M) / CELL_M
    ix, iy = int(np.clip(fx, 0, N - 2)), int(np.clip(fy, 0, N - 2))
    return float(height_m[ix, iy])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=869)
    args = parser.parse_args()

    ASSETS.mkdir(parents=True, exist_ok=True)
    ROCKS_DIR.mkdir(parents=True, exist_ok=True)

    height_m, elevation_z, z_min = build_heightfield(args.seed)
    png_t = height_to_png(height_m, elevation_z, z_min)
    # world_z = geom.pos.z + value01*elevation_z = geom.pos.z + (height_m - z_min);
    # setting geom.pos.z = z_min makes world_z reconstruct height_m exactly, so the
    # noise-only baseline (height_m ~ 0) lands at world z=0, matching the old flat plane.
    ground_pos_z = z_min

    Image.fromarray(png_t, mode="L").save(ASSETS / "lunar_hfield.png")

    rock_rng = np.random.default_rng(args.seed + 1)
    rock_meshes = [make_rock_mesh(rock_rng) for _ in range(ROCK_LIBRARY_SIZE)]
    for i, (verts, faces) in enumerate(rock_meshes):
        write_stl(ROCKS_DIR / f"rock_{i}.stl", verts, faces)

    placements = scatter_rocks(rock_rng)

    # One <mesh> asset per (variant, size bucket) combo, each loaded once and shared
    # by every geom instance at that size - real instancing, not per-rock geometry.
    assets_xml = ["<mujocoinclude>"]
    for i in range(ROCK_LIBRARY_SIZE):
        for b, s in enumerate(ROCK_SIZE_BUCKETS):
            assets_xml.append(
                f'  <mesh name="rock_{i}_{b}" file="rocks/rock_{i}.stl" scale="{s:.4f} {s:.4f} {s:.4f}" />'
            )
    assets_xml.append("</mujocoinclude>\n")
    (HERE / "rocks_assets_generated.xml").write_text("\n".join(assets_xml))

    geoms_xml = ["<mujocoinclude>"]
    for i, p in enumerate(placements):
        z = height_at(height_m, p["x"], p["y"])
        qw, qx, qy, qz = quat_from_yaw(p["yaw"])
        geoms_xml.append(
            f'  <geom name="rock_{i:04d}" type="mesh" mesh="rock_{p["variant"]}_{p["bucket"]}" '
            f'pos="{p["x"]:.4f} {p["y"]:.4f} {z:.4f}" '
            f'quat="{qw:.6f} {qx:.6f} {qy:.6f} {qz:.6f}" '
            f'material="regolith_ground" contype="1" conaffinity="1" />'
        )
    geoms_xml.append("</mujocoinclude>\n")
    (HERE / "rocks_geoms_generated.xml").write_text("\n".join(geoms_xml))

    print(f"seed={args.seed}")
    print(
        f"heightfield: {N}x{N} px, cell={CELL_M * 100:.2f} cm, elevation_z={elevation_z:.4f} m, "
        f"ground_pos_z={ground_pos_z:.4f}"
    )
    print(
        f"craters stamped: {CRATER_COUNT}, diameter range sampled [{CRATER_DMIN_M},{CRATER_DMAX_M}] m"
    )
    print(f"rocks placed: {len(placements)} / requested {ROCK_COUNT}")
    print(
        f'hfield size attr for MJCF: "{PLANE_HALF_M} {PLANE_HALF_M} {elevation_z:.4f} 0.1"'
    )
    print(f'ground geom pos attr for MJCF: "0 0 {ground_pos_z:.4f}"')


if __name__ == "__main__":
    main()
