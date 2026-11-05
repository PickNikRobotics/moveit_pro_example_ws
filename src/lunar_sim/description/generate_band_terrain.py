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

"""Offline, seeded generator for lunar_sim's drivable outer band (10-15 m from the centre).

Not run at simulation time; run manually and commit its outputs:
  - assets/lunar_band_<side>.png         collision heightfields, 2 cm cells (group 3, unrendered)
  - assets/lunar_band_<side>_visual.png  render heightfields at the centre visual mesh's pitch
  - band_geoms_generated.xml             <geom> entries
  - husky_scene.xml's <hfield> block between the BAND HFIELDS markers, rewritten in place. An
    <include> of them would not work from both a relative and an absolute scene path: MuJoCo resolves
    an included <hfield> file against the include's own directory, and then prefixes meshdir.

The band continues the cratered centre past its edge with generate_terrain.py's own recipe (the same
noise octaves and crater size-frequency distribution, at the same density), so the drivable area
grows from 20x20 m to 30x30 m without regenerating the centre or its calibration:
  - At the 10 m edge it starts from the centre's exact rim heights and blends into its own terrain
    over BLEND_M, with the rim low-passed first so a crater cut by the edge is not extruded outward.
    Craters the edge cuts are finished here from generate_terrain.py's replayed crater list.
  - Out to 15 m it stays at the centre's level. Its last RIM_BERM_M rises into a regolith berm
    whose steep inner face the front lidar sees from near its foot, so the rover cannot drive off
    the collision ground. The far horizon (generate_far_terrain.py) emerges from under the berm at
    the same height.
  - The visual strips share the centre visual mesh's vertices along the 10 m edge and are built
    from the same low-passed data that generate_visual_terrain.py's downsample produces, so the
    drawn ground has no seam either.

Regenerate in this order, since each reads the one before:
  1. generate_terrain.py           the centre and its small rocks
  2. generate_visual_terrain.py    the centre's render twin
  3. generate_far_terrain.py       the far horizon
  4. generate_band_terrain.py      this band
  5. the band bodies' datums in moon_base_geoms.xml (by hand; validate_moon_base.py checks them)
  6. generate_landmarks.py         the landmark rocks, which sit on all of the above

Usage: python3 generate_band_terrain.py [--seed 1869]

Afterwards run the repo's prettier on the files it wrote (`pre-commit run prettier`), as
generate_terrain.py's generated XML needs.
"""
import argparse
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
SCENE_XML = HERE / "husky_scene.xml"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _require(element, what):
    if element is None:
        raise SystemExit(f"{Path(__file__).name} reads {what}, which is missing")
    return element


def scan_planes_m():
    """Front and rear lidar heights above the ground: chassis_link's, each lidar link's and each
    lidar camera's own offset, added up."""
    root = ET.parse(HERE / "husky_a300.xml").getroot()

    def z(element):
        return float((element.get("pos") or "0 0 0").split()[2])

    chassis = z(
        _require(
            root.find(".//body[@name='chassis_link']"), "husky_a300.xml's chassis_link"
        )
    )
    planes = []
    for link, camera in (
        ("lidar_front_link", "lidar_front"),
        ("lidar_rear_link", "lidar_rear"),
    ):
        body = _require(
            root.find(f".//body[@name='{link}']"), f"husky_a300.xml's {link}"
        )
        lidar = _require(
            body.find(f"camera[@name='{camera}']"), f"husky_a300.xml's camera {camera}"
        )
        planes.append(chassis + z(body) + z(lidar))
    return tuple(planes)


TERRAIN = load_module("generate_terrain")
VISUAL = load_module("generate_visual_terrain")
FAR = load_module("generate_far_terrain")
INNER_M = TERRAIN.PLANE_HALF_M  # the centre's half-extent
OUTER_M = FAR.NEAR_HALF_M  # the drivable edge: the far horizon emerges just past it
CELL_M = TERRAIN.CELL_M  # wheels see the same resolution across the edge
VISUAL_GRID = VISUAL.DEFAULT_GRID  # the centre's render twin, cells per side
RIM_BERM_M = (
    2.0  # width of the berm at the drivable edge: inner face, crest, outer face
)
SCAN_PLANES_M = scan_planes_m()
SCAN_MARGIN_M = 0.1  # how far a landmark's top keeps from a scan plane, so pitch does not flicker it
# The berm's crest stands above the ground at its foot by a height the front lidar sees and the rear
# one stays clear of, by SCAN_MARGIN_M each: the rule generate_landmarks.py applies to rocks.
BERM_TOP_RANGE_M = (SCAN_PLANES_M[0] + SCAN_MARGIN_M, SCAN_PLANES_M[1] - SCAN_MARGIN_M)
RIM_BERM_HEIGHT_M = sum(BERM_TOP_RANGE_M) / 2
# The inner face climbs to the crest this quickly so the front lidar sees the berm within about 0.28 m
# of its foot: a gentler face leaves a strip the costmaps call free beside blocks near the edge, and the
# rover wedges in it.
BERM_INNER_FACE_M = 0.4
# The berm's foot wanders up to this far outward, over these wavelengths, so its 30 m sides give scan
# matching distinct segments to lock onto along them. Only outward, and not within reach of anything
# standing near the edge, so the gaps beside blocks and buildings stay as narrow as they are.
BERM_WOBBLE_M = 0.6
BERM_WOBBLE_WAVELENGTHS_M = (3.0, 5.0, 8.0)
BERM_WOBBLE_CORNER_M = 3.0  # flat this far from each corner, where two sides meet
# Along the edge, flat within the first of these of an edge object's span and full past the second.
BERM_WOBBLE_CLEAR_M = (0.5, 1.5)
# A man-made berm leaves a way out: an opening on the north edge by the habitat, continued as a walled
# road whose walls end where it runs onto the far horizon's ground.
ROAD_X_M = FAR.ROAD_X_M
ROAD_HALF_M = FAR.ROAD_HALF_M  # half-width including the walls
ROAD_END_M = FAR.ROAD_END_M
ROAD_LANE_HALF_M = ROAD_HALF_M - RIM_BERM_M  # half the opening, between the walls' feet
# The walls end in a face as steep as the berm's inner face, so the front lidar sees each end as it
# sees the berm; a gentler end would leave a ridge too tall to roll over and too low to see.
ROAD_END_FACE_M = BERM_INNER_FACE_M
ROAD_FALL_M = 1.0  # and over its last this-much the road meets the far horizon's ground
FAR_CLEARANCE_M = 0.01  # how far the band stays above the far horizon where they meet
ROAD_GRADE_M = (
    1.0  # how far past the 15 m edge the road blends from the band's ground to level
)
if abs(ROAD_X_M) + ROAD_HALF_M > INNER_M:
    raise SystemExit(
        "the access road must stay within the north strip's +-10 m span, which its seam rows come from"
    )
if OUTER_M + ROAD_GRADE_M > ROAD_END_M - ROAD_END_FACE_M:
    raise SystemExit(
        "the access road needs room for its grade, then full walls, then their end faces: check ROAD_END_M "
        "in generate_far_terrain.py against ROAD_GRADE_M and ROAD_END_FACE_M"
    )
LANE_M = (
    1.2  # clear gap between tall obstacles: the 0.7 m rover plus margin either side
)
# Horizontal reach of a scaled landmark mesh in units of its nominal half-width: the hulls reach up
# to 1.27x their x half-extent, and placement stretches y by up to 1.1x. Spacing uses the full reach.
MESH_REACH = 1.35
# The large fractured block (griz's mesa): world x, y, yaw, half-width, exposed height. Hand placed in
# open ground well clear of the base.
BIG_BLOCK = ("landmark_mesa", 5.0, -10.4, 0.4, 1.7, 1.35)
# Band landmarks: name, mesh index, world x, y, yaw, half-width, exposed height. Each corner gets a
# different pair of shapes and sizes, so no two corners look alike to a scan matcher, and the wide
# slab breaks up the otherwise empty north band.
BAND_BLOCKS = [
    ("landmark_band_nw", 0, -11.6, 11.6, 0.9, 0.85, 1.05),
    ("landmark_band_nw_low", 3, -8.6, 12.0, 2.4, 0.5, 1.0),
    ("landmark_band_ne", 2, 11.9, 11.9, 2.1, 0.6, 0.62),
    ("landmark_band_n_slab", 1, 3.0, 11.2, 0.05, 1.3, 1.05),
    ("landmark_band_sw", 5, -11.4, -11.4, 0.3, 0.95, 1.1),
    ("landmark_band_sw_low", 6, -8.8, -12.1, 1.0, 0.45, 0.63),
    ("landmark_band_e", 4, 11.8, -2.4, 1.4, 0.7, 1.0),
]


# Gaussian equivalent of the centre's LANCZOS 1000 -> 350 downsample, in metres.
VISUAL_LOWPASS_M = 0.42 * (2 * INNER_M / (VISUAL_GRID - 1))

BLEND_M = (
    0.6  # distance over which the centre's rim hands over to the band's own terrain
)
RIM_LOWPASS_M = (
    0.3  # half-width of the rim smoothing that keeps edge-cut craters from extruding
)
# Collision strips (corners belong to north/south); visual strips add separate corners so each one
# shares the centre's vertex pitch along the 10 m edge it touches.
COLLISION_STRIPS = {
    "n": ((-OUTER_M, OUTER_M), (INNER_M, OUTER_M)),
    "s": ((-OUTER_M, OUTER_M), (-OUTER_M, -INNER_M)),
    "e": ((INNER_M, OUTER_M), (-INNER_M, INNER_M)),
    "w": ((-OUTER_M, -INNER_M), (-INNER_M, INNER_M)),
}
VISUAL_STRIPS = {
    "n": ((-INNER_M, INNER_M), (INNER_M, OUTER_M)),
    "s": ((-INNER_M, INNER_M), (-OUTER_M, -INNER_M)),
    "e": ((INNER_M, OUTER_M), (-INNER_M, INNER_M)),
    "w": ((-OUTER_M, -INNER_M), (-INNER_M, INNER_M)),
    "ne": ((INNER_M, OUTER_M), (INNER_M, OUTER_M)),
    "nw": ((-OUTER_M, -INNER_M), (INNER_M, OUTER_M)),
    "se": ((INNER_M, OUTER_M), (-OUTER_M, -INNER_M)),
    "sw": ((-OUTER_M, -INNER_M), (-OUTER_M, -INNER_M)),
}


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def gaussian_blur(field, sigma_cells):
    """FFT Gaussian blur, periodic: each outer edge wraps onto the opposite one. Fine for noise; use
    blur_unwrapped for terrain."""
    ky = np.fft.fftfreq(field.shape[0])[:, None]
    kx = np.fft.rfftfreq(field.shape[1])[None, :]
    kernel = np.exp(-2.0 * (np.pi * sigma_cells) ** 2 * (kx**2 + ky**2))
    return np.fft.irfft2(np.fft.rfft2(field) * kernel, s=field.shape)


def blur_unwrapped(field, sigma_cells):
    """gaussian_blur with edge padding, so no outer edge bleeds onto the opposite one."""
    pad = 4 * int(np.ceil(sigma_cells))
    return gaussian_blur(np.pad(field, pad, mode="edge"), sigma_cells)[
        pad:-pad, pad:-pad
    ]


def bilinear(axis, grid, xq, yq):
    """Sample `grid` [row=y, col=x] on a square `axis` at (xq, yq)."""
    step = axis[1] - axis[0]
    fx = np.clip((xq - axis[0]) / step, 0, len(axis) - 1.001)
    fy = np.clip((yq - axis[0]) / step, 0, len(axis) - 1.001)
    i, j = fy.astype(int), fx.astype(int)
    ty, tx = fy - i, fx - j
    return (
        grid[i, j] * (1 - tx) * (1 - ty)
        + grid[i, j + 1] * tx * (1 - ty)
        + grid[i + 1, j] * (1 - tx) * ty
        + grid[i + 1, j + 1] * tx * ty
    )


def hfield_png_to_z(path, elevation_z, base_z):
    """World z [row=y from -half, col=x from -half] of an hfield PNG as MuJoCo loads it: MuJoCo 3.6
    keeps 8 bits per sample whatever the file's depth, so a 16-bit PNG is rounded to 1/255 steps.
    """
    with Image.open(path) as image:
        raw = np.asarray(image).astype(np.float64)
    if raw.ndim == 3:
        raw = raw[..., 0]
    full_scale = 65535.0 if raw.max() > 255 else 255.0
    levels = np.round(raw / full_scale * 255.0) / 255.0
    return base_z + np.flipud(levels) * elevation_z


def replay_centre_craters():
    """generate_terrain.py's craters for its default seed, each with its reference elevation, from a
    replay of the generator itself; refuses to continue if the replay no longer reproduces the
    committed heightmap."""
    rng = np.random.default_rng(TERRAIN.DEFAULT_SEED)
    height = TERRAIN.build_noise(rng)
    craters = TERRAIN.stamp_craters(height, rng)
    committed = np.asarray(Image.open(ASSETS / "lunar_hfield.png")).astype(float)
    replayed = TERRAIN.height_to_png(
        height, float(height.max() - height.min()), float(height.min())
    )
    if not np.array_equal(replayed.astype(float), committed):
        raise SystemExit(
            "replaying generate_terrain.py does not reproduce assets/lunar_hfield.png; "
            "regenerate the centre first, or update this replay to match it"
        )
    return craters


def band_craters(rng, area_m2):
    """Craters at the centre's density with its size-frequency distribution, none touching the edge."""
    count = int(round(TERRAIN.CRATER_COUNT / (2 * TERRAIN.PLANE_HALF_M) ** 2 * area_m2))
    diameters = TERRAIN.sample_crater_diameters(rng, count)
    craters = []
    for d in diameters:
        while True:
            cx, cy = rng.uniform(-OUTER_M, OUTER_M, 2)
            if max(abs(cx), abs(cy)) > INNER_M + 0.5 + 0.65 * d:
                break
        depth, rim_h = TERRAIN.crater_depth_rim(rng, d)
        craters.append((d, cx, cy, depth, rim_h))
    return craters


def stamp_on_grid(height, axis, x, y, d, cx, cy, depth, rim_h, e0, mask=None):
    """generate_terrain.py's crater on this script's [row=y, col=x] grid; `mask` limits where it may
    cut or build."""
    radius = d / 2.0
    outer = radius * (1.0 + TERRAIN.CRATER_RIM_WIDTH)
    i0, i1 = np.searchsorted(axis, cy - outer), np.searchsorted(axis, cy + outer)
    j0, j1 = np.searchsorted(axis, cx - outer), np.searchsorted(axis, cx + outer)
    if i1 <= i0 or j1 <= j0:
        return
    window = (slice(i0, i1), slice(j0, j1))
    u = np.hypot(x[window] - cx, y[window] - cy) / radius
    TERRAIN.crater_profile(
        height[window], u, e0, depth, rim_h, None if mask is None else mask[window]
    )


def far_half_extent(scene_xml):
    root = ET.parse(scene_xml).getroot()
    return next(
        float(h.get("size").split()[0])
        for h in root.iter("hfield")
        if h.get("name") == "lunar_far_hfield"
    )


def scene_ground(scene_xml):
    """(elevation_z, base_z) for lunar_hfield, lunar_hfield_visual and lunar_far_hfield from the scene."""
    root = ET.parse(scene_xml).getroot()
    sizes = {
        h.get("name"): [float(v) for v in h.get("size").split()]
        for h in root.iter("hfield")
    }
    pos_z = {
        g.get("hfield"): float(g.get("pos").split()[2])
        for g in root.iter("geom")
        if g.get("type") == "hfield"
    }
    return {name: (sizes[name][2], pos_z[name]) for name in sizes if name in pos_z}


def build(seed):
    ground = scene_ground(SCENE_XML)
    rng = np.random.default_rng(seed)

    n = round(2 * OUTER_M / CELL_M) + 1
    axis = np.linspace(-OUTER_M, OUTER_M, n)
    x, y = np.meshgrid(axis, axis)
    chebyshev = np.maximum(np.abs(x), np.abs(y))
    band = chebyshev >= INNER_M - 1e-9

    # The band's own terrain: generate_terrain.py's noise octaves, normalised over the band so every
    # side has the centre's amplitude, and its crater distribution at the centre's density.
    detail = np.zeros_like(x)
    for wavelength_m, amplitude_m in TERRAIN.NOISE_OCTAVES:
        smooth = gaussian_blur(
            rng.standard_normal(x.shape), (wavelength_m / CELL_M) / (2 * np.pi)
        )
        detail += amplitude_m * smooth / smooth[band].std()
    area_m2 = (2 * OUTER_M) ** 2 - (2 * INNER_M) ** 2
    craters = band_craters(rng, area_m2)
    for d, cx, cy, depth, rim_h in craters:
        i = min(np.searchsorted(axis, cy), n - 1)
        j = min(np.searchsorted(axis, cx), n - 1)
        stamp_on_grid(detail, axis, x, y, d, cx, cy, depth, rim_h, detail[i, j])

    # The centre's rim along each 10 m edge, from its committed collision heightmap.
    elev, base = ground["lunar_hfield"]
    centre = hfield_png_to_z(ASSETS / "lunar_hfield.png", elev, base)
    centre_axis = np.linspace(-INNER_M, INNER_M, centre.shape[0])
    edge_t = np.linspace(-INNER_M, INNER_M, round(2 * INNER_M / CELL_M) + 1)
    lim = INNER_M - 1e-6
    rims = {
        "n": bilinear(centre_axis, centre, edge_t, np.full_like(edge_t, lim)),
        "s": bilinear(centre_axis, centre, edge_t, np.full_like(edge_t, -lim)),
        "e": bilinear(centre_axis, centre, np.full_like(edge_t, lim), edge_t),
        "w": bilinear(centre_axis, centre, np.full_like(edge_t, -lim), edge_t),
    }
    half_cells = int(round(RIM_LOWPASS_M / CELL_M))
    kernel = np.ones(2 * half_cells + 1) / (2 * half_cells + 1)
    soft = {
        k: np.convolve(np.pad(v, half_cells, mode="edge"), kernel, mode="valid")
        for k, v in rims.items()
    }
    px, py = np.clip(x, -INNER_M, INNER_M), np.clip(y, -INNER_M, INNER_M)
    dist = np.hypot(x - px, y - py)
    # Which edge a cell hands over from: the nearer one, with the north and south edges owning the
    # rows at y = +-10 m (those rows border the centre's north and south rims, not its corners).
    north_south = np.abs(y) >= np.abs(x) - 1e-9

    def along_edge(profiles):
        return np.where(
            north_south,
            np.where(
                y > 0,
                np.interp(px, edge_t, profiles["n"]),
                np.interp(px, edge_t, profiles["s"]),
            ),
            np.where(
                x > 0,
                np.interp(py, edge_t, profiles["e"]),
                np.interp(py, edge_t, profiles["w"]),
            ),
        )

    half_blend = smoothstep(0.0, BLEND_M * 0.5, dist)
    rim = along_edge(rims) * (1 - half_blend) + along_edge(soft) * half_blend
    rim_mean = float(np.mean(np.concatenate(list(rims.values()))))
    own = rim_mean + (detail - detail[band].mean())
    blend = smoothstep(0.0, BLEND_M, dist)
    height = rim * (1 - blend) + own * blend

    # Finish the centre's craters that the 10 m edge cuts, outside the edge only.
    edge_cut = [
        c
        for c in replay_centre_craters()
        if max(abs(c[1]), abs(c[2])) + c[0] / 2 * (1 + TERRAIN.CRATER_RIM_WIDTH)
        > INNER_M
    ]
    # The rows on the 10 m edge stay resampled from the centre's, so the two grounds meet there.
    beyond_edge = chebyshev > INNER_M + CELL_M / 2
    for d, cx, cy, depth, rim_h, e0 in edge_cut:
        stamp_on_grid(height, axis, x, y, d, cx, cy, depth, rim_h, e0, mask=beyond_edge)

    # The berm at the drivable edge: over the band's last RIM_BERM_M the ground rises to a crest the
    # front lidar sees and the rover cannot climb, then falls on its outer face to the far horizon's
    # height at the edge, so the band does not end in a wall. The same walls line the access road.
    far_elev, far_base = ground["lunar_far_hfield"]
    far = hfield_png_to_z(ASSETS / "lunar_far_hfield.png", far_elev, far_base)
    far_size = far_half_extent(SCENE_XML)
    far_axis = np.linspace(-far_size, far_size, far.shape[0])
    wobble = berm_wobble(seed + 1, axis)
    ground_z = height
    # Past the 15 m edge the road is graded level across its lane, so both walls stand the same height
    # above it; it blends in over ROAD_GRADE_M from the band's own row at the edge.
    lane_cols = np.abs(axis - ROAD_X_M) <= ROAD_LANE_HALF_M
    road_level = float(height[-1, lane_cols].mean())

    def ground_at(px, py):
        natural = bilinear(axis, ground_z, px, np.minimum(py, OUTER_M))
        graded = smoothstep(OUTER_M, OUTER_M + ROAD_GRADE_M, py)
        fall = smoothstep(ROAD_END_M - ROAD_FALL_M, ROAD_END_M, py)
        road_z = natural * (1 - graded) + road_level * graded
        return (
            road_z * (1 - fall)
            + (bilinear(far_axis, far, px, py) + FAR_CLEARANCE_M) * fall
        )

    def walled(gx, gy):
        offset = wobble_at(wobble, axis, gx, gy)
        depth, fx, fy = wall_depth(gx, gy, offset)
        foot_z = ground_at(fx, fy)
        base = ground_at(gx, gy)
        edge_z = far_at_edge(far_axis, far, gx, gy)
        rise = RIM_BERM_HEIGHT_M * (
            1 - smoothstep(ROAD_END_M - ROAD_END_FACE_M, ROAD_END_M, gy)
        )
        return (
            wall_profile(
                base, foot_z, depth, FAR.footprint_depth(gx, gy), edge_z, rise
            ),
            depth,
            foot_z,
        )

    (height, crest_z), depth, foot_z = walled(x, y)
    check_berm(depth, crest_z, foot_z)

    # The road beyond the 15 m edge, on the band's 2 cm lattice so its first row is the band's last.
    cols = (axis >= ROAD_X_M - ROAD_HALF_M - 1e-9) & (
        axis <= ROAD_X_M + ROAD_HALF_M + 1e-9
    )
    road_x = axis[cols]
    road_y = OUTER_M + CELL_M * np.arange(round((ROAD_END_M - OUTER_M) / CELL_M) + 1)
    rx, ry = np.meshgrid(road_x, road_y)
    (road, road_crest), road_depth, road_foot = walled(rx, ry)
    check_berm(road_depth, road_crest, road_foot, ry < ROAD_END_M - ROAD_END_FACE_M)
    road[0] = height[-1, cols]

    # The drawn version: the centre's committed collision data inside, the band outside, low-passed
    # the way the centre's visual twin is.
    drawn_source = height.copy()
    inside = ~band
    drawn_source[inside] = bilinear(centre_axis, centre, x[inside], y[inside])
    drawn = blur_unwrapped(drawn_source, VISUAL_LOWPASS_M / CELL_M)
    vis_elev, vis_base = ground["lunar_hfield_visual"]
    centre_visual = hfield_png_to_z(
        ASSETS / "lunar_hfield_visual.png", vis_elev, vis_base
    )
    if centre_visual.shape != (VISUAL_GRID, VISUAL_GRID):
        raise SystemExit(
            f"lunar_hfield_visual.png is {centre_visual.shape}, expected {VISUAL_GRID} square"
        )

    # The far horizon has to stay under the band wherever the band is drawn.
    poke = bilinear(far_axis, far, x[band], y[band]) - drawn[band]
    if poke.max() > 0:
        raise SystemExit(
            f"the far horizon rises {poke.max():.3f} m through the band; finish generate_far_terrain.py's "
            "ramp further out"
        )
    # The road's drawn version, low-passed with a metre of the band below it.
    below = round(1.0 / CELL_M)
    patch = np.vstack([height[-below - 1 : -1, cols], road])
    road_drawn = blur_unwrapped(patch, VISUAL_LOWPASS_M / CELL_M)[below:]
    poke = bilinear(far_axis, far, rx, ry) - road_drawn
    if poke.max() > 0:
        raise SystemExit(
            f"the far horizon rises {poke.max():.3f} m through the access road; extend "
            "generate_far_terrain.py's footprint over it"
        )
    road_strips = (road_x, road_y, road, road_drawn)
    return axis, height, drawn, centre_visual, road_strips, len(craters), len(edge_cut)


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


def edge_objects():
    """World XY boxes of everything standing in the band: the moon base bodies and the hand-placed
    blocks."""
    hand_placed = [(BIG_BLOCK[1], BIG_BLOCK[2], BIG_BLOCK[4])] + [
        (b[2], b[3], b[5]) for b in BAND_BLOCKS
    ]
    boxes = structure_boxes()
    for x, y, half_width in hand_placed:
        r = MESH_REACH * half_width
        boxes.append((np.array([x - r, y - r]), np.array([x + r, y + r])))
    return boxes


def berm_wobble(seed, t):
    """Per side, how far the berm's foot sits outward of its nominal line at each along-edge t."""
    rng = np.random.default_rng(seed)
    foot_m = OUTER_M - RIM_BERM_M
    # Anything reaching past this leaves less than a lane to the foot; further in, moving the foot out
    # only widens a lane.
    near_m = foot_m - LANE_M
    corner = smoothstep(OUTER_M, OUTER_M - BERM_WOBBLE_CORNER_M, np.abs(t))
    # The opening is kept straight, like the stretches beside blocks and buildings.
    gate = (
        np.array([ROAD_X_M - ROAD_HALF_M, foot_m]),
        np.array([ROAD_X_M + ROAD_HALF_M, OUTER_M]),
    )
    boxes = edge_objects() + [gate]
    wobble = {}
    for side in "nsew":
        wave = sum(
            np.sin(2 * np.pi * t / w + rng.uniform(0, 2 * np.pi))
            for w in BERM_WOBBLE_WAVELENGTHS_M
        )
        wave = (wave - wave.min()) / (wave.max() - wave.min())
        clear = corner.copy()
        for lo, hi in boxes:
            reach = {"n": hi[1], "s": -lo[1], "e": hi[0], "w": -lo[0]}[side]
            if reach < near_m:
                continue
            a, b = (lo[0], hi[0]) if side in "ns" else (lo[1], hi[1])
            gap = np.maximum(np.maximum(a - t, t - b), 0.0)
            clear *= smoothstep(*BERM_WOBBLE_CLEAR_M, gap)
        wobble[side] = BERM_WOBBLE_M * wave * clear
    return wobble


def wobble_at(wobble, t_axis, x, y):
    """The foot's outward offset for each point, from the side it belongs to."""
    north_south = np.abs(y) >= np.abs(x) - 1e-9
    return np.where(
        north_south,
        np.where(
            y > 0, np.interp(x, t_axis, wobble["n"]), np.interp(x, t_axis, wobble["s"])
        ),
        np.where(
            x > 0, np.interp(y, t_axis, wobble["e"]), np.interp(y, t_axis, wobble["w"])
        ),
    )


def wall_depth(x, y, offset):
    """How far past the drivable ground's edge each point is, and the foot point straight in from it.

    The drivable ground is the square inside the berm's foot (OUTER_M - RIM_BERM_M, moved out by
    `offset`) plus the road's lane north to its open end; distances are Chebyshev, matching the
    berm along each edge.
    """
    foot = OUTER_M - RIM_BERM_M + offset
    north_south = np.abs(y) >= np.abs(x) - 1e-9
    square = np.maximum(np.maximum(np.abs(x), np.abs(y)) - foot, 0.0)
    square_fx = np.where(north_south, np.clip(x, -foot, foot), np.sign(x) * foot)
    square_fy = np.where(north_south, np.sign(y) * foot, np.clip(y, -foot, foot))
    lane_lo = (
        OUTER_M - RIM_BERM_M - 1.0
    )  # the lane starts inside the square, so the two pieces join
    lane = np.maximum(
        np.abs(x - ROAD_X_M) - ROAD_LANE_HALF_M, np.maximum(lane_lo - y, 0.0)
    )
    lane_fx = np.clip(x, ROAD_X_M - ROAD_LANE_HALF_M, ROAD_X_M + ROAD_LANE_HALF_M)
    lane_fy = np.maximum(y, lane_lo)
    nearer_lane = lane < square
    return (
        np.where(nearer_lane, lane, square),
        np.where(nearer_lane, lane_fx, square_fx),
        np.where(nearer_lane, lane_fy, square_fy),
    )


def wall_profile(ground, foot_z, depth, edge_depth, edge_z, rise=RIM_BERM_HEIGHT_M):
    """Ground with the berm on it: the inner face climbs over BERM_INNER_FACE_M to a crest `rise` above the foot, then the outer face falls to `edge_z` where the footprint ends (`edge_depth` reaches 0). Returns (height, height before the outer face)."""
    inner = smoothstep(0.0, BERM_INNER_FACE_M, depth)
    crest_z = ground * (1 - inner) + (foot_z + rise) * inner
    beyond = np.maximum(depth - BERM_INNER_FACE_M, 0.0)
    span = beyond + np.maximum(edge_depth, 0.0)
    outer = smoothstep(
        0.0, 1.0, np.divide(beyond, span, out=np.zeros_like(span), where=span > 0)
    )
    return crest_z * (1 - outer) + edge_z * outer, crest_z


def far_at_edge(far_axis, far, x, y):
    """The far horizon's height where the footprint ends nearest each point, plus FAR_CLEARANCE_M."""
    square_depth = OUTER_M - np.maximum(np.abs(x), np.abs(y))
    side = ROAD_HALF_M - np.abs(x - ROAD_X_M)
    end = ROAD_END_M - y
    on_road = (y > 0) & (np.minimum(side, end) > square_depth)
    ox = np.where(
        on_road,
        np.where(
            side < end, ROAD_X_M + np.sign(x - ROAD_X_M) * (ROAD_HALF_M + CELL_M), x
        ),
        np.where(
            np.abs(x) >= np.abs(y),
            np.sign(x) * (OUTER_M + CELL_M),
            np.clip(x, -OUTER_M, OUTER_M),
        ),
    )
    oy = np.where(
        on_road,
        np.where(side < end, y, ROAD_END_M + CELL_M),
        np.where(
            np.abs(y) > np.abs(x),
            np.sign(y) * (OUTER_M + CELL_M),
            np.clip(y, -OUTER_M, OUTER_M),
        ),
    )
    return (
        np.maximum(bilinear(far_axis, far, ox, oy), bilinear(far_axis, far, x, y))
        + FAR_CLEARANCE_M
    )


def check_berm(depth, crest_z, foot_z, mask=True):
    """Along every wall (where `mask` holds), the crest stands within BERM_TOP_RANGE_M of the ground at its foot."""
    crest = (np.abs(depth - BERM_INNER_FACE_M) < CELL_M / 2) & mask
    if not crest.any():
        raise SystemExit("check_berm found no wall crest to check")
    top = (crest_z - foot_z)[crest]
    lo, hi = BERM_TOP_RANGE_M
    if top.min() < lo or top.max() > hi:
        raise SystemExit(
            f"the edge berm stands {top.min():.3f}..{top.max():.3f} m above its foot, outside "
            f"{lo:.3f}..{hi:.3f} m"
        )


def collision_strip(axis, height, xr, yr):
    j0, j1 = int(np.abs(axis - xr[0]).argmin()), int(np.abs(axis - xr[1]).argmin()) + 1
    i0, i1 = int(np.abs(axis - yr[0]).argmin()), int(np.abs(axis - yr[1]).argmin()) + 1
    return height[i0:i1, j0:j1]


def visual_strip(axis, drawn, centre_visual, xr, yr):
    """Vertices at the centre visual mesh's pitch, coinciding with its vertices along the 10 m edge,
    whose heights there are copied from it."""
    pitch = 2 * INNER_M / (VISUAL_GRID - 1)
    nx = int(round((xr[1] - xr[0]) / pitch)) + 1
    ny = int(round((yr[1] - yr[0]) / pitch)) + 1
    xs, ys = np.linspace(xr[0], xr[1], nx), np.linspace(yr[0], yr[1], ny)
    gx, gy = np.meshgrid(xs, ys)
    grid = bilinear(axis, drawn, gx, gy)
    for i in range(ny):
        for j in range(nx):
            px, py = gx[i, j], gy[i, j]
            on_x = abs(abs(px) - INNER_M) < 1e-6 and abs(py) <= INNER_M + 1e-6
            on_y = abs(abs(py) - INNER_M) < 1e-6 and abs(px) <= INNER_M + 1e-6
            if on_x or on_y:
                ci = int(round((np.clip(py, -INNER_M, INNER_M) + INNER_M) / pitch))
                cj = int(round((np.clip(px, -INNER_M, INNER_M) + INNER_M) / pitch))
                grid[i, j] = centre_visual[ci, cj]
    return grid


def road_strip_pair(road_strips, strips):
    """The access road's collision strip, and its visual strip at the centre visual pitch, whose
    first row is the north visual strip's last so the two drawn grounds meet."""
    road_x, road_y, road, road_drawn = road_strips
    collision = (
        "lunar_band_road",
        road,
        (float(road_x[0]), float(road_x[-1])),
        (float(road_y[0]), float(road_y[-1])),
        'contype="1" conaffinity="1" group="3"',
    )
    pitch = 2 * INNER_M / (VISUAL_GRID - 1)
    k0 = int(np.floor((road_x[0] + INNER_M) / pitch + 1e-9))
    k1 = int(np.ceil((road_x[-1] + INNER_M) / pitch - 1e-9))
    xs = -INNER_M + pitch * np.arange(k0, k1 + 1)
    ys = np.linspace(
        road_y[0], road_y[-1], int(round((road_y[-1] - road_y[0]) / pitch)) + 1
    )
    gx, gy = np.meshgrid(xs, ys)
    fx = np.clip((gx - road_x[0]) / CELL_M, 0, len(road_x) - 1.001)
    fy = np.clip((gy - road_y[0]) / CELL_M, 0, len(road_y) - 1.001)
    i, j = fy.astype(int), fx.astype(int)
    ty, tx = fy - i, fx - j
    grid = (
        road_drawn[i, j] * (1 - tx) * (1 - ty)
        + road_drawn[i, j + 1] * tx * (1 - ty)
        + road_drawn[i + 1, j] * (1 - tx) * ty
        + road_drawn[i + 1, j + 1] * tx * ty
    )
    north = next(g for name, g, *_ in strips if name == "lunar_band_n_visual")
    grid[0] = north[-1, k0 : k1 + 1]
    visual = (
        "lunar_band_road_visual",
        grid,
        (float(xs[0]), float(xs[-1])),
        (float(ys[0]), float(ys[-1])),
        'contype="0" conaffinity="0" group="2"',
    )
    return [collision, visual]


def encode_hfield(grid):
    """8-bit image, row 0 at +y as MuJoCo reads it; MuJoCo keeps 8 bits per sample, so writing more
    would only mean the committed file and the loaded ground disagree. Returns (png, elevation_z, base_z).
    """
    z_min, z_max = float(grid.min()), float(grid.max())
    elevation_z = max(z_max - z_min, 1e-4)
    png = np.flipud(np.round((grid - z_min) / elevation_z * 255.0).astype(np.uint8))
    return png, elevation_z, z_min


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=1869)
    args = parser.parse_args()

    axis, height, drawn, centre_visual, road_strips, n_craters, n_edge = build(
        args.seed
    )
    strips = [
        (
            f"lunar_band_{side}",
            collision_strip(axis, height, xr, yr),
            xr,
            yr,
            'contype="1" conaffinity="1" group="3"',
        )
        for side, (xr, yr) in COLLISION_STRIPS.items()
    ] + [
        (
            f"lunar_band_{side}_visual",
            visual_strip(axis, drawn, centre_visual, xr, yr),
            xr,
            yr,
            'contype="0" conaffinity="0" group="2"',
        )
        for side, (xr, yr) in VISUAL_STRIPS.items()
    ]
    strips += road_strip_pair(road_strips, strips)
    images, assets, geoms = {}, [], ["<mujocoinclude>"]
    for name, grid, xr, yr, collision in strips:
        images[name], elevation_z, base_z = encode_hfield(grid)
        hx, hy = (xr[1] - xr[0]) / 2, (yr[1] - yr[0]) / 2
        assets.append(
            f'<hfield name="{name}" file="{name}.png" size="{hx:g} {hy:g} {elevation_z:.4f} 0.05" />'
        )
        geoms.append(
            f'  <geom name="{name}" type="hfield" hfield="{name}" pos="{(xr[0] + xr[1]) / 2:g} {(yr[0] + yr[1]) / 2:g} {base_z:.4f}" '
            f'material="regolith_ground" {collision} />'
        )
    geoms.append("</mujocoinclude>\n")
    scene = SCENE_XML.read_text()
    begin, end = "<!-- BAND HFIELDS BEGIN", "<!-- BAND HFIELDS END -->"
    if scene.count(begin) != 1 or scene.count(end) != 1:
        raise SystemExit(f"husky_scene.xml needs exactly one {begin} ... {end} block")
    head, rest = scene.split(begin, 1)
    _, tail = rest.split(end, 1)
    block = (
        begin
        + " (generate_band_terrain.py) -->\n"
        + "\n".join("    " + a for a in assets)
        + "\n    "
        + end
    )
    # Everything is built and checked before any file is touched.
    for name, png in images.items():
        Image.fromarray(png).save(ASSETS / f"{name}.png")
    (HERE / "band_geoms_generated.xml").write_text("\n".join(geoms))
    SCENE_XML.write_text(head + block + tail)
    print(
        f"band: {n_craters} craters, {n_edge} edge-cut centre craters finished; z {height.min():.3f}..{height.max():.3f} m"
    )


if __name__ == "__main__":
    main()
