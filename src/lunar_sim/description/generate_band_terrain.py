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
  - Out to 15 m it stays at the centre's level. Its last RIM_BERM_M rises into a low regolith
    berm, which the lidars see, and the rover cannot drive off the collision ground. The far
    horizon (generate_far_TERRAIN.py) emerges from under the berm at the same height.
  - The visual strips share the centre visual mesh's vertices along the 10 m edge and are built
    from the same low-passed data that generate_visual_TERRAIN.py's downsample produces, so the
    drawn ground has no seam either.

Regenerate in this order, since each reads the one before:
  1. generate_terrain.py           the centre and its small rocks
  2. generate_visual_TERRAIN.py    the centre's render twin
  3. generate_far_TERRAIN.py       the far horizon
  4. generate_band_terrain.py      this band
  5. the band bodies' datums in moon_base_geoms.xml (by hand; validate_moon_base.py checks them)
  6. generate_landmarks.py         the landmark rocks, which sit on all of the above

Usage: python3 generate_band_terrain.py [--seed 1869]

Afterwards run the repo's prettier on the files it wrote (`pre-commit run prettier`), as
generate_terrain.py's generated XML needs.
"""
import argparse
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
RIM_BERM_HEIGHT_M = (
    0.35  # crest height: above both lidar planes' 0.1 m margin from below
)
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
    """FFT Gaussian blur (periodic); the band strips are far from the grid's wrap edges."""
    ky = np.fft.fftfreq(field.shape[0])[:, None]
    kx = np.fft.rfftfreq(field.shape[1])[None, :]
    kernel = np.exp(-2.0 * (np.pi * sigma_cells) ** 2 * (kx**2 + ky**2))
    return np.fft.irfft2(np.fft.rfft2(field) * kernel, s=field.shape)


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
        depth = rng.uniform(*TERRAIN.CRATER_DEPTH_RATIO) * d
        rim_h = rng.uniform(*TERRAIN.CRATER_RIM_RATIO) * d
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
    ground = scene_ground(HERE / "husky_scene.xml")
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
    # The rows on the 10 m edge stay the centre's own, so the two grounds share them exactly.
    beyond_edge = chebyshev > INNER_M + CELL_M / 2
    for d, cx, cy, depth, rim_h, e0 in edge_cut:
        stamp_on_grid(height, axis, x, y, d, cx, cy, depth, rim_h, e0, mask=beyond_edge)

    # The berm at the drivable edge: over the band's last RIM_BERM_M the ground rises to a crest that
    # both lidar planes see, then falls on its outer face to the far horizon's height at the edge, so
    # the band neither ends in a wall nor lets the rover drive off the collision ground.
    far_elev, far_base = ground["lunar_far_hfield"]
    far = hfield_png_to_z(ASSETS / "lunar_far_hfield.png", far_elev, far_base)
    far_size = far_half_extent(HERE / "husky_scene.xml")
    far_axis = np.linspace(-far_size, far_size, far.shape[0])
    crest = OUTER_M - RIM_BERM_M * 0.4
    inner_face = smoothstep(OUTER_M - RIM_BERM_M, crest, chebyshev)
    outer_face = smoothstep(crest, OUTER_M, chebyshev)
    ox = np.where(
        np.abs(x) >= np.abs(y),
        np.sign(x) * (OUTER_M + CELL_M),
        np.clip(x, -OUTER_M, OUTER_M),
    )
    oy = np.where(
        np.abs(y) > np.abs(x),
        np.sign(y) * (OUTER_M + CELL_M),
        np.clip(y, -OUTER_M, OUTER_M),
    )
    far_at_edge = (
        np.maximum(bilinear(far_axis, far, ox, oy), bilinear(far_axis, far, x, y))
        + 0.01
    )
    crest_z = height + RIM_BERM_HEIGHT_M * inner_face
    height = crest_z * (1 - outer_face) + far_at_edge * outer_face

    # The drawn version: the centre's committed collision data inside, the band outside, low-passed
    # the way the centre's visual twin is.
    drawn_source = height.copy()
    inside = ~band
    drawn_source[inside] = bilinear(centre_axis, centre, x[inside], y[inside])
    drawn = gaussian_blur(drawn_source, VISUAL_LOWPASS_M / CELL_M)
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
            f"the far horizon rises {poke.max():.3f} m through the band; raise RIM_BERM_HEIGHT_M "
            "or finish generate_far_terrain.py's ramp further out"
        )
    return axis, height, drawn, centre_visual, len(craters), len(edge_cut)


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


def write_hfield(grid, path):
    """8-bit PNG, row 0 at +y as MuJoCo reads it; MuJoCo keeps 8 bits per sample, so writing more
    would only mean the committed file and the loaded ground disagree. Returns (elevation_z, base_z).
    """
    z_min, z_max = float(grid.min()), float(grid.max())
    elevation_z = max(z_max - z_min, 1e-4)
    png = np.flipud(np.round((grid - z_min) / elevation_z * 255.0).astype(np.uint8))
    Image.fromarray(png, mode="L").save(path)
    return elevation_z, z_min


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=1869)
    args = parser.parse_args()

    axis, height, drawn, centre_visual, n_craters, n_edge = build(args.seed)
    assets, geoms = [None], ["<mujocoinclude>"]
    for side, (xr, yr) in COLLISION_STRIPS.items():
        name = f"lunar_band_{side}"
        elevation_z, base_z = write_hfield(
            collision_strip(axis, height, xr, yr), ASSETS / f"{name}.png"
        )
        hx, hy = (xr[1] - xr[0]) / 2, (yr[1] - yr[0]) / 2
        assets.append(
            f'  <hfield name="{name}" file="{name}.png" size="{hx:g} {hy:g} {elevation_z:.4f} 0.05" />'
        )
        geoms.append(
            f'  <geom name="{name}" type="hfield" hfield="{name}" pos="{(xr[0] + xr[1]) / 2:g} {(yr[0] + yr[1]) / 2:g} {base_z:.4f}" '
            f'material="regolith_ground" contype="1" conaffinity="1" group="3" />'
        )
    for side, (xr, yr) in VISUAL_STRIPS.items():
        name = f"lunar_band_{side}_visual"
        grid = visual_strip(axis, drawn, centre_visual, xr, yr)
        elevation_z, base_z = write_hfield(grid, ASSETS / f"{name}.png")
        hx, hy = (xr[1] - xr[0]) / 2, (yr[1] - yr[0]) / 2
        assets.append(
            f'  <hfield name="{name}" file="{name}.png" size="{hx:g} {hy:g} {elevation_z:.4f} 0.05" />'
        )
        geoms.append(
            f'  <geom name="{name}" type="hfield" hfield="{name}" pos="{(xr[0] + xr[1]) / 2:g} {(yr[0] + yr[1]) / 2:g} {base_z:.4f}" '
            f'material="regolith_ground" contype="0" conaffinity="0" group="2" />'
        )
    geoms.append("</mujocoinclude>\n")
    (HERE / "band_geoms_generated.xml").write_text("\n".join(geoms))
    scene = (HERE / "husky_scene.xml").read_text()
    begin, end = "<!-- BAND HFIELDS BEGIN", "<!-- BAND HFIELDS END -->"
    if scene.count(begin) != 1 or scene.count(end) != 1:
        raise SystemExit(f"husky_scene.xml needs exactly one {begin} ... {end} block")
    head, rest = scene.split(begin, 1)
    _, tail = rest.split(end, 1)
    block = (
        begin
        + " (generate_band_terrain.py) -->\n"
        + "\n".join("  " + a for a in assets[1:])
        + "\n    "
        + end
    )
    (HERE / "husky_scene.xml").write_text(head + block + tail)
    print(
        f"band: {n_craters} craters, {n_edge} edge-cut centre craters finished; z {height.min():.3f}..{height.max():.3f} m"
    )


if __name__ == "__main__":
    main()
