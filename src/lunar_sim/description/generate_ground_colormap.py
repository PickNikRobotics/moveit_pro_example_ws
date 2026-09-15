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

"""Offline, seedable generator for lunar_sim's ground colour map (diffuse texture).

Not run at simulation time. Run manually and commit its output:
  - assets/lunar_regolith_untiled.png   the colour map husky_scene.xml's <texture> loads

Nine real Apollo Hasselblad frames (cropped, cleaned, committed under
assets/ground_colormap_sources/ - see provenance.txt written alongside the output) are randomly
placed, rotated (0/90/180/270deg) and mirrored across a grid covering the whole ground plane, each
placement jittered off the grid and its edges feathered into its neighbours, so no periodic tile
pitch exists anywhere on the plane (verified by verify_ground_colormap.py's autocorrelation/
template-match check) - unlike the single-photo texrepeat="0.3 0.3" tiling it replaces, which shows
an obvious repeating grid.

v2: each placed frame's contrast is additionally equalized to one global reference (see
equalize_contrast()) before placement - v1 matched every frame's mean but not its contrast, and the
nine source frames vary a lot in dynamic range (std 0.026-0.193 after mean-matching alone), so v1's
placements formed a visible light/dark patchwork at the frame pitch even though no single frame
repeated. Placements are also jittered off the nominal grid (see JITTER_FRAC_*) so there's no
regular spacing for the eye to lock onto, on top of the random rotation/mirroring.

Usage: python3 generate_ground_colormap.py [--plane-m 20] [--px-per-m 410] [--seed 869]

After regenerating, update husky_scene.xml's <texture file=...> if the output filename changed.
With texuniform="true", texrepeat is repeats per *metre* of world space, so drawing this map
exactly once over the plane needs texrepeat = 1/--plane-m on both axes ("0.05 0.05" at the
default 20 m) - recompute it there if --plane-m changes.
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter
from scipy.ndimage import uniform_filter1d

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
SOURCES = ASSETS / "ground_colormap_sources"

# Each source frame is resampled to represent this many metres of ground, independent of the
# final plane size/resolution - keeps per-frame upscale factor (see TILES below) constant as
# --plane-m/--px-per-m change.
PHYS_TILE_M = 1.2
SOURCE_TILE_PX = (
    2048  # working resolution per source frame before placement/downsampling
)
SEAM_FRAC = (
    320 / 2048
)  # feather zone as a fraction of tile size, matches prior single-plane build
FULL_FRAME_M = 1.75  # native ground coverage of a full (uncropped) Hasselblad frame at ~0.4m altitude

CRATER_MIN_SEP_M = 0.4
ROCK_MIN_SEP_M = 0.15

# v2: contrast equalization - one of the nine frames is the "global regolith reference" every
# other frame's contrast (std, around the shared 0.49 mean) is matched to, so no placement reads
# brighter/flatter or punchier than its neighbours regardless of the source photo's own exposure.
REFERENCE_TILE_NAME = "AS15-86-11671"

# v2: break the placement pitch - each cell's nominal grid position is jittered by a random offset
# of this many tile-widths (magnitude only; sign is random), independently in x and y. The nominal
# grid step is shrunk (see GRID_DENSITY) so tiles still overlap enough after the worst-case jitter
# that no gap in canvas coverage is possible.
JITTER_FRAC_MIN = 0.25
JITTER_FRAC_MAX = 0.40
# Nominal step as a fraction of tile_px - small enough that the ~(1/GRID_DENSITY)^2 nominal overlap
# multiplicity survives jitter this large without opening a coverage gap (checked empirically
# across seeds; 0.4 leaves real gaps, 0.3 is the first safe value, 0.25 keeps a comfortable
# margin). build_mosaic()'s min_coverage check below catches a regression if this ever needs
# retuning (bigger JITTER_FRAC_MAX, smaller tile_px, etc).
GRID_DENSITY = 0.25
# Direction a baked craterlet rim / rock cap is lit from, as an offset in this map's own array
# axes - (axis 0 = image row, axis 1 = image column), the order _local_patch returns. Derived from
# husky_scene.xml's <light dir="-0.2 0.2 -1"> (~74deg elevation), which is the direction light
# travels, so the sun sits toward world (+x, -y) and that is the side of a bump that lights up.
# MuJoCo maps this texture's image column onto world +x and its image row onto world -y (verified
# on both plane and hfield geoms by rendering a quadrant-coded texture over the ground), so world
# (+x, -y) is +column and +row: both components positive, i.e. down-and-right in image pixels.
SUN_DIR_PX = np.array([0.2, 0.2])
SUN_DIR_PX = SUN_DIR_PX / np.linalg.norm(SUN_DIR_PX)

# (name, source file, crop's own native full-frame width in px, reseau-line threshold or None)
TILES = [
    ("AS15-86-11671", "AS15-86-11671.png", 3920, None),
    ("AS15-87-11766", "AS15-87-11766.png", 3934, 4.0),
    ("AS15-87-11777", "AS15-87-11777.png", 3936, 4.0),
    ("AS17-146-22370", "AS17-146-22370.png", 4104, 4.0),
    ("as17-137-20963", "as17-137-20963.png", 4103, 4.0),
    ("as17-137-20981", "as17-137-20981.png", 4091, 4.0),
    ("as17-137-20984", "as17-137-20984.png", 4104, 4.0),
    ("as17-138-21179", "as17-138-21179.png", 4032, 4.0),
    ("as17-142-21706", "as17-142-21706.png", 4033, 4.0),
]


# --- Hasselblad reseau-plate fiducial line removal (same technique as inpaint_reseau.py) ---


def detect_lines(gray, thresh_std):
    col_med, row_med = np.median(gray, axis=0), np.median(gray, axis=1)
    col_resid = col_med - uniform_filter1d(col_med, size=25, mode="nearest")
    row_resid = row_med - uniform_filter1d(row_med, size=25, mode="nearest")
    cols = np.where(np.abs(col_resid) > col_resid.std() * thresh_std)[0]
    rows = np.where(np.abs(row_resid) > row_resid.std() * thresh_std)[0]
    return cols, rows


def _group(indices, margin, limit):
    if len(indices) == 0:
        return []
    bands = []
    lo = hi = indices[0]
    for i in indices[1:]:
        if i - hi <= margin * 2:
            hi = i
        else:
            bands.append((lo, hi))
            lo = hi = i
    bands.append((lo, hi))
    return [(max(lo - margin, 0), min(hi + margin, limit - 1)) for lo, hi in bands]


def shift_fill_columns(arr, cols, margin=5):
    w = arr.shape[1]
    out = arr.copy()
    for c0, c1 in _group(cols, margin, w):
        width = c1 - c0 + 1
        src0, src1 = c0 - width, c0
        if src0 < 0:
            src0, src1 = c1 + 1, c1 + 1 + width
        out[:, c0 : c1 + 1] = arr[:, src0:src1]
    return out


def shift_fill_rows(arr, rows, margin=5):
    h = arr.shape[0]
    out = arr.copy()
    for r0, r1 in _group(rows, margin, h):
        height = r1 - r0 + 1
        src0, src1 = r0 - height, r0
        if src0 < 0:
            src0, src1 = r1 + 1, r1 + 1 + height
        out[r0 : r1 + 1, :] = arr[src0:src1, :]
    return out


def process_tile(path, full_frame_px, reseau_thresh):
    """Load a committed source crop, remove reseau lines (if any), flat-field, desaturate, and
    resample to SOURCE_TILE_PX representing PHYS_TILE_M metres. Returns a [0,1] luminance array.
    """
    im = Image.open(path).convert("RGB")
    native_phys_m = im.size[0] / full_frame_px * FULL_FRAME_M

    arr = np.asarray(im).astype(np.float32)
    if reseau_thresh is not None:
        gray = np.asarray(im.convert("L")).astype(np.float32)
        cols, rows = detect_lines(gray, reseau_thresh)
        arr = shift_fill_columns(arr, cols)
        arr = shift_fill_rows(arr, rows)
    im = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))

    arr = np.asarray(im).astype(np.float32) / 255.0
    luminance = arr @ np.array([0.2126, 0.7152, 0.0722])
    side = im.size[0]
    illum = (
        np.asarray(
            Image.fromarray((luminance * 255).astype(np.uint8)).filter(
                ImageFilter.GaussianBlur(side // 3)
            )
        ).astype(np.float32)
        / 255.0
    )
    luminance = luminance / np.clip(illum, 0.15, None)
    luminance = luminance * (0.49 / luminance.mean())
    luminance = np.clip(luminance, 0.0, 1.0)

    upscale = PHYS_TILE_M / native_phys_m
    lum_img = Image.fromarray((luminance * 255).astype(np.uint8)).resize(
        (SOURCE_TILE_PX, SOURCE_TILE_PX), Image.LANCZOS
    )
    return np.asarray(lum_img).astype(np.float32) / 255.0, native_phys_m, upscale


def equalize_contrast(tiles_lum, names, reference_name):
    """Match every tile's contrast (std around the shared 0.49 mean) to the reference tile's own -
    process_tile already matches every tile's MEAN, but leaves each photo's own dynamic range
    (std) untouched, and that varies a lot frame to frame (0.026-0.193 in this set). Placed
    side-by-side, that reads as a light/dark patchwork at the frame pitch even with no repeated
    frame. Rescaling std around the mean keeps each tile's own fine grain/shape, only changing its
    amplitude, so real detail survives - this is a low-frequency (whole-frame exposure) match, not
    a detail-destroying blur."""
    ref_std = tiles_lum[names.index(reference_name)].std()
    out = []
    for lum in tiles_lum:
        std = lum.std()
        scale = ref_std / std if std > 1e-6 else 1.0
        eq = np.clip(0.49 + (lum - 0.49) * scale, 0.0, 1.0)
        eq = np.clip(eq * (0.49 / eq.mean()), 0.0, 1.0)
        out.append(eq)
    return out


def dihedral(arr, k):
    """8 square-preserving transforms: k=0..3 are rotations, k=4..7 are mirrored rotations."""
    if k >= 4:
        arr = np.fliplr(arr)
        k -= 4
    return np.rot90(arr, k)


def feather_mask(size, edge_px):
    ramp = np.ones(size, dtype=np.float32)
    r = np.linspace(0, 1, edge_px)
    ramp[:edge_px] = r
    ramp[-edge_px:] = r[::-1]
    return ramp


# --- Craterlet/rock stamping, local-patch (non-toroidal - this map is not tiled by MuJoCo) ---


def _poisson_scatter_plane(rng, size, n, min_sep, max_tries=40):
    pts = []
    for _ in range(n):
        for _ in range(max_tries):
            p = rng.uniform(0, size, 2)
            if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 >= min_sep**2 for q in pts):
                pts.append(p)
                break
    return pts


def _local_patch(lum, cx, cy, outer_r):
    size = lum.shape[0]
    x0, x1 = max(0, int(cx - outer_r)), min(size, int(cx + outer_r) + 1)
    y0, y1 = max(0, int(cy - outer_r)), min(size, int(cy + outer_r) + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    xx, yy = np.meshgrid(np.arange(x0, x1) - cx, np.arange(y0, y1) - cy, indexing="ij")
    return x0, x1, y0, y1, xx, yy


def stamp_crater(lum, cx, cy, r):
    """Soft depression with a raised rim, rim lit toward the sun, dark crescent away."""
    patch_bounds = _local_patch(lum, cx, cy, r * 1.6)
    if patch_bounds is None:
        return
    x0, x1, y0, y1, dx, dy = patch_bounds
    dist = np.sqrt(dx * dx + dy * dy)
    within = dist < r * 1.6
    if not within.any():
        return
    d = dist[within]
    ux, uy = dx[within] / np.maximum(d, 1e-6), dy[within] / np.maximum(d, 1e-6)
    sun_align = ux * SUN_DIR_PX[0] + uy * SUN_DIR_PX[1]

    bowl = -0.07 * np.exp(-((d / (r * 0.8)) ** 2))
    rim_profile = np.exp(-(((d - r) / (r * 0.35)) ** 2))
    rim = rim_profile * (0.035 + 0.05 * np.clip(sun_align, 0, 1))
    shadow_len = r * 0.35
    shadow = np.exp(
        -(((d - r - shadow_len * np.clip(-sun_align, 0, 1)) / (r * 0.35)) ** 2)
    )
    shadow = shadow * -0.045 * np.clip(-sun_align, 0, 1)

    patch = lum[x0:x1, y0:y1]
    patch[within] = np.clip(patch[within] + bowl + rim + shadow, 0.05, 1.0)


def stamp_rock(lum, cx, cy, r):
    """Small embedded rock/clod: brighter sunlit cap, thin dark shadow crescent."""
    patch_bounds = _local_patch(lum, cx, cy, r * 1.4)
    if patch_bounds is None:
        return
    x0, x1, y0, y1, dx, dy = patch_bounds
    dist = np.sqrt(dx * dx + dy * dy)
    within = dist < r * 1.4
    if not within.any():
        return
    d = dist[within]
    ux, uy = dx[within] / np.maximum(d, 1e-6), dy[within] / np.maximum(d, 1e-6)
    sun_align = ux * SUN_DIR_PX[0] + uy * SUN_DIR_PX[1]

    body = np.exp(-((d / r) ** 2))
    lit = body * (0.03 + 0.07 * np.clip(sun_align, 0, 1))
    shadow_len = r * 0.4
    shadow = np.exp(
        -(((d - r - shadow_len * np.clip(-sun_align, 0, 1)) / (r * 0.35)) ** 2)
    )
    shadow = shadow * -0.04 * np.clip(-sun_align, 0, 1)

    patch = lum[x0:x1, y0:y1]
    patch[within] = np.clip(patch[within] + lit + shadow, 0.05, 1.0)


def build_mosaic(plane_m: float, px_per_m: float, seed: int):
    rng = np.random.default_rng(seed)

    tiles_lum, provenance = [], []
    for name, filename, ffpx, thresh in TILES:
        lum, native_m, upscale = process_tile(SOURCES / filename, ffpx, thresh)
        tiles_lum.append(lum)
        provenance.append((name, native_m, upscale))
    tile_names = [name for name, _, _, _ in TILES]
    tiles_lum = equalize_contrast(tiles_lum, tile_names, REFERENCE_TILE_NAME)

    tile_px = max(8, round(PHYS_TILE_M * px_per_m))
    seam_px = max(2, round(tile_px * SEAM_FRAC))
    step_px = max(1, round(tile_px * GRID_DENSITY))
    target_canvas_px = round(plane_m * px_per_m)
    n_cells = max(1, round((target_canvas_px - tile_px) / step_px) + 1)
    canvas_px = step_px * (n_cells - 1) + tile_px
    actual_px_per_m = canvas_px / plane_m

    # Precompute every (source tile x dihedral transform) variant once at placement resolution,
    # rather than per grid cell - 9x8=72 variants total, reused across every cell that draws them.
    small = [
        np.asarray(
            Image.fromarray((lum * 255).astype(np.uint8)).resize(
                (tile_px, tile_px), Image.LANCZOS
            )
        ).astype(np.float32)
        / 255.0
        for lum in tiles_lum
    ]
    variants = [[dihedral(s, k) for k in range(8)] for s in small]

    edge = feather_mask(tile_px, seam_px)
    tile_w2d = np.outer(edge, edge)

    # An edge cell whose jitter happens to push it INWARD (away from the plane border) would leave
    # a sliver of the border uncovered if the grid stopped exactly at the crop boundary - so place
    # an extra ring of cells beyond the boundary on every side (their footprint still reaches
    # across it, guaranteed by margin >= tile_px) and crop them off afterwards, same jitter
    # freedom as every interior cell.
    margin = tile_px
    ring = int(np.ceil(margin / step_px)) + 1
    grid_lo, grid_hi = -ring, n_cells + ring  # cell index range, exclusive hi
    padded_px = canvas_px + 2 * margin
    canvas = np.zeros((padded_px, padded_px), dtype=np.float32)
    weight = np.zeros((padded_px, padded_px), dtype=np.float32)
    grid_tile_idx = {}

    for r in range(grid_lo, grid_hi):
        for c in range(grid_lo, grid_hi):
            avoid = {
                grid_tile_idx.get((r - 1, c), -1),
                grid_tile_idx.get((r, c - 1), -1),
            }
            tile_idx = int(rng.integers(0, len(TILES)))
            tries = 0
            while tile_idx in avoid and tries < 5:
                tile_idx = int(rng.integers(0, len(TILES)))
                tries += 1
            grid_tile_idx[(r, c)] = tile_idx
            k = int(rng.integers(0, 8))
            jitter_r = rng.uniform(JITTER_FRAC_MIN, JITTER_FRAC_MAX) * tile_px
            jitter_r *= rng.choice([-1.0, 1.0])
            jitter_c = rng.uniform(JITTER_FRAC_MIN, JITTER_FRAC_MAX) * tile_px
            jitter_c *= rng.choice([-1.0, 1.0])
            r0 = margin + round(r * step_px + jitter_r)
            c0 = margin + round(c * step_px + jitter_c)
            if r0 < 0 or c0 < 0 or r0 + tile_px > padded_px or c0 + tile_px > padded_px:
                continue  # this ring cell's footprint doesn't reach the padded canvas at all
            canvas[r0 : r0 + tile_px, c0 : c0 + tile_px] += (
                variants[tile_idx][k] * tile_w2d
            )
            weight[r0 : r0 + tile_px, c0 : c0 + tile_px] += tile_w2d

    canvas = canvas[margin : margin + canvas_px, margin : margin + canvas_px]
    weight = weight[margin : margin + canvas_px, margin : margin + canvas_px]
    min_coverage = float(weight.min())
    if min_coverage <= 1e-6:
        raise RuntimeError(
            f"gap in colormap coverage (min weight={min_coverage:.4f}) - increase GRID_DENSITY "
            "or shrink JITTER_FRAC_MAX"
        )
    mosaic = canvas / np.clip(weight, 1e-6, None)

    illum = (
        np.asarray(
            Image.fromarray((mosaic * 255).astype(np.uint8)).filter(
                ImageFilter.GaussianBlur(canvas_px // 6)
            )
        ).astype(np.float32)
        / 255.0
    )
    mosaic = mosaic / np.clip(illum, 0.15, None)
    mosaic = np.clip(mosaic * (0.49 / mosaic.mean()), 0.0, 1.0)

    area_m2 = plane_m * plane_m
    n_craters = max(2, round(area_m2 * 1.8 * 6 / 20))
    n_rocks = max(4, round(area_m2 * 1.8 * 14 / 20))
    crater_rng = np.random.default_rng(seed + 1)
    for cx, cy in _poisson_scatter_plane(
        crater_rng, canvas_px, n_craters, CRATER_MIN_SEP_M * actual_px_per_m
    ):
        r_m = crater_rng.uniform(0.10, 0.45)
        stamp_crater(mosaic, cx, cy, r_m * actual_px_per_m)
    for cx, cy in _poisson_scatter_plane(
        crater_rng, canvas_px, n_rocks, ROCK_MIN_SEP_M * actual_px_per_m
    ):
        r_m = crater_rng.uniform(0.03, 0.12)
        stamp_rock(mosaic, cx, cy, r_m * actual_px_per_m)

    tint = np.array([1.03, 1.0, 0.94])
    rgb = np.clip(np.stack([mosaic] * 3, axis=-1) * tint, 0.0, 1.0)
    final = (rgb * 255).astype(np.uint8)

    info = {
        "canvas_px": canvas_px,
        "plane_m": plane_m,
        "px_per_m": actual_px_per_m,
        "n_cells": n_cells,
        "tile_px": tile_px,
        "n_craters": n_craters,
        "n_rocks": n_rocks,
        "seed": seed,
        "provenance": provenance,
        "min_coverage": min_coverage,
    }
    return final, info


def low_freq_std(final_rgb, side=256):
    """Downsample to ~side px and report the std of that low-res image - a low-frequency
    brightness lattice from mismatched per-placement exposure/contrast (the v1 bug) shows up as an
    elevated std even at this heavily-blurred scale; a properly equalized map should look
    essentially flat here aside from craterlet/rock shading."""
    im = Image.fromarray(final_rgb).convert("L").resize((side, side), Image.LANCZOS)
    return float(np.asarray(im).astype(np.float32).std())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--plane-m", type=float, default=20.0, help="ground plane side length, metres"
    )
    parser.add_argument(
        "--px-per-m", type=float, default=410.0, help="output resolution, px/metre"
    )
    parser.add_argument("--seed", type=int, default=869)
    parser.add_argument("--out", default="lunar_regolith_untiled.png")
    args = parser.parse_args()

    final, info = build_mosaic(args.plane_m, args.px_per_m, args.seed)
    out_path = ASSETS / args.out
    Image.fromarray(final).save(out_path)

    print(
        f"saved {out_path} {final.shape[1]}x{final.shape[0]}px "
        f"({info['plane_m']:.2f}m x {info['plane_m']:.2f}m plane, {info['px_per_m']:.1f} px/m, "
        f"{1000 / info['px_per_m']:.2f} mm/px)"
    )
    print(
        f"grid: {info['n_cells']}x{info['n_cells']} cells of {len(TILES)} source frames "
        f"x 8 dihedral transforms, seed={info['seed']}"
    )
    print(f"craterlets: {info['n_craters']}, rocks: {info['n_rocks']}")
    print(f"min placement coverage weight: {info['min_coverage']:.4f} (no gaps if > 0)")
    lf_std = low_freq_std(final)
    print(
        f"256px-downsampled std: {lf_std:.2f} (low = featureless aside from craters/rocks)"
    )

    with open(ASSETS / "ground_colormap_provenance.txt", "w") as f:
        f.write(
            f"canvas {info['canvas_px']}x{info['canvas_px']}px, plane {info['plane_m']:.2f}m x "
            f"{info['plane_m']:.2f}m, {info['px_per_m']:.1f} px/m ({1000 / info['px_per_m']:.2f} mm/px)\n"
        )
        f.write(
            f"{info['n_cells']}x{info['n_cells']} cells, {len(TILES)} distinct source frames, "
            f"8 dihedral transforms each, PHYS_TILE_M={PHYS_TILE_M}, seed={info['seed']}, "
            f"contrast-equalized to {REFERENCE_TILE_NAME}, jittered "
            f"{JITTER_FRAC_MIN}-{JITTER_FRAC_MAX} tile-widths off grid (v2)\n"
        )
        for name, native_m, upscale in info["provenance"]:
            f.write(
                f"{name}: native crop ~{native_m:.2f}m, placed at {PHYS_TILE_M}m (upscale {upscale:.2f}x)\n"
            )
        f.write(f"craterlets: {info['n_craters']}, rocks: {info['n_rocks']}\n")
        f.write(
            "Source frames: NASA Apollo 15/17 Hasselblad surface photos, public domain NASA/JSC, "
            "archive.org (collection johnsonspacecentermediaarchive) - cropped to a square clear of "
            "the gnomon and any hardware (bootprints/rover tracks left in); committed under "
            "assets/ground_colormap_sources/. Excluded as near-duplicate content (same sample/"
            "station documented by a second magazine/camera as an already-selected frame): "
            "AS15-87-11769, as17-138-21049, as17-142-21699. Rejected as boulder/rock-dominated or "
            "for people/hardware in frame: AS15-86-11659, as17-134-20403, as17-137-20901, "
            "as17-137-20913, as17-140-21411, as17-146-22336, AS15-87-11778, as17-137-20972, "
            "as17-138-21030, as16-117-18825, as16-117-18826, as17-145-22157, AS12-48-7107/7128.\n"
        )


if __name__ == "__main__":
    main()
