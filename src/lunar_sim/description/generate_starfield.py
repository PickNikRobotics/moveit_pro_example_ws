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


"""Generates lunar_sim's star-field skybox (assets/lunar_starfield.png).

Offline and seeded; run manually and commit the output.

Replaces a gradient skybox, which could not draw stars at all. Stars are sampled on the sphere and
then projected onto a cube map, which is what keeps them continuous across the seams instead of six
unrelated fields that repeat as the camera pans.

Brightness is deliberately not photometric: a camera exposed for sunlit regolith would show no
stars, which is why Apollo surface photos have none.

Usage: python3 generate_starfield.py [--face-px 512] [--stars 6500] [--seed 869]

gridsize/gridlayout in husky_scene.xml must match what this prints.
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"

# MuJoCo's cube-map atlas layout (see the <texture> gridsize/gridlayout entry in the MuJoCo XML
# reference). A 3x4 grid holding the standard unwrapped cross; '.' cells are unused and stay black.
#   row 0:  .  U  .  .
#   row 1:  L  F  R  B
#   row 2:  .  D  .  .
ATLAS_ROWS = 3
ATLAS_COLS = 4
GRID_LAYOUT = ".U..LFRB.D.."

# Per-face view basis in world axes, as (forward, right, up) for each cube face. forward is the
# outward face normal; a point on the face is forward + u*right + v*up for u,v in [-1, 1].
# Orientation only has to be self-consistent to keep the field continuous across seams - an
# isotropic star field has no preferred orientation, so these follow the conventional OpenGL
# cube-map handedness rather than anything MuJoCo-specific.
FACE_BASES = {
    "R": ((1, 0, 0), (0, -1, 0), (0, 0, 1)),
    "L": ((-1, 0, 0), (0, 1, 0), (0, 0, 1)),
    "F": ((0, 1, 0), (1, 0, 0), (0, 0, 1)),
    "B": ((0, -1, 0), (-1, 0, 0), (0, 0, 1)),
    "U": ((0, 0, 1), (1, 0, 0), (0, -1, 0)),
    "D": ((0, 0, -1), (1, 0, 0), (0, 1, 0)),
}

# Star count per magnitude bin grows ~10**0.6 per magnitude, which gives the "few bright, many
# faint" balance that reads as a sky. MAG_MAX stops short of the naked-eye limit on purpose: a
# magnitude-6 star is sub-pixel, and sub-pixel points are the first thing video compression
# discards. Nothing here is photometric - a camera exposed for sunlit regolith would show no stars
# at all, which is why Apollo surface photos have none.
MAG_MIN = -1.0
MAG_MAX = 5.0
MAG_SLOPE = 0.6

# Above 1.0 so the brightest stars saturate their centre pixel and the faint tail clears the
# codec's noise floor. At a photometric 2.6 the brightest peaked at 188/255 and the median near
# 20/255 - fine in a PNG, invisible on a projector.
PEAK_BRIGHTNESS = 5.5

# Bright stars spread to imitate sensor bloom, not atmospheric scatter (there is none). The faint
# end stays near a pixel rather than below one: a star whose energy fits in one pixel reads as
# compression noise and gets discarded as such.
SIGMA_BRIGHT = 1.5
SIGMA_FAINT = 0.85

# Blackbody-ish colour ramp, hot (blue-white) to cool (orange), as linear RGB multipliers
# normalised so none of them dims a star below its magnitude-implied brightness. Sampled with a
# bias toward the middle of the ramp so the field is mostly white with occasional tinted stars,
# the way a real field looks to a camera without long exposure.
COLOR_RAMP = np.array(
    [
        (0.72, 0.80, 1.00),  # hot blue-white
        (0.88, 0.92, 1.00),
        (1.00, 1.00, 1.00),  # white
        (1.00, 0.96, 0.88),
        (1.00, 0.86, 0.70),  # cool orange
    ]
)


def sample_stars(rng, count):
    """Sample `count` stars uniformly on the unit sphere with magnitudes and colours.

    Returns (directions, brightness, colors): unit vectors (count, 3), linear peak brightness
    (count,), and linear RGB tints (count, 3).
    """
    # Uniform on the sphere needs z uniform in [-1, 1], not a uniform polar angle - sampling
    # theta uniformly instead would pile stars up at the poles.
    z = rng.uniform(-1.0, 1.0, count)
    azimuth = rng.uniform(0.0, 2.0 * np.pi, count)
    radial = np.sqrt(np.clip(1.0 - z * z, 0.0, None))
    directions = np.column_stack(
        (radial * np.cos(azimuth), radial * np.sin(azimuth), z)
    )

    # Inverse-transform sample the magnitude power law: the cumulative count up to magnitude m
    # goes as 10**(MAG_SLOPE * m), so drawing u uniform and inverting gives the right mix without
    # rejection sampling.
    lo = 10.0 ** (MAG_SLOPE * MAG_MIN)
    hi = 10.0 ** (MAG_SLOPE * MAG_MAX)
    magnitudes = np.log10(rng.uniform(lo, hi, count)) / MAG_SLOPE

    # Magnitude is a log scale: 5 magnitudes is a factor of 100 in flux. Anchor the brightest at
    # PEAK_BRIGHTNESS and let the rest fall off physically from there.
    brightness = PEAK_BRIGHTNESS * 10.0 ** (-0.4 * (magnitudes - MAG_MIN))

    # Triangular colour index keeps most stars near the white middle of the ramp.
    ramp_pos = rng.triangular(0.0, 0.5, 1.0, count) * (len(COLOR_RAMP) - 1)
    lower = np.floor(ramp_pos).astype(int)
    upper = np.clip(lower + 1, None, len(COLOR_RAMP) - 1)
    frac = (ramp_pos - lower)[:, None]
    colors = COLOR_RAMP[lower] * (1.0 - frac) + COLOR_RAMP[upper] * frac

    return directions, brightness, magnitudes, colors


def project_to_face(directions, basis):
    """Project directions onto one cube face.

    Returns (mask, u, v): which directions land on this face, and their face coordinates in
    [-1, 1]. A direction belongs to the face whose outward normal it is most aligned with, so the
    six masks partition the sphere with no star drawn twice and none dropped.
    """
    forward, right, up = (np.asarray(v, dtype=float) for v in basis)
    depth = directions @ forward
    # Most-aligned axis means the forward component dominates both tangential ones.
    tangent_u = directions @ right
    tangent_v = directions @ up
    mask = (depth > 0) & (np.abs(tangent_u) <= depth) & (np.abs(tangent_v) <= depth)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(mask, tangent_u / depth, 0.0)
        v = np.where(mask, tangent_v / depth, 0.0)
    return mask, u, v


def splat_face(face_px, u, v, brightness, magnitudes, colors):
    """Render one face by accumulating a Gaussian splat per star. Returns linear RGB (h, w, 3)."""
    image = np.zeros((face_px, face_px, 3), dtype=np.float64)
    if len(u) == 0:
        return image

    # Face coords [-1, 1] -> pixel centres. A cube face is a gnomonic projection, so pixel pitch
    # is not constant in angle across the face; at 90 deg per face the variation is small enough
    # that splatting in pixel space (rather than resampling per-star) leaves no visible
    # distortion, and it keeps every star exactly one splat.
    px = (u + 1.0) * 0.5 * (face_px - 1)
    py = (1.0 - (v + 1.0) * 0.5) * (face_px - 1)

    # Sigma per star, interpolated over the magnitude range.
    span = MAG_MAX - MAG_MIN
    t = np.clip((magnitudes - MAG_MIN) / span, 0.0, 1.0)
    sigmas = SIGMA_BRIGHT * (1.0 - t) + SIGMA_FAINT * t

    # Splat radius: 3 sigma captures essentially all of the Gaussian's energy, and clamping to at
    # least 1 px guarantees even the faintest star writes a pixel instead of rounding away.
    radii = np.maximum(1, np.ceil(3.0 * sigmas).astype(int))

    for x0, y0, sigma, radius, peak, color in zip(
        px, py, sigmas, radii, brightness, colors
    ):
        x_lo, x_hi = int(np.floor(x0)) - radius, int(np.floor(x0)) + radius + 1
        y_lo, y_hi = int(np.floor(y0)) - radius, int(np.floor(y0)) + radius + 1
        # Stars near a face edge have their splat clipped rather than wrapped. The neighbouring
        # face draws its own half from the same sphere sample, so the star stays continuous across
        # the seam; wrapping here would instead duplicate it onto the wrong face.
        x_lo_c, x_hi_c = max(0, x_lo), min(face_px, x_hi)
        y_lo_c, y_hi_c = max(0, y_lo), min(face_px, y_hi)
        if x_lo_c >= x_hi_c or y_lo_c >= y_hi_c:
            continue
        xs = np.arange(x_lo_c, x_hi_c) - x0
        ys = np.arange(y_lo_c, y_hi_c) - y0
        kernel = np.exp(-(ys[:, None] ** 2 + xs[None, :] ** 2) / (2.0 * sigma * sigma))
        image[y_lo_c:y_hi_c, x_lo_c:x_hi_c] += (
            peak * kernel[:, :, None] * color[None, None, :]
        )

    return image


def build_atlas(face_px, stars, seed):
    """Render all six faces and assemble them into the MuJoCo cube-map atlas."""
    rng = np.random.default_rng(seed)
    directions, brightness, magnitudes, colors = sample_stars(rng, stars)

    atlas = np.zeros((ATLAS_ROWS * face_px, ATLAS_COLS * face_px, 3), dtype=np.float64)
    drawn = 0
    for cell, face in enumerate(GRID_LAYOUT):
        if face == ".":
            continue
        mask, u, v = project_to_face(directions, FACE_BASES[face])
        face_image = splat_face(
            face_px, u[mask], v[mask], brightness[mask], magnitudes[mask], colors[mask]
        )
        drawn += int(mask.sum())
        row, col = divmod(cell, ATLAS_COLS)
        atlas[
            row * face_px : (row + 1) * face_px, col * face_px : (col + 1) * face_px
        ] = face_image

    # Every sampled star must land on exactly one face. A mismatch means FACE_BASES no longer
    # partitions the sphere, which would show up as gaps or doubled stars rather than as an error.
    if drawn != stars:
        raise RuntimeError(
            f"cube faces did not partition the sphere: drew {drawn} of {stars} stars"
        )

    return np.clip(atlas, 0.0, 1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--face-px", type=int, default=1024, help="pixels per cube face"
    )
    parser.add_argument(
        "--stars", type=int, default=6500, help="stars over the whole sphere"
    )
    parser.add_argument("--seed", type=int, default=869)
    parser.add_argument(
        "--out",
        type=Path,
        default=ASSETS / "lunar_starfield.png",
        help="output atlas path",
    )
    args = parser.parse_args()

    atlas = build_atlas(args.face_px, args.stars, args.seed)

    # sRGB encode: the splat accumulates in linear light so overlapping stars add correctly, but
    # PNG (and MuJoCo's texture read) expect gamma-encoded values. Skipping this crushes every
    # faint star toward black.
    encoded = np.where(
        atlas <= 0.0031308,
        atlas * 12.92,
        1.055 * np.power(np.clip(atlas, 0.0031308, None), 1.0 / 2.4) - 0.055,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray((encoded * 255.0 + 0.5).astype(np.uint8), mode="RGB").save(args.out)

    lit = int((atlas.max(axis=2) > 1.0 / 255.0).sum())
    print(f"wrote {args.out} ({ATLAS_COLS * args.face_px}x{ATLAS_ROWS * args.face_px})")
    print(f"  {args.stars} stars, seed {args.seed}, {lit} lit pixels")
    print(
        f'  husky_scene.xml: gridsize="{ATLAS_ROWS} {ATLAS_COLS}" gridlayout="{GRID_LAYOUT}"'
    )


if __name__ == "__main__":
    main()
