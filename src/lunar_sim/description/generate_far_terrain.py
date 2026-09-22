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


"""Generates lunar_sim's far-field horizon relief (assets/lunar_far_hfield.png).

Offline and seeded; run manually and commit the output.

Visual only, so the driven terrain keeps the 10 m half-extents its spawn elevation, rock keepout,
square calibration and shadow frustum are all derived from.

It has to MEET the driven terrain's rim, not just sit beyond it: from the camera's ~0.95 m height
that rim already is the horizon, so ridges with void behind it render as black sky below the
horizon line.

Usage: python3 generate_far_terrain.py [--half-m 300] [--grid 256] [--seed 869]

Copy the printed size/pos into husky_scene.xml.
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"

# The driven surface this field must meet, from husky_scene.xml's <hfield>/<geom>. Keep in sync:
# a mismatch shows up as a step at the rim, or this field poking through the driven ground.
NEAR_HALF_M = 10.0
NEAR_Z_MIN = -0.2095
NEAR_Z_MAX = NEAR_Z_MIN + 0.286

# Keyed on Chebyshev distance max(|x|, |y|), NOT radius: the driven terrain is a square 20x20 m
# hfield whose corners reach 14.1 m, so a radial profile would lift this field at 10 m in every
# direction and push it up through those corners. Hidden below HIDE_D, ramped up to meet the rim at
# MEET_D - the ramp must finish there, since past it there is no driven ground left to hide under
# and any shortfall renders as a trench ringing the terrain.
HIDE_D = 6.0
MEET_D = NEAR_HALF_M

# Held below the driven ground so it stays hidden, but not so far that the ramp reads as a wall.
HIDDEN_Z = NEAR_Z_MIN - 0.35

# Meet the rim at the middle of the driven terrain's range, so the join is not a visible step.
MEET_Z = 0.5 * (NEAR_Z_MIN + NEAR_Z_MAX)

# Rolling relief, ramped in beyond the join so the ground only heaves once it reads as distance.
RELIEF_AMP_M = 4.0
RELIEF_SIGMA_CELLS = 7.0
RELIEF_RAMP_M = 60.0

# Ridge band giving the horizon its silhouette. RIDGE_AMP_M is set by how much sky the ridges may
# take, not by how dramatic they can be: the camera's half-FOV is ~25 deg, and at 42 m the skyline
# reached 12 deg above eye level and closed off the sky the stars sit in. main() prints that angle -
# treat it, not the metre figure, as the number being tuned.
RIDGE_R_FRAC = 0.55
RIDGE_W_FRAC = 0.30
RIDGE_AMP_M = 18.0
RIDGE_SIGMA_CELLS = 4.0

# (cycles around the horizon, weight). A few low harmonics break the ridge into separate massifs.
RIDGE_HARMONICS = ((2, 1.00), (3, 0.62), (5, 0.41), (8, 0.24), (13, 0.14))


def smoothstep(edge0, edge1, x):
    """Hermite smoothstep, clamped outside [edge0, edge1]."""
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def build_far_field(half_m, grid, seed, ridge_amp=RIDGE_AMP_M, relief_amp=RELIEF_AMP_M):
    """Build the far-field elevation grid in metres. Returns height_m indexed [row, col]."""
    rng = np.random.default_rng(seed)

    axis = np.linspace(-half_m, half_m, grid)
    x = axis[None, :]
    y = axis[:, None]
    radius = np.hypot(x, y)
    chebyshev = np.maximum(np.abs(x), np.abs(y))
    azimuth = np.arctan2(y, x)

    # Base profile: hidden under the driven ground, then up to meet its rim. Keyed on Chebyshev
    # distance to follow the driven terrain's square footprint - see HIDE_D/MEET_D above.
    height = HIDDEN_Z + (MEET_Z - HIDDEN_Z) * smoothstep(HIDE_D, MEET_D, chebyshev)

    # Rolling relief from smoothed white noise. Smoothing a noise field (rather than summing
    # octaves) is the same approach generate_terrain.py takes for its low-frequency base, and at
    # this cell size one scale is all the silhouette can resolve.
    relief = gaussian_filter(
        rng.standard_normal((grid, grid)), RELIEF_SIGMA_CELLS, mode="nearest"
    )
    peak = np.abs(relief).max()
    if peak > 0:
        relief /= peak
    height += (
        relief_amp * relief * smoothstep(MEET_D, MEET_D + RELIEF_RAMP_M, chebyshev)
    )

    # Ridge band, modulated around the horizon so it forms separate massifs.
    modulation = np.zeros_like(azimuth)
    weight_total = 0.0
    for cycles, weight in RIDGE_HARMONICS:
        modulation += weight * np.cos(cycles * azimuth + rng.uniform(0.0, 2.0 * np.pi))
        weight_total += weight
    modulation /= weight_total

    ridge_r = RIDGE_R_FRAC * half_m
    ridge_w = RIDGE_W_FRAC * half_m
    # Radial falloff of the band: a raised cosine over +-ridge_w, zero outside, so the ridges rise
    # and fall rather than ending abruptly at the field's edge.
    band = np.where(
        np.abs(radius - ridge_r) < ridge_w,
        0.5 * (1.0 + np.cos(np.pi * (radius - ridge_r) / ridge_w)),
        0.0,
    )
    # Modulation is mapped to [0, 1] rather than used signed: a signed one would cut troughs below
    # the surrounding relief and open gaps in the skyline.
    ridges = ridge_amp * band * (0.35 + 0.65 * 0.5 * (1.0 + modulation))
    height += gaussian_filter(ridges, RIDGE_SIGMA_CELLS, mode="nearest")

    # Nothing inside the driven terrain's footprint may rise above HIDDEN_Z, whatever the noise
    # did. The relief and ridge terms are already ramped to zero there, so this only guards
    # against a future parameter change reopening the problem - and it is keyed on the same
    # Chebyshev footprint, so it covers the corners a radial test would miss.
    height = np.where(chebyshev < NEAR_HALF_M, np.minimum(height, HIDDEN_Z), height)

    return height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--half-m", type=float, default=300.0, help="field half-extent in metres"
    )
    parser.add_argument(
        "--grid", type=int, default=256, help="heightmap cells per side"
    )
    parser.add_argument("--seed", type=int, default=869)
    parser.add_argument(
        "--ridge-amp",
        type=float,
        default=RIDGE_AMP_M,
        help="ridge band height in metres",
    )
    parser.add_argument(
        "--relief-amp",
        type=float,
        default=RELIEF_AMP_M,
        help="rolling relief amplitude in metres",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=ASSETS / "lunar_far_hfield.png",
        help="output heightmap path",
    )
    args = parser.parse_args()

    height_m = build_far_field(
        args.half_m, args.grid, args.seed, args.ridge_amp, args.relief_amp
    )
    z_min = float(height_m.min())
    z_max = float(height_m.max())
    elevation_z = z_max - z_min

    # MuJoCo maps PNG [0, 1] onto [pos.z, pos.z + elevation_z], so setting the geom's pos.z to
    # z_min makes world elevation reconstruct height_m exactly - the same convention
    # generate_terrain.py uses for the driven ground.
    values01 = (height_m - z_min) / elevation_z

    # MuJoCo reads an hfield PNG with row 0 at the top, while this grid is built with row 0 at
    # y = -half_m. Flipping here keeps the field's orientation matching the world axes it was
    # computed in - it is radially symmetric enough not to show, but the azimuthal ridge
    # modulation is not, so an unflipped write would mirror the skyline north/south.
    # 16-bit unsigned at full scale: world_z = geom.pos.z + (raw / 65535) * elevation_z. PIL reads
    # this back as mode "I" (int32 container), so consumers must divide by 65535, not by the numpy
    # dtype max.
    png = np.flipud((values01 * 65535.0 + 0.5).astype(np.uint16))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(png, mode="I;16").save(args.out)

    # Diagnostics over the join line and the hidden footprint, both keyed on Chebyshev distance
    # to match the profile above. The hidden figure is the one that matters most: any value in it
    # above NEAR_Z_MIN is this field poking up through the driven ground.
    axis = np.linspace(-args.half_m, args.half_m, args.grid)
    chebyshev = np.maximum(np.abs(axis[None, :]), np.abs(axis[:, None]))
    cell_m = 2.0 * args.half_m / args.grid
    rim = height_m[np.abs(chebyshev - NEAR_HALF_M) < cell_m]
    hidden = height_m[chebyshev < NEAR_HALF_M - cell_m]

    print(
        f"wrote {args.out} ({args.grid}x{args.grid}, {2 * args.half_m / args.grid:.2f} m/cell)"
    )
    print(
        f"  elevation range {z_min:.4f} .. {z_max:.4f} m (elevation_z={elevation_z:.4f})"
    )
    if rim.size:
        print(
            f"  at the rim: {rim.min():.3f} .. {rim.max():.3f} m "
            f"(driven ground spans {NEAR_Z_MIN:.3f} .. {NEAR_Z_MAX:.3f})"
        )
    if hidden.size:
        print(
            f"  under the driven ground: max {hidden.max():.3f} m "
            f"(must stay below {NEAR_Z_MIN:.3f})"
        )
        if hidden.max() > NEAR_Z_MIN:
            raise SystemExit(
                f"far field rises to {hidden.max():.3f} m inside the driven terrain's footprint, "
                f"above its {NEAR_Z_MIN:.3f} m floor - it would poke through the driven ground"
            )
    print(f"  triangles rendered per camera pass: {2 * (args.grid - 1) ** 2}")
    # The number that actually decides the look: how far up the frame the skyline reaches from
    # the OAK-D's mount height. Too tall and the ridges swallow the sky the stars sit in.
    eye_m = 0.95
    ridge_r = RIDGE_R_FRAC * args.half_m
    print(
        f"  skyline subtends ~{np.degrees(np.arctan2(z_max - eye_m, ridge_r)):.1f} deg above eye "
        f"level from {eye_m} m, at {ridge_r:.0f} m out (camera half-FOV is ~25 deg)"
    )
    print(
        f'hfield size attr for MJCF: "{args.half_m:g} {args.half_m:g} {elevation_z:.4f} 0.1"'
    )
    print(f'far geom pos attr for MJCF: "0 0 {z_min:.4f}"')


if __name__ == "__main__":
    main()
