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


"""Generates the render-only twin of the driven ground (assets/lunar_hfield_visual.png).

Offline; run manually and commit the output.

MuJoCo renders and collides the same hfield mesh, and the driven field's 2.0M triangles cost 134 ms
per camera frame - about 2.5 fps over three cameras. That field cannot be shrunk: the Dead Reckon
Square's closure error is calibrated against it, and a 350x350 collision surface moved that error
from 0.6331 m to 1.8232 m. So it stays full resolution in geom group 3, which MuJoCo does not
render, and this twin draws in its place: 5.3 ms, physics bit-identical.

A pure resample - generate_terrain.py is untouched. Re-run whenever the driven field is
regenerated; both hfield sizes and both geom pos values in husky_scene.xml must stay identical.

Usage: python3 generate_visual_terrain.py [--grid 350]
"""
import argparse
from pathlib import Path

from PIL import Image

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"

# Just past a sharp cost cliff, not a gradual tradeoff: 1000 -> 134 ms, 500 -> 33 ms, 350 -> 5.2 ms,
# 250 -> 4.3 ms. A 6x drop for a 2x triangle reduction is the signature of fitting a driver limit.
# Below 350 buys almost nothing and only loses relief. At 20 m across, 350 cells is ~5.7 cm.
DEFAULT_GRID = 350


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--grid", type=int, default=DEFAULT_GRID, help="cells per side")
    parser.add_argument("--source", type=Path, default=ASSETS / "lunar_hfield.png")
    parser.add_argument("--out", type=Path, default=ASSETS / "lunar_hfield_visual.png")
    args = parser.parse_args()

    with Image.open(args.source) as source:
        src_size = source.size
        # LANCZOS rather than NEAREST: this is a height field, so a resample that averages
        # neighbouring cells keeps the surface smooth, while point sampling would alias crater rims
        # into visible stair steps at exactly the grazing angles the robot's camera views them from.
        resampled = source.resize((args.grid, args.grid), Image.LANCZOS)
        resampled.save(args.out)

    print(
        f"wrote {args.out} ({args.grid}x{args.grid}, from {src_size[0]}x{src_size[1]})"
    )
    print(
        f"  triangles per camera pass: {2 * (args.grid - 1) ** 2:,} (was {2 * (src_size[0] - 1) ** 2:,})"
    )
    print(f"  cell size at 20 m across: {2000 / args.grid:.1f} cm")
    print(
        "  husky_scene.xml: keep this <hfield>'s size and its geom's pos identical to lunar_hfield's"
    )


if __name__ == "__main__":
    main()
