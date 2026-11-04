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

"""Write the convex collision parts of the franka_description meshes MuJoCo collides as hulls.

MuJoCo collides a mesh as its convex hull, so a link whose collision mesh is far from convex
collides as the filled-in shape. Each part written here is already convex, so MuJoCo's own
hulling is a no-op and the union of the parts tracks the real shape.

Reads the source meshes from the franka_description submodule, which it never writes, and writes
the parts into this package at the source mesh's own relative path. A URDF needs none of them:
MoveIt collides the mesh itself, so only an MJCF should load the parts.

    python3 script/generate_collision_decomposition.py [--check | --paths]

Needs trimesh, coacd and scipy, which are not workspace dependencies: this is a dev tool run by
hand when a source mesh changes, not at build or launch time. --paths needs none of them.
"""

import argparse
import contextlib
import io
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE_MESHES = (
    PACKAGE.parent / "external_dependencies" / "franka_description" / "meshes"
)
PART_MESHES = PACKAGE / "meshes"

# Written into each part's 80-byte STL header, so a part is identifiable as a PickNik file
# derived from upstream geometry rather than an upstream file. The package test checks it.
HEADER_NOTICE = b"Modified by PickNik Inc. - convex collision part"

# Fixed so a rerun reproduces the committed files byte for byte.
SEED = 1337
THRESHOLD = 0.05
PREPROCESS_RESOLUTION = 100
RESOLUTION = 4000

# Cap on the parts per mesh, not the number written: the decomposition may return fewer.
# The fewest parts whose summed hull volume stays near the true mesh volume without going
# under, because under-covering leaves a hole in the collision model.
MAX_PARTS = {
    "robots/fr3v2/collision/link3.stl": 8,
    "robots/fr3v2/collision/link4.stl": 8,
    "robots/fr3v2/collision/link5.stl": 6,
    "accessories/franka_head_v0_2/collision/franka_head.stl": 4,
    "accessories/fr3_duo_mount_v0_3/collision/fr3_duo_mount.stl": 4,
}


def source_path(relative):
    """The upstream mesh a part set is decomposed from, in the submodule."""
    return SOURCE_MESHES / relative


def part_paths(relative, count):
    """Where this package holds the parts of one source mesh: its own relative path."""
    part = PART_MESHES / relative
    return [
        part.with_name(f"{part.stem}_convex_{index:02d}.stl") for index in range(count)
    ]


def committed_parts(relative):
    """Every part file this package already holds for one source mesh."""
    part = PART_MESHES / relative
    return sorted(part.parent.glob(f"{part.stem}_convex_*.stl"))


def obsolete_parts(relative, count):
    """Committed parts a run of this many hulls would not write, so they are stale."""
    written = set(part_paths(relative, count))
    return [path for path in committed_parts(relative) if path not in written]


def decompose(mesh, max_parts):
    """Convex hulls of one mesh's approximate convex decomposition, largest volume first."""
    import coacd
    import trimesh

    handle = coacd.Mesh(mesh.vertices, mesh.faces)
    quiet = io.StringIO()
    with contextlib.redirect_stdout(quiet), contextlib.redirect_stderr(quiet):
        parts = coacd.run_coacd(
            handle,
            threshold=THRESHOLD,
            max_convex_hull=max_parts,
            preprocess_resolution=PREPROCESS_RESOLUTION,
            resolution=RESOLUTION,
            seed=SEED,
        )
    hulls = [trimesh.Trimesh(vertices=v, faces=f).convex_hull for v, f in parts]
    return sorted(hulls, key=lambda hull: hull.volume, reverse=True)


def stl_bytes(hull):
    """Binary STL for one hull, with the PickNik notice in place of the free-form header."""
    import trimesh

    raw = bytearray(trimesh.exchange.stl.export_stl(hull))
    raw[0:80] = HEADER_NOTICE.ljust(80, b"\0")
    return bytes(raw)


def report_paths():
    """Resolve every input and output path, and report any source mesh that is missing."""
    missing = []
    for relative, max_parts in MAX_PARTS.items():
        source = source_path(relative)
        mark = "found" if source.is_file() else "MISSING"
        if not source.is_file():
            missing.append(source)
        committed = committed_parts(relative)
        print(f"{relative}\n    source {mark}: {source}")
        print(
            f"    parts in this package: {len(committed)} in {committed[0].parent}"
            if committed
            else f"    parts in this package: none yet in {part_paths(relative, 1)[0].parent}"
        )
    if missing:
        print(
            "\nmissing source meshes: check out the franka_description submodule",
            file=sys.stderr,
        )
        return 1
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--check",
        action="store_true",
        help="report the committed files against a fresh run without writing",
    )
    group.add_argument(
        "--paths",
        action="store_true",
        help="resolve the input and output paths and stop, without trimesh or coacd",
    )
    args = parser.parse_args()

    if args.paths:
        return report_paths()

    import trimesh

    drifted = []
    stale = []
    for relative, max_parts in MAX_PARTS.items():
        source = source_path(relative)
        mesh = trimesh.load(source, force="mesh")
        hulls = decompose(mesh, max_parts)
        summed = sum(hull.volume for hull in hulls)
        print(
            f"{relative}\n"
            f"    single hull {mesh.convex_hull.volume / mesh.volume:5.2f}x the mesh volume"
            f" -> {len(hulls)} parts at {summed / mesh.volume:5.3f}x"
        )
        for path, hull in zip(part_paths(relative, len(hulls)), hulls):
            written = stl_bytes(hull)
            if args.check:
                if not path.exists() or path.read_bytes() != written:
                    drifted.append(path)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(written)

        # A run returning fewer hulls than are committed writes no file over the
        # higher-index ones, so compare the whole set and not just what it wrote.
        obsolete = obsolete_parts(relative, len(hulls))
        stale.extend(obsolete)
        if not args.check:
            for path in obsolete:
                path.unlink()
                print(f"    removed stale {path.name}")

    if args.check:
        for label, paths in (("drifted from a fresh run", drifted), ("stale", stale)):
            if paths:
                print(f"\n{label}:", file=sys.stderr)
                for path in paths:
                    print(f"    {path.relative_to(PART_MESHES)}", file=sys.stderr)
        if drifted or stale:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
