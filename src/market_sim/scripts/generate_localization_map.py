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

"""Rasterize lidar-height visual surfaces for localization, independently of Nav2's solid footprints."""

import argparse
from pathlib import Path

import mujoco
import numpy as np
import yaml
from PIL import Image


CYLINDER_SEGMENTS = 64

BOX_VERTICES = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
BOX_FACES = np.array(
    [
        [0, 1, 3],
        [0, 3, 2],
        [4, 6, 7],
        [4, 7, 5],
        [0, 4, 5],
        [0, 5, 1],
        [2, 3, 7],
        [2, 7, 6],
        [0, 2, 6],
        [0, 6, 4],
        [1, 5, 7],
        [1, 7, 3],
    ]
)


def section_segments(triangles, height):
    """Intersect triangles with a horizontal plane, retaining only crossing edges."""
    active = triangles[
        (triangles[:, :, 2].min(axis=1) <= height)
        & (triangles[:, :, 2].max(axis=1) > height)
    ]
    endpoints = {}
    for a, b in ((0, 1), (1, 2), (2, 0)):
        va, vb = active[:, a], active[:, b]
        keep = ((va[:, 2] <= height) & (vb[:, 2] > height)) | (
            (vb[:, 2] <= height) & (va[:, 2] > height)
        )
        aa, bb = va[keep], vb[keep]
        points = aa + (bb - aa) * ((height - aa[:, 2]) / (bb[:, 2] - aa[:, 2]))[:, None]
        for index, point in zip(np.flatnonzero(keep), points):
            endpoints.setdefault(int(index), []).append(point[:2])
    return [pair for pair in endpoints.values() if len(pair) == 2]


def generate(package, resolution=0.02):
    """Use both real lidar heights and the default scene; never sample a moving robot's truth."""
    model = mujoco.MjModel.from_xml_path(str(package / "mjcf/scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    mujoco.mj_forward(model, data)
    meta = yaml.safe_load((package / "maps/market.yaml").read_text())
    original = np.array(Image.open(package / "maps" / meta["image"]))
    origin = np.array(meta["origin"][:2])
    size = np.ceil(
        np.array(original.shape[::-1]) * meta["resolution"] / resolution
    ).astype(int)
    grid = np.full(size[::-1], 254, dtype=np.uint8)
    heights = [
        data.cam_xpos[model.camera("lidar_" + side).id, 2] for side in ("front", "rear")
    ]
    robot_root = model.body_rootid[model.camera("lidar_front").bodyid]
    for geom in range(model.ngeom):
        if (
            model.body_rootid[model.geom_bodyid[geom]] == robot_root
            or model.body(model.geom_bodyid[geom]).name.startswith("stocking_")
            or model.geom_group[geom] > 2
            or model.geom_rgba[geom, 3] == 0
        ):
            continue
        if not any(
            abs(height - data.geom_xpos[geom, 2]) <= model.geom_rbound[geom]
            for height in heights
        ):
            continue
        kind = model.geom_type[geom]
        if kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            local = model.mesh_vert[start : start + count]
            start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            faces = model.mesh_face[start : start + count]
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            local, faces = BOX_VERTICES * model.geom_size[geom], BOX_FACES
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            angles = np.arange(CYLINDER_SEGMENTS) * 2 * np.pi / CYLINDER_SEGMENTS
            local = np.array(
                [
                    [
                        model.geom_size[geom, 0] * np.cos(a),
                        model.geom_size[geom, 0] * np.sin(a),
                        z,
                    ]
                    for z in (-model.geom_size[geom, 1], model.geom_size[geom, 1])
                    for a in angles
                ]
            )
            faces = np.array(
                [
                    [i, (i + 1) % CYLINDER_SEGMENTS, i + CYLINDER_SEGMENTS]
                    for i in range(CYLINDER_SEGMENTS)
                ]
                + [
                    [
                        (i + 1) % CYLINDER_SEGMENTS,
                        (i + 1) % CYLINDER_SEGMENTS + CYLINDER_SEGMENTS,
                        i + CYLINDER_SEGMENTS,
                    ]
                    for i in range(CYLINDER_SEGMENTS)
                ]
                + [[0, i, i + 1] for i in range(1, CYLINDER_SEGMENTS - 1)]
                + [
                    [
                        CYLINDER_SEGMENTS,
                        i + CYLINDER_SEGMENTS + 1,
                        i + CYLINDER_SEGMENTS,
                    ]
                    for i in range(1, CYLINDER_SEGMENTS - 1)
                ]
            )
        elif kind in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE):
            # Sample the rounded cart handles with the same angular density as cylinders.
            radius = model.geom_size[geom, 0]
            half = (
                model.geom_size[geom, 1]
                if kind == mujoco.mjtGeom.mjGEOM_CAPSULE
                else 0.0
            )
            latitudes = np.linspace(-np.pi / 2, np.pi / 2, CYLINDER_SEGMENTS // 2 + 1)
            angles = np.arange(CYLINDER_SEGMENTS) * 2 * np.pi / CYLINDER_SEGMENTS
            local = np.array(
                [
                    [
                        radius * np.cos(phi) * np.cos(a),
                        radius * np.cos(phi) * np.sin(a),
                        radius * np.sin(phi) + (half if phi >= 0 else -half),
                    ]
                    for phi in latitudes
                    for a in angles
                ]
            )
            faces = []
            for row in range(len(latitudes) - 1):
                for col in range(CYLINDER_SEGMENTS):
                    a = row * CYLINDER_SEGMENTS + col
                    b = row * CYLINDER_SEGMENTS + (col + 1) % CYLINDER_SEGMENTS
                    faces += [
                        [a, b, a + CYLINDER_SEGMENTS],
                        [b, b + CYLINDER_SEGMENTS, a + CYLINDER_SEGMENTS],
                    ]
            faces = np.array(faces)
        elif kind == mujoco.mjtGeom.mjGEOM_PLANE:
            # The horizontal floor is below both scans.
            continue
        else:
            raise ValueError(
                f"Cannot generate the localization map: visible geometry {model.geom(geom).name!r} has unsupported MuJoCo type {kind}. Add its horizontal section to generate_localization_map.py before regenerating the map."
            )
        vertices = local @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        for height in heights:
            for a, b in section_segments(vertices[faces], height):
                count = max(2, int(np.linalg.norm(b - a) / resolution * 3) + 1)
                indices = np.floor(
                    (np.linspace(a, b, count) - origin) / resolution
                ).astype(int)
                keep = (
                    (indices[:, 0] >= 0)
                    & (indices[:, 0] < size[0])
                    & (indices[:, 1] >= 0)
                    & (indices[:, 1] < size[1])
                )
                indices = indices[keep]
                grid[indices[:, 1], indices[:, 0]] = 0
    return grid, dict(meta, image="localization.png", resolution=resolution)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--package", type=Path, default=Path(__file__).resolve().parents[1]
    )
    args = parser.parse_args()
    grid, meta = generate(args.package)
    Image.fromarray(grid[::-1]).save(args.package / "maps/localization.png")
    (args.package / "maps/localization.yaml").write_text(
        "# Generated by scripts/generate_localization_map.py.\n" + yaml.safe_dump(meta)
    )


if __name__ == "__main__":
    main()
