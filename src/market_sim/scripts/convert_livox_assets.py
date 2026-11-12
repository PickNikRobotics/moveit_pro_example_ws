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

"""Convert the retained A2 CAD meshes into metre-scale sim/viewer assets.

Offline tool, not a runtime dependency. Requires numpy, trimesh, scipy, rtree,
shapely and fast-simplification. Run from any directory; all paths are package-local.
The bracket uses base-aligned scan-O axes. Only sensor meshes are rotated back
into native axes, so the fixed livox_frame joint applies the roll exactly once.
"""

import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

ASSETS = Path(__file__).resolve().parents[1] / "description/meshes/livox_mid360"


def surface_samples(mesh, seed, count=6000):
    """Deterministic area-weighted face samples, augmented with every vertex."""
    rng = np.random.default_rng(seed)
    faces = rng.choice(len(mesh.faces), count, p=mesh.area_faces / mesh.area)
    uv = rng.random((count, 2))
    uv[uv.sum(axis=1) > 1] = 1 - uv[uv.sum(axis=1) > 1]
    triangle = mesh.triangles[faces]
    points = triangle[:, 0] + uv[:, :1] * (triangle[:, 1] - triangle[:, 0])
    points += uv[:, 1:] * (triangle[:, 2] - triangle[:, 0])
    return np.vstack((mesh.vertices, points))


def distance_mm(target, points):
    # The proximity consumer also has absolute tolerances. Check millimetre
    # geometry, otherwise tiny coplanar triangles can look displaced by them.
    target_mm = target.copy()
    target_mm.apply_scale(1000)
    return float(trimesh.proximity.closest_point(target_mm, points * 1000)[1].max())


def convert_visual(mesh, output, seed):
    # QEM uses absolute thresholds: work in the CAD's millimetres, not metres.
    # Aggressive reduction destroys small through-holes, so increase retention
    # until the actual watertightness/surface checks pass. Never convexify visuals.
    cad = mesh.copy()
    cad.apply_scale(1000)
    for retention in (0.70, 0.85, 0.95, 0.98, 1.0):
        reduced = (
            cad.simplify_quadric_decimation(face_count=int(len(cad.faces) * retention))
            if retention < 1.0
            else cad.copy()
        )
        reduced.apply_scale(0.001)
        reduced.fix_normals()
        errors = [
            distance_mm(reduced, surface_samples(mesh, seed)),
            distance_mm(mesh, surface_samples(reduced, seed)),
        ]
        if reduced.is_watertight and max(errors) <= 0.20:
            break
    else:
        raise ValueError(f"{output.name}: invalid source surface")
    reduced.export(output)
    return {
        "source_faces": len(mesh.faces),
        "visual_faces": len(reduced.faces),
        "retention_target": retention,
        "bounds_m": reduced.bounds.tolist(),
        "sampled_bidirectional_max_error_mm": errors,
        "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    }


def crown_hulls(mesh):
    """Three separately clipped U-rails; never convexify the complete crown."""
    # Exact clipping at x=35 mm splits the front from the two side rails.
    front = mesh.slice_plane([0.035, 0, 0], [1, 0, 0]).convex_hull
    back = mesh.slice_plane([0.035, 0, 0], [-1, 0, 0])
    left = back.slice_plane([0, 0, 0], [0, 1, 0]).convex_hull
    right = back.slice_plane([0, 0, 0], [0, -1, 0]).convex_hull
    return {"crown_front": front, "crown_left": left, "crown_right": right}


def main():
    source = json.loads((ASSETS / "cad/source.json").read_text())
    visual = ASSETS / "visual"
    collision = ASSETS / "collision"
    visual.mkdir(exist_ok=True)
    collision.mkdir(exist_ok=True)
    record = {"visuals": {}, "collisions": {}}
    total_mass = 0.0
    first_moment = np.zeros(3)
    inertia_origin = np.zeros((3, 3))
    for index, part in enumerate(source["parts"] + source["sensor_parts"]):
        name = part["name"]
        path = ASSETS / "cad" / f"{name}.stl"
        if hashlib.sha256(path.read_bytes()).hexdigest() != part["source_stl_sha256"]:
            raise ValueError(f"Source hash mismatch: {name}")
        mesh = trimesh.load_mesh(path)
        mesh.apply_scale(0.001)
        if not mesh.is_watertight or mesh.volume <= 0:
            raise ValueError(f"Invalid CAD solid: {name}")
        if name.startswith("sensor_"):
            # CAD is already inverted. Roll it back before putting it on the
            # rolled native link; do not roll the bracket or move its origin.
            mesh.apply_transform(np.diag([1.0, -1.0, -1.0, 1.0]))
        else:
            mass = part["mass_kg"]
            if mass is None:
                mass = mesh.volume * 2700.0
            properties = mesh.mass_properties
            com = properties["center_mass"]
            inertia = properties["inertia"] * mass / properties["mass"]
            total_mass += mass
            first_moment += mass * com
            inertia_origin += inertia + mass * (
                np.dot(com, com) * np.eye(3) - np.outer(com, com)
            )
            hulls = (
                crown_hulls(mesh) if name == "P02_crown" else {name: mesh.convex_hull}
            )
            for label, hull in hulls.items():
                output = collision / f"{label}.stl"
                hull.export(output)
                record["collisions"][label] = {
                    "faces": len(hull.faces),
                    "bounds_m": hull.bounds.tolist(),
                    "source_part": name,
                }
        record["visuals"][name] = convert_visual(mesh, visual / f"{name}.stl", index)
    com = first_moment / total_mass
    inertia_com = inertia_origin - total_mass * (
        np.dot(com, com) * np.eye(3) - np.outer(com, com)
    )
    record["mount_inertial"] = {
        "mass_kg": total_mass,
        "com_m": com.tolist(),
        "inertia_kg_m2": inertia_com.tolist(),
        "authority": source["print_mass_authority"]
        + "; 6061 density 2700 kg/m3; uniform density per part",
    }
    (ASSETS / "conversion.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
