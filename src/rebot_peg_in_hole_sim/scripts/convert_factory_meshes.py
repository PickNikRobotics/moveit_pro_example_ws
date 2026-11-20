#!/usr/bin/env python3
# Copyright 2026 PickNik Inc. All rights reserved. BSD-3-Clause.
"""Convert the three unmodified factory_sim 3MF meshes to metre-unit binary STL."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zipfile

import numpy as np


def convert(source, destination):
    """Read the single identity-transformed mesh; preserve its native coordinates."""
    with zipfile.ZipFile(source) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
    namespace = {"m": root.tag.split("}")[0][1:]}
    if root.get("unit") != "meter":
        raise ValueError("Expected the factory source's explicit metre unit")
    items = root.findall("m:build/m:item", namespace)
    if len(items) != 1 or "transform" in items[0].attrib:
        raise ValueError("Expected one identity-transformed build item")
    obj = root.find(
        f"m:resources/m:object[@id='{items[0].get('objectid')}']", namespace
    )
    if obj is None or obj.find("m:mesh", namespace) is None:
        raise ValueError("Expected a mesh, rather than a component assembly")
    vertices = np.array(
        [
            [float(vertex.get(axis)) for axis in ("x", "y", "z")]
            for vertex in obj.findall("m:mesh/m:vertices/m:vertex", namespace)
        ]
    )
    faces = np.array(
        [
            [int(face.get(index)) for index in ("v1", "v2", "v3")]
            for face in obj.findall("m:mesh/m:triangles/m:triangle", namespace)
        ]
    )
    triangles = vertices[faces]
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    lengths = np.linalg.norm(normals, axis=1)
    normals[lengths > 0] /= lengths[lengths > 0, None]
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("wb") as output:
        output.write(
            b"Factory 3MF conversion; native metres; BSD-3-Clause".ljust(80, b"\0")
        )
        output.write(struct.pack("<I", len(faces)))
        for normal, triangle in zip(normals, triangles):
            output.write(struct.pack("<12fH", *normal, *triangle.ravel(), 0))
    return {
        "source": source.name,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "stl_sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
        "unit": "meter",
        "build_transform": "identity",
        "faces": len(faces),
        "bounds_m": [vertices.min(0).tolist(), vertices.max(0).tolist()],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="factory_sim/3mf directory")
    parser.add_argument(
        "destination", type=Path, help="consuming package assets directory"
    )
    args = parser.parse_args()
    manifest = {}
    for name in ("part", "jig_base", "jig_post"):
        manifest[name] = convert(
            args.source / f"pick_and_place_bracket_{name}.3mf",
            args.destination / f"{name}.stl",
        )
    (args.destination / "provenance.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
