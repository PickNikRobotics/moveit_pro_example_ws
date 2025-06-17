#!/usr/bin/env python3
"""
assign-geom-names.py

Reads a MuJoCo MJCF XML file (preserving comments), assigns unique names to all <geom> elements,
and writes the modified MJCF back to the same file without stripping comments.
If multiple geoms under the same <body> share a base name, subsequent ones get a numeric suffix.

Usage:
    python3 assign-geom-names.py path/to/scene.xml
"""
import argparse
import os
from xml.dom import minidom, Node


def find_parent_body(node):
    """Climb the DOM tree to find the nearest enclosing <body> element."""
    parent = node.parentNode
    while parent and not (
        parent.nodeType == Node.ELEMENT_NODE and parent.tagName == "body"
    ):
        parent = parent.parentNode
    return parent


def main():
    parser = argparse.ArgumentParser(
        description="Assign unique names to all <geom> in an MJCF, preserving comments."
    )
    parser.add_argument("mjcf_file", help="Path to MJCF XML file to modify in place")
    args = parser.parse_args()

    # Parse with minidom to preserve comments
    doc = minidom.parse(args.mjcf_file)

    # Find all geom elements
    geoms = doc.getElementsByTagName("geom")

    # Track counts of base names per body
    counts = {}
    for geom in geoms:
        # Determine base name
        if geom.hasAttribute("mesh"):
            mesh_ref = geom.getAttribute("mesh")
            base = os.path.splitext(os.path.basename(mesh_ref))[0]
        else:
            body = find_parent_body(geom)
            if body and body.hasAttribute("name"):
                base = body.getAttribute("name")
            else:
                base = (
                    geom.getAttribute("type") if geom.hasAttribute("type") else "geom"
                )

        # Identify parent body for grouping key
        parent_body = find_parent_body(geom)
        key = (id(parent_body), base)

        # Increment count and build unique name
        counts[key] = counts.get(key, 0) + 1
        if counts[key] > 1:
            name = f"{base}{counts[key]}"
        else:
            name = base

        # Assign name attribute
        geom.setAttribute("name", name)

    # Write back, preserving comments
    with open(args.mjcf_file, "w", encoding="utf-8") as f:
        doc.writexml(f)


if __name__ == "__main__":
    main()
