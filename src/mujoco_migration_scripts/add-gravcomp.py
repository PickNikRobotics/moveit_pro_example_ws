#!/usr/bin/env python3
"""
add-gravcomp.py

Reads a MuJoCo MJCF XML file (preserving comments), sets gravcomp="1" on every <body> element,
and writes the modified MJCF back to the same file without stripping comments.

Usage:
    python3 add-gravcomp.py path/to/scene.xml
"""
import argparse
from xml.dom import minidom


def main():
    parser = argparse.ArgumentParser(
        description='Set gravcomp="1" on all <body> elements in an MJCF file, preserving comments.'
    )
    parser.add_argument(
        "mjcf_file", help="Path to the MJCF XML file to modify in-place"
    )
    args = parser.parse_args()

    # Parse the MJCF with minidom (preserves comments)
    doc = minidom.parse(args.mjcf_file)

    # Iterate over all <body> elements and set gravcomp="1"
    for node in doc.getElementsByTagName("body"):
        node.setAttribute("gravcomp", "1")

    # Write the XML back, preserving comments
    with open(args.mjcf_file, "w", encoding="utf-8") as f:
        doc.writexml(f, encoding="utf-8")


if __name__ == "__main__":
    main()
