#!/usr/bin/env python3
"""
add-position-actuators.py

Reads a MuJoCo MJCF XML file (preserving comments), adds a <position> actuator for every <joint>,
and writes the modified MJCF back in-place.

Each <position> uses:
  joint="<joint_name>"
  name="<joint_name>"
  ctrlrange="-3.14 3.14"
  kp="1000"
  forcelimited="false"
  dampratio="1"

Usage:
    python3 add-position-actuators.py path/to/scene.xml
"""
import argparse
from xml.dom import minidom, Node


def main():
    parser = argparse.ArgumentParser(
        description="Add position actuators for every joint in an MJCF, preserving comments."
    )
    parser.add_argument(
        "mjcf_file", help="Path to the MJCF XML file to modify in-place"
    )
    args = parser.parse_args()

    # Parse the MJCF, preserving comments
    doc = minidom.parse(args.mjcf_file)
    root = doc.documentElement

    # Collect joint names
    joint_nodes = doc.getElementsByTagName("joint")
    joint_names = [
        n.getAttribute("name") for n in joint_nodes if n.hasAttribute("name")
    ]

    # Find or create <actuator> element
    actuators = doc.getElementsByTagName("actuator")
    if actuators:
        actuator = actuators[0]
    else:
        actuator = doc.createElement("actuator")
        # insert before first worldbody or at end
        inserted = False
        for child in root.childNodes:
            if child.nodeType == Node.ELEMENT_NODE and child.tagName == "worldbody":
                root.insertBefore(actuator, child)
                inserted = True
                break
        if not inserted:
            root.appendChild(actuator)
        # add newline for readability
        actuator.appendChild(doc.createTextNode("\n"))

    # Append <position> for each joint
    for name in joint_names:
        # whitespace + indent
        actuator.appendChild(doc.createTextNode("    "))
        pos = doc.createElement("position")
        pos.setAttribute("joint", name)
        pos.setAttribute("name", name)
        pos.setAttribute("ctrlrange", "-3.14 3.14")
        pos.setAttribute("kp", "1000")
        pos.setAttribute("forcelimited", "false")
        pos.setAttribute("dampratio", "1")
        actuator.appendChild(pos)
        actuator.appendChild(doc.createTextNode("\n"))

    # Write back to file
    with open(args.mjcf_file, "w", encoding="utf-8") as f:
        doc.writexml(f, addindent="", newl="")


if __name__ == "__main__":
    main()
