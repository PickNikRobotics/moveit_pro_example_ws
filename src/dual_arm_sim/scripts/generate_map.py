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

"""Write the Nav2 map, maps/scene.pgm and maps/scene.yaml, from the tables in the scene.

Each table contributes its whole surface projected onto the ground, so the base cannot plan
under an overhanging top. The map's edge is marked occupied, which also keeps the image binary
to the end-of-file pre-commit hook.

    python3 scripts/generate_map.py [output_dir]
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
SCENE = PACKAGE / "description" / "mujoco" / "scene.xml"
RESOLUTION = 0.05
HALF_EXTENT = 5.0
FREE, OCCUPIED = 254, 0


def _floats(text):
    return [float(v) for v in text.split()]


def table_boxes(scene=SCENE):
    """World (x_min, x_max, y_min, y_max) of every geom of every table body.

    The union of a table's tops and legs, so an overhanging top is an obstacle too. Each top
    happens to contain its own legs, which therefore add no cells.
    """
    boxes = []
    for body in ET.parse(scene).getroot().iter("body"):
        if not body.get("name", "").startswith("table"):
            continue
        if body.get("quat") or body.get("euler") or body.get("axisangle"):
            # An axis-aligned box per geom is only the true footprint while the body is unrotated.
            raise ValueError(
                f"table body {body.get('name')!r} is rotated; extend the projection"
            )
        bx, by, _ = _floats(body.get("pos", "0 0 0"))
        for geom in body.findall("geom"):
            gx, gy, _ = _floats(geom.get("pos", "0 0 0"))
            sx, sy, _ = _floats(geom.get("size"))
            cx, cy = bx + gx, by + gy
            boxes.append((cx - sx, cx + sx, cy - sy, cy + sy))
    return boxes


def occupancy(boxes):
    """Rows top (maximum y) first, as map_server reads the image."""
    cells = round(2 * HALF_EXTENT / RESOLUTION)
    rows = [[FREE] * cells for _ in range(cells)]
    for i in range(cells):
        rows[0][i] = rows[-1][i] = rows[i][0] = rows[i][-1] = OCCUPIED
    for x_min, x_max, y_min, y_max in boxes:
        for col in range(cells):
            x0 = -HALF_EXTENT + col * RESOLUTION
            if x0 + RESOLUTION <= x_min or x0 >= x_max:
                continue
            for row in range(cells):
                y0 = HALF_EXTENT - (row + 1) * RESOLUTION
                if y0 + RESOLUTION <= y_min or y0 >= y_max:
                    continue
                rows[row][col] = OCCUPIED
    return rows


def write(output_dir):
    rows = occupancy(table_boxes())
    header = f"P5\n{len(rows[0])} {len(rows)}\n255\n".encode()
    (output_dir / "scene.pgm").write_bytes(
        header + bytes(v for row in rows for v in row)
    )
    (output_dir / "scene.yaml").write_text(
        "image: scene.pgm\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{-HALF_EXTENT}, {-HALF_EXTENT}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )


if __name__ == "__main__":
    write(Path(sys.argv[1]) if len(sys.argv) > 1 else PACKAGE / "maps")
