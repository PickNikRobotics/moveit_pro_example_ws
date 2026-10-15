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

"""Generate the market_sim supermarket from its text floor plan.

Reads FLOOR_PLAN below and the shelf assets in ``market_assets/``, and writes the MuJoCo
scene parts (``mjcf/market/``), their mesh and texture (``mjcf/assets/market/``), the Nav2
map (``maps/market.{png,yaml}``), the localization map (``maps/market_localization.{png,yaml}``)
and the Navigate to Aisle Objectives.

Run from anywhere, with numpy installed:

    python3 src/market_sim/scripts/generate_market.py

See the package README for the layout rules.
"""

import argparse
import json
import math
import re
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import numpy as np

PACKAGE_DIR = Path(__file__).resolve().parents[1]
ASSET_DIR = PACKAGE_DIR / "market_assets"
# Output folders, relative to the package (or to --output-root).
MJCF_OUT = Path("mjcf") / "market"
MESH_OUT = Path("mjcf") / "assets" / "market"
MAP_OUT = Path("maps")
OBJECTIVES_OUT = Path("objectives")

# The requested floor plan, verbatim. '//' lines are labels and take no floor space.
FLOOR_PLAN = r"""
 ______________________________________________________________________
|                                                                      |
//      E1   E2   E3   E4   E5   E6   E7   E8   E9   E10  E11
|       }{   }{   }{   }{   }{   }{   }{   }{   }{   }{   }{           |
|       }{   }{   }{   }{   }{   }{   }{   }{   }{   }{   }{           |
|       }{   }{   }{   }{   }{   }{   }{   }{   }{   }{   }{           |
|       }{   }{   }{   }{   }{   }{   }{   }{   }{   }{   }{           |
|                                                                      |
//        I1          I2          I3          I4          I5
|        ----        ----        ----        ----        ----          |
|                                                                      |
//       #A                 #B                 #C           #D
|    ----------    --------------------    ----------    ----------    |  //7
|    ----------    --------------------  --  --------      --------    |  //6
|    --------      --------------------    ----------    ----------    |  //5
|    ----------    --------------------      --------  --  --------    |  //4
|    --------      --------------------    ----------    ----------    |  //3
|    ----------    --------------------  --  --------      --------    |  //2
|    ----------    --------------------    ----------    ----------    |  //1
|                                                                      |
//      R1   R2   R3   R4   R5   R6   R7   R8   R9   R10  R11
|       ][   ][   ][   ][   ][   ][   ][   ][   ][   ][   ][           |
|       ][   ][   ][   ][   ][   ][   ][   ][   ][   ][   ][           |
|        ______________________________________________________        |
"""

# One text column is one shelf asset's width. Every '-' and every '}{' is a gondola: two
# shelf units back to back, facing north and south ('-') or west and east ('}{').
COLUMN_PITCH = 1.304
GONDOLA_DEPTH = 1.114
AISLE_WIDTH = 1.6
# North-south depth of each kind of text line.
LINE_PITCH = {
    "open": 3.0,
    "E": COLUMN_PITCH,
    "I": GONDOLA_DEPTH + AISLE_WIDTH,
    "AD": GONDOLA_DEPTH + AISLE_WIDTH,
    "R": 1.5,
    "exit": 3.0,
}
# Aisles with a "Navigate to Aisle" Objective.
EI_REAL = ("E1", "E5", "I2")
AD_REAL = ("A1", "B2", "C3", "D4")
REAL_AISLES = EI_REAL + AD_REAL
# Gondola column, from the row's west end, that each A to D aisle goal faces.
AD_GOAL_COLUMNS = {"A1": 6, "B2": 11, "C3": 6, "D4": 4}

WALL_THICKNESS = 0.2
WALL_HEIGHT = 2.5
REGISTER_HALF_SIZE = (0.5, 1.3, 0.45)
# The robot boots here, on the open floor south of row 1; world and map share this origin.
START_LINE_INDEX = 19

# The default keyframe: the Mobile FR3 Duo's default pose.
# Order: planar joints, 9 passive wheel joints, spine, then each arm with its two fingers.
ROBOT_QPOS = (
    [0.0] * 3
    + [0.0] * 9
    + [0.0]
    + [0.0, -0.7853981633974483, 0.0, -2.356194490192345]
    + [0.0, 1.5707963267948966, 0.7853981633974483, 0.0, 0.0]
    + [0.0, -0.7853981633974483, 0.0, -2.356194490192345]
    + [0.0, 1.5707963267948966, 0.7853981633974483, 0.0, 0.0]
)
ROBOT_CTRL = (
    [0.0] * 3
    + [0.0]
    + [0.0, -0.7853981633974483, 0.0, -2.356194490192345]
    + [0.0, 1.5707963267948966, 0.7853981633974483, 0.0]
    + [0.0, -0.7853981633974483, 0.0, -2.356194490192345]
    + [0.0, 1.5707963267948966, 0.7853981633974483, 0.0]
)

# Distance from a shelf's front face to the base centre in a "Navigate to Aisle" goal. It keeps
# the stowed-arms Nav2 footprint clear of both racks while it turns to face the shelf.
AISLE_STANDOFF = 0.96

# Gondolas draw the shelf from its collision boxes only, to cut triangles.
LOW_SHELF_BOX_MATERIALS = {
    "upright": "shelf_frame_steel",
    "bracket": "shelf_frame_steel",
    "channel": "shelf_price_channel",
    "back": "shelf_panel_steel",
    "shelf_base": "shelf_panel_steel",
    "_collision": "shelf_panel_steel",
}

# Height above the floor of both lidars' scan plane (the mounts in mjcf/mobile_fr3_duo.xml);
# a test keeps the two equal. The localization map holds what crosses this plane.
LIDAR_HEIGHT = 0.19065

GENERATED = "GENERATED by scripts/generate_market.py. Do not edit."
MAP_RESOLUTION = 0.05
MAP_MARGIN = 2.0
PALETTE_BLOCK = 16
PALETTE_COLUMNS = 8


def quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_matrix(q):
    """Rotation matrix of a (w, x, y, z) quaternion."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


# ---------------------------------------------------------------- floor plan


class Layout:
    """Gondolas, registers and walls in plan coordinates.

    Plan x runs east from the inside of the west wall; plan y runs south from the
    inside of the north wall. ``to_world`` moves both into the world frame. A gondola's
    anchor is the plan point where its two units meet back to back.
    """

    def __init__(self, plan_text):
        self.gondolas = []
        self.registers = []
        self.line_spans = {}
        self.bottom_wall_columns = None
        lines = [line.rstrip() for line in plan_text.strip("\n").split("\n")]
        self.width = 70 * COLUMN_PITCH
        y = 0.0
        section = None
        labels = {}
        for index, line in enumerate(lines):
            if index == 0:
                continue
            if line.startswith("//"):
                section, labels = self._parse_labels(line)
                continue
            body = line[: line.rindex("|") + 1] if "|" in line[1:] else line
            kind = self._line_kind(body, section)
            pitch = LINE_PITCH[kind]
            self.line_spans[index] = (y, y + pitch)
            if kind == "E":
                self._add_e_gondolas(body, labels, y)
            elif kind in ("I", "AD"):
                row = self._row_number(line) if kind == "AD" else None
                self._add_row_gondolas(body, kind, labels, row, y)
            elif kind == "R":
                self._add_registers(body, labels, y, pitch)
            elif kind == "exit":
                first = body.index("_")
                last = body.rindex("_")
                self.bottom_wall_columns = (first, last)
            y += pitch
        self.depth = y
        start_y = sum(self.line_spans[START_LINE_INDEX]) / 2
        self.origin = (self.width / 2, start_y)

    @staticmethod
    def _parse_labels(line):
        labels = {m.start(): m.group() for m in re.finditer(r"\S+", line[2:])}
        labels = {col + 2: text for col, text in labels.items()}
        first = next(iter(labels.values()))
        if first.startswith("E"):
            return "E", labels
        if first.startswith("I"):
            return "I", labels
        if first.startswith("#"):
            return "AD", {col: text[1:] for col, text in labels.items()}
        return "R", labels

    @staticmethod
    def _line_kind(body, section):
        if "}{" in body:
            return "E"
        if "][" in body:
            return "R"
        if "-" in body:
            return section
        if "_" in body:
            return "exit"
        return "open"

    @staticmethod
    def _row_number(line):
        return int(re.search(r"//\s*(\d+)\s*$", line).group(1))

    @staticmethod
    def column_center(col):
        return (col - 0.5) * COLUMN_PITCH

    def _add_gondola(self, aisle, anchor, axis):
        index = 1 + sum(g["aisle"] == aisle for g in self.gondolas)
        self.gondolas.append(
            {
                "name": f"{aisle}_{index}",
                "aisle": aisle,
                "index": index,
                "anchor": anchor,
                "axis": axis,
            }
        )

    def _add_e_gondolas(self, body, labels, y_top):
        # Numbered 1 to 4 from north to south.
        for col, label in labels.items():
            assert body[col : col + 2] == "}{", (label, body)
            self._add_gondola(
                label, (col * COLUMN_PITCH, y_top + COLUMN_PITCH / 2), "we"
            )

    def _add_row_gondolas(self, body, kind, labels, row, y_top):
        # Numbered from the west end of the row.
        for match in re.finditer(r"-+", body):
            start, end = match.start(), match.end()
            owner = [text for col, text in labels.items() if start <= col < end]
            if kind == "AD":
                aisle = f"{owner[0]}{row}" if owner else f"island{row}_{start}"
            else:
                aisle = owner[0]
            for col in range(start, end):
                anchor = (self.column_center(col), y_top + GONDOLA_DEPTH / 2)
                self._add_gondola(aisle, anchor, "ns")

    def _add_registers(self, body, labels, y_top, pitch):
        if any(r["y_top"] < y_top for r in self.registers):
            return
        for col, label in labels.items():
            assert body[col : col + 2] == "][", (label, body)
            self.registers.append(
                {
                    "name": f"register_{label}",
                    "x": col * COLUMN_PITCH,
                    "y_top": y_top,
                    "y_span": 2 * pitch,
                }
            )

    def to_world(self, x, y):
        return (x - self.origin[0], -(y - self.origin[1]))


# ---------------------------------------------------------------- assets


def load_obj(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            vertices.append([float(v) for v in parts[1:4]])
        elif parts[0] == "f":
            idx = [int(p.split("/")[0]) - 1 for p in parts[1:]]
            for k in range(1, len(idx) - 1):
                faces.append([idx[0], idx[k], idx[k + 1]])
    return np.array(vertices), np.array(faces, dtype=int)


def box_mesh(half):
    hx, hy, hz = half
    v = np.array(
        [
            [sx * hx, sy * hy, sz * hz]
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
        ]
    )
    f = [
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
    return v, np.array(f)


class MeshBuilder:
    """Merges triangles into one OBJ whose faces index a colour-palette texture."""

    def __init__(self, palette):
        self.palette = palette
        self.vertices = []
        self.faces = []
        self.count = 0

    def add(self, vertices, faces, color):
        texel = self.palette.index(self.palette.add(color))
        self.vertices.append(vertices)
        self.faces.append((faces + self.count, texel))
        self.count += len(vertices)

    def write(self, path, header):
        """Write the OBJ; call only once every mesh has added its colours to the palette."""
        lines = [f"# {header}"]
        for block in self.vertices:
            lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in block]
        lines += [f"vt {u:.6f} {v:.6f}" for u, v in self.palette.texcoords()]
        for faces, texel in self.faces:
            t = texel + 1
            lines += [f"f {a + 1}/{t} {b + 1}/{t} {c + 1}/{t}" for a, b, c in faces]
        path.write_text("\n".join(lines) + "\n")
        return sum(len(f) for f, _ in self.faces)


class Palette:
    def __init__(self):
        self.colors = []

    def add(self, rgb):
        rgb = tuple(round(c, 4) for c in rgb)
        if rgb not in self.colors:
            self.colors.append(rgb)
        return rgb

    def index(self, rgb):
        return self.colors.index(tuple(round(c, 4) for c in rgb))

    def shape(self):
        rows = math.ceil(len(self.colors) / PALETTE_COLUMNS)
        return rows * PALETTE_BLOCK, PALETTE_COLUMNS * PALETTE_BLOCK

    def texcoords(self):
        height, width = self.shape()
        coords = []
        for i in range(len(self.colors)):
            row, col = divmod(i, PALETTE_COLUMNS)
            u = (col + 0.5) * PALETTE_BLOCK / width
            v = 1.0 - (row + 0.5) * PALETTE_BLOCK / height
            coords.append((u, v))
        return coords

    def write_png(self, path):
        height, width = self.shape()
        image = np.zeros((height, width, 3), dtype=np.uint8)
        for i, rgb in enumerate(self.colors):
            row, col = divmod(i, PALETTE_COLUMNS)
            block = (np.array(rgb) * 255).round().astype(np.uint8)
            image[
                row * PALETTE_BLOCK : (row + 1) * PALETTE_BLOCK,
                col * PALETTE_BLOCK : (col + 1) * PALETTE_BLOCK,
            ] = block
        write_png(path, image)


def write_png(path, image):
    """Write an 8-bit greyscale (2D) or RGB (3D) image as PNG, with no imaging library."""
    height, width = image.shape[:2]
    color_type = 2 if image.ndim == 3 else 0
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    path.write_bytes(png)


def load_shelf():
    materials = {}
    meshes = {}
    root = ET.parse(ASSET_DIR / "shelf" / "supermarket_shelf_assets.xml").getroot()
    for mat in root.iter("material"):
        materials[mat.get("name")] = [float(c) for c in mat.get("rgba").split()][:3]
    for mesh in root.iter("mesh"):
        meshes[mesh.get("name")] = (
            ASSET_DIR / "shelf" / "meshes" / Path(mesh.get("file")).name
        )
    body = (
        ET.parse(ASSET_DIR / "shelf" / "supermarket_shelf_body.xml")
        .getroot()
        .find("body")
    )
    visuals = [
        (meshes[g.get("mesh")], materials[g.get("material")])
        for g in body.iter("geom")
        if g.get("type") == "mesh"
    ]
    collisions = [g for g in body.iter("geom") if g.get("type") == "box"]
    geometry = json.loads((ASSET_DIR / "shelf" / "shelf_geometry.json").read_text())
    full = [load_obj(path) + (rgb,) for path, rgb in visuals]
    low = []
    for geom in collisions:
        v, f = box_mesh([float(c) for c in geom.get("size").split()])
        v = v + np.array([float(c) for c in geom.get("pos").split()])
        kind = next(k for k in LOW_SHELF_BOX_MATERIALS if k in geom.get("name"))
        low.append((v, f, materials[LOW_SHELF_BOX_MATERIALS[kind]]))
    return {"full": full, "low": low}, collisions, geometry


def gondola_parts(parts, back_depth):
    """Two copies of a unit, back to back: one facing -Y of the gondola frame, one facing +Y."""
    front = [(v + np.array([0.0, -back_depth, 0.0]), f, rgb) for v, f, rgb in parts]
    turned = np.array([-1.0, -1.0, 1.0])
    back = [
        (v * turned + np.array([0.0, back_depth, 0.0]), f, rgb) for v, f, rgb in parts
    ]
    return front + back


def mesh_builder(palette, parts):
    builder = MeshBuilder(palette)
    for v, f, rgb in parts:
        builder.add(v, f, rgb)
    return builder


# ---------------------------------------------------------------- world


# Gondola frame yaw; its two units face -Y and +Y of that frame.
AXIS_YAW = {"ns": 0.0, "we": -math.pi / 2}


def gondola_pose(layout, gondola):
    x, y = layout.to_world(*gondola["anchor"])
    return (x, y, 0.0), AXIS_YAW[gondola["axis"]]


def unit_poses(layout, gondola, back_depth):
    """World poses of a gondola's two unit frames: on the floor, at the front, +Y into the shelf."""
    pose, yaw = gondola_pose(layout, gondola)
    return [
        (transform(pose, yaw, (0.0, -back_depth, 0.0)), yaw),
        (transform(pose, yaw, (0.0, back_depth, 0.0)), yaw + math.pi),
    ]


def transform(pose, yaw, local):
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = local
    return (pose[0] + c * x - s * y, pose[1] + s * x + c * y, pose[2] + z)


def build_world(layout, shelf):
    """Return the worldbody XML lines and the map footprints."""
    _, _, geometry = shelf
    overall = geometry["overall"]
    gondola_half_depth = overall["y"][1] - overall["y"][0]
    assert abs(2 * gondola_half_depth - GONDOLA_DEPTH) < 1e-6
    half_x = (overall["x"][1] - overall["x"][0]) / 2
    half_z = (overall["z"][1] - overall["z"][0]) / 2
    nodes, footprints = [], []

    for gondola in layout.gondolas:
        pose, yaw = gondola_pose(layout, gondola)
        footprints.append(
            [
                transform(pose, yaw, (sx * half_x, sy * gondola_half_depth, 0.0))
                for sx in (-1, 1)
                for sy in (-1, 1)
            ]
        )
        visual = element(
            "geom",
            [
                ("type", "mesh"),
                ("mesh", "market_pair_empty"),
                ("material", "market_palette"),
                ("contype", "0"),
                ("conaffinity", "0"),
                ("group", "2"),
            ],
        )
        box = element(
            "geom",
            [
                ("type", "box"),
                ("pos", fmt((0.0, 0.0, half_z))),
                ("size", fmt((half_x, gondola_half_depth, half_z))),
                ("group", "3"),
            ],
        )
        attrs = [("name", gondola["name"]), ("pos", fmt(pose))]
        attrs.append(("quat", fmt(yaw_quat(yaw))))
        nodes.append(element("body", attrs, [visual, box]))

    for register in layout.registers:
        cx = register["x"]
        cy = register["y_top"] + register["y_span"] / 2
        wx, wy = layout.to_world(cx, cy)
        hx, hy, hz = REGISTER_HALF_SIZE
        nodes.append(
            element(
                "geom",
                [
                    ("name", register["name"]),
                    ("type", "box"),
                    ("pos", fmt((wx, wy, hz))),
                    ("size", fmt(REGISTER_HALF_SIZE)),
                    ("material", "market_steel"),
                    ("group", "2"),
                ],
            )
        )
        footprints.append(
            [(wx + sx * hx, wy + sy * hy, 0) for sx in (-1, 1) for sy in (-1, 1)]
        )

    t = WALL_THICKNESS
    first, last = layout.bottom_wall_columns
    walls = {
        "wall_north": ((-t, -t), (layout.width + t, 0.0)),
        "wall_west": ((-t, -t), (0.0, layout.depth + t)),
        "wall_east": ((layout.width, -t), (layout.width + t, layout.depth + t)),
        "wall_south": (
            ((first - 1) * COLUMN_PITCH, layout.depth),
            (last * COLUMN_PITCH, layout.depth + t),
        ),
    }
    for name, ((x0, y0), (x1, y1)) in walls.items():
        (wx0, wy0), (wx1, wy1) = layout.to_world(x0, y0), layout.to_world(x1, y1)
        cx, cy = (wx0 + wx1) / 2, (wy0 + wy1) / 2
        hx, hy = abs(wx1 - wx0) / 2, abs(wy1 - wy0) / 2
        nodes.append(
            element(
                "geom",
                [
                    ("name", name),
                    ("type", "box"),
                    ("pos", fmt((cx, cy, WALL_HEIGHT / 2))),
                    ("size", fmt((hx, hy, WALL_HEIGHT / 2))),
                    ("material", "market_wall"),
                    ("group", "2"),
                ],
            )
        )
        footprints.append(
            [(cx + sx * hx, cy + sy * hy, 0) for sx in (-1, 1) for sy in (-1, 1)]
        )
    return nodes, footprints


def localization_footprints(layout, shelf, footprints):
    """What the lidars see at their height: the shelf boxes crossing it, the registers, the walls.

    The scan plane passes over the bottom deck to the back panel, so the full gondola footprint
    that Nav2 keeps the base out of would give AMCL a deck's depth of slack.
    """
    _, collisions, geometry = shelf
    back_depth = geometry["overall"]["y"][1]
    boxes = []
    for geom in collisions:
        pos = [float(v) for v in geom.get("pos").split()]
        half = [float(v) for v in geom.get("size").split()]
        if pos[2] - half[2] <= LIDAR_HEIGHT <= pos[2] + half[2]:
            boxes.append((pos, half))
    seen = []
    for gondola in layout.gondolas:
        for pose, yaw in unit_poses(layout, gondola, back_depth):
            for pos, half in boxes:
                seen.append(
                    [
                        transform(
                            pose,
                            yaw,
                            (pos[0] + sx * half[0], pos[1] + sy * half[1], 0.0),
                        )
                        for sx in (-1, 1)
                        for sy in (-1, 1)
                    ]
                )
    # build_world lists the gondolas first, then the registers and walls.
    return seen + footprints[len(layout.gondolas) :]


def keyframes():
    """The default key: the robot's default pose, at the start."""
    attrs = [("name", "default"), ("qpos", fmt(ROBOT_QPOS)), ("ctrl", fmt(ROBOT_CTRL))]
    return [element("key", attrs)]


# ---------------------------------------------------------------- objectives


def aisle_goals(layout, back_depth):
    """A base pose facing the middle of each real aisle, in the map frame.

    The map frame equals the world frame. E gondolas are worked from their west side, the
    others from their south side.
    """
    goals = {}
    for aisle in REAL_AISLES:
        if aisle in AD_REAL:
            middle = [AD_GOAL_COLUMNS[aisle]]
        else:
            count = sum(g["aisle"] == aisle for g in layout.gondolas)
            middle = [(count + 1) // 2] if count % 2 else [count // 2, count // 2 + 1]
        poses = [
            unit_poses(layout, g, back_depth)[0]
            for g in layout.gondolas
            if g["aisle"] == aisle and g["index"] in middle
        ]
        x = sum(p[0][0] for p in poses) / len(poses)
        y = sum(p[0][1] for p in poses) / len(poses)
        # The robot's +X points into the shelf, along the unit's +Y.
        yaw = poses[0][1] + math.pi / 2
        goals[aisle] = (
            x - AISLE_STANDOFF * math.cos(yaw),
            y - AISLE_STANDOFF * math.sin(yaw),
            yaw,
        )
    return goals


def write_aisle_objective(path, aisle, goal):
    x, y, yaw = goal
    name = f"Navigate to Aisle {aisle}"
    description = (
        f"Drive the base with Nav2 to the middle of aisle {aisle}, facing its shelves, "
        "with the arms stowed."
    )
    tree = element(
        "BehaviorTree",
        [("ID", name), ("_description", description), ("_favorite", "false")],
        [
            element(
                "Control",
                [("ID", "Sequence"), ("name", "TopLevelSequence")],
                [
                    element(
                        "Action",
                        [
                            ("ID", "CreatePoseStamped"),
                            ("reference_frame", "map"),
                            ("position_xyz", f"{x:.3f};{y:.3f};0"),
                            (
                                "orientation_xyzw",
                                f"0;0;{math.sin(yaw / 2):.6f};{math.cos(yaw / 2):.6f}",
                            ),
                            ("pose_stamped", "{goal}"),
                        ],
                    ),
                    # Stows the arms, switches the base controller and drives with Nav2.
                    element(
                        "SubTree",
                        [
                            ("ID", "Drive Base using Nav2"),
                            ("_collapsed", "true"),
                            ("goal_pose", "{goal}"),
                        ],
                    ),
                ],
            )
        ],
    )
    model = element(
        "TreeNodesModel",
        children=[
            element(
                "SubTree",
                [("ID", name)],
                [
                    element(
                        "MetadataFields",
                        children=[
                            element("Metadata", [("runnable", "true")]),
                            element("Metadata", [("subcategory", "Navigation")]),
                        ],
                    )
                ],
            )
        ],
    )
    root = element(
        "root", [("BTCPP_format", "4"), ("main_tree_to_execute", name)], [tree, model]
    )
    lines = ['<?xml version="1.0" encoding="UTF-8" ?>'] + render(root)
    lines.insert(2, f"  <!-- {GENERATED} -->")
    text = "\n".join(lines) + "\n"
    ET.fromstring(text.split("\n", 1)[1])
    path.write_text(text)


# ---------------------------------------------------------------- map


def write_map(footprints, map_out, name="market"):
    points = np.array([p[:2] for fp in footprints for p in fp])
    x_min, y_min = points.min(axis=0) - MAP_MARGIN
    x_max, y_max = points.max(axis=0) + MAP_MARGIN
    width = int(math.ceil((x_max - x_min) / MAP_RESOLUTION))
    height = int(math.ceil((y_max - y_min) / MAP_RESOLUTION))
    grid = np.full((height, width), 254, dtype=np.uint8)
    xs = x_min + (np.arange(width) + 0.5) * MAP_RESOLUTION
    ys = y_min + (np.arange(height) + 0.5) * MAP_RESOLUTION
    gx, gy = np.meshgrid(xs, ys)
    # Every cell a footprint overlaps is occupied, so Nav2's footprint check never sees a rack
    # edge inside a free cell.
    half = MAP_RESOLUTION / 2
    for fp in footprints:
        corners = np.array([p[:2] for p in fp])
        lo, hi = corners.min(axis=0) - half, corners.max(axis=0) + half
        grid[(gx > lo[0]) & (gx < hi[0]) & (gy > lo[1]) & (gy < hi[1])] = 0
    # Image row 0 is the top of the picture, which is the map's largest y.
    write_png(map_out / f"{name}.png", grid[::-1])
    (map_out / f"{name}.yaml").write_text(
        "# GENERATED by scripts/generate_market.py. Do not edit.\n"
        f"image: {name}.png\n"
        "mode: trinary\n"
        f"resolution: {MAP_RESOLUTION}\n"
        f"origin: [{x_min:.3f}, {y_min:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    return width, height


# ---------------------------------------------------------------- main


PRINT_WIDTH = 80


def element(tag, attrs=(), children=()):
    return (tag, list(attrs), list(children))


def render(node, indent=0):
    """Print one element the way the repository's prettier XML hook does, so it stays stable."""
    tag, attrs, children = node
    pad = " " * indent
    head = " ".join([f"<{tag}"] + [f'{k}="{v}"' for k, v in attrs])
    close = " />" if not children else ">"
    if len(pad + head + close) <= PRINT_WIDTH or not attrs:
        lines = [pad + head + close]
    else:
        lines = [f"{pad}<{tag}"] + [f'{pad}  {k}="{v}"' for k, v in attrs]
        lines.append(pad + ("/>" if not children else ">"))
    if children:
        for child in children:
            lines += render(child, indent + 2)
        lines.append(f"{pad}</{tag}>")
    return lines


def write_xml(path, nodes):
    lines = [f"<!-- {GENERATED} -->", "<mujocoinclude>"]
    for node in nodes:
        lines += render(node, 2)
    text = "\n".join(lines + ["</mujocoinclude>", ""])
    ET.fromstring(text)
    path.write_text(text)


def main(output_root=PACKAGE_DIR):
    """Write every generated file under output_root, which mirrors the package layout."""
    mjcf_out = Path(output_root) / MJCF_OUT
    mesh_out = Path(output_root) / MESH_OUT
    map_out = Path(output_root) / MAP_OUT
    objectives_out = Path(output_root) / OBJECTIVES_OUT
    layout = Layout(FLOOR_PLAN)
    shelf = load_shelf()
    for folder in (mjcf_out, mesh_out, map_out, objectives_out):
        folder.mkdir(parents=True, exist_ok=True)

    back_depth = shelf[2]["overall"]["y"][1]
    palette = Palette()
    pair_empty = mesh_builder(palette, gondola_parts(shelf[0]["low"], back_depth))
    faces = pair_empty.write(mesh_out / "pair_empty.obj", GENERATED)
    palette.write_png(mesh_out / "palette.png")

    world, footprints = build_world(layout, shelf)
    write_xml(mjcf_out / "market_world.xml", world)

    shiny = [("specular", "0.3"), ("shininess", "0.4")]
    assets = [
        element(
            "texture",
            [
                ("name", "market_palette"),
                ("type", "2d"),
                ("file", "assets/market/palette.png"),
            ],
        ),
        element(
            "material",
            [("name", "market_palette"), ("texture", "market_palette")] + shiny,
        ),
        element(
            "material",
            [
                ("name", "market_steel"),
                ("rgba", "0.62 0.64 0.67 1"),
                ("specular", "0.6"),
                ("shininess", "0.7"),
            ],
        ),
        element("material", [("name", "market_wall"), ("rgba", "0.86 0.85 0.8 1")]),
        element(
            "mesh",
            [("name", "market_pair_empty"), ("file", "market/pair_empty.obj")],
        ),
    ]
    write_xml(mjcf_out / "market_assets.xml", [element("asset", children=assets)])
    write_xml(mjcf_out / "keyframes.xml", [element("keyframe", children=keyframes())])

    width, height = write_map(footprints, map_out)
    seen = localization_footprints(layout, shelf, footprints)
    write_map(seen, map_out, "market_localization")
    for aisle, goal in aisle_goals(layout, back_depth).items():
        path = objectives_out / f"navigate_to_aisle_{aisle.lower()}.xml"
        write_aisle_objective(path, aisle, goal)
    print(f"gondolas: {len(layout.gondolas)}, registers: {len(layout.registers)}")
    print(f"mesh faces: {faces}, palette colours: {len(palette.colors)}")
    print(f"map: {width} x {height} cells")
    print(
        f"store: {layout.width:.2f} m x {layout.depth:.2f} m, start at plan {layout.origin}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--output-root",
        default=PACKAGE_DIR,
        help="Folder to write into, laid out like the package (default: the package).",
    )
    main(parser.parse_args().output_root)
