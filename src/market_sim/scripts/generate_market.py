#!/usr/bin/env python3
"""Generate the market_sim supermarket from its text floor plan.

Reads FLOOR_PLAN below and the shelf and product assets in ``market_assets/``, and
writes the MuJoCo scene parts (``mjcf/market/``), their meshes and texture
(``mjcf/assets/market/``) and the Nav2 map (``maps/market.{pgm,yaml}``).

Run from anywhere, with numpy installed:

    python3 src/market_sim/scripts/generate_market.py

See the package README for the layout rules and the loose-product budget.
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

# The captain's floor plan, verbatim. '//' lines are labels and take no floor space.
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
# Aisles whose products are loose. The 24 loose shelf units fill every E and I gondola
# here in loose_e_i, and move to the A to D gondolas below in loose_a_d.
EI_REAL = ("E1", "E5", "I2")
AD_REAL = ("A1", "B2", "C3", "D4")
REAL_AISLES = EI_REAL + AD_REAL
# Gondola columns, from each row's west end, that take the loose shelf units in loose_a_d.
# The gondolas on both sides of each one are empty in every keyframe.
AD_LOOSE_COLUMNS = {
    "A1": (4, 6, 8),
    "B2": (9, 11, 13),
    "C3": (4, 6, 8),
    "D4": (2, 4, 6),
}
# Keyframe name and where the loose shelf units are; the scene file holds the "a_d" poses.
KEYFRAMES = (("default", "a_d"), ("loose_e_i", "e_i"), ("loose_a_d", "a_d"))

WALL_THICKNESS = 0.2
WALL_HEIGHT = 2.5
REGISTER_HALF_SIZE = (0.5, 1.3, 0.45)
# The robot boots here, on the open floor south of row 1; world and map share this origin.
START_LINE_INDEX = 19

# Stocked items that are loose on every loose shelf unit; a stack is loose or fixed as a whole.
LOOSE_ITEMS = (
    "cracker_box_1",
    "cracker_box_3",
    "cracker_box_5",
    "sugar_box_1",
    "sugar_box_3",
    "sugar_box_5",
    "gelatin_box_1",
    "gelatin_box_2",
    "pudding_box_1",
    "pudding_box_2",
    "meat_can_1",
    "meat_can_2",
    "soup_can_1",
    "soup_can_3",
    "coffee_can_1",
    "coffee_can_3",
)

# Grippy enough for the simulated Franka Hand's light squeeze to hold a product, as in lab_sim.
LOOSE_FRICTION = "2.0 0.1 0.01"

# Robot part of every keyframe: the Mobile FR3 Duo's default pose.
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
NAV_TO_POSE_TREE = (
    "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
    "navigate_to_pose_w_replanning_and_recovery.xml"
)

# Solid gondolas draw the shelf from its collision boxes only, to cut triangles.
LOW_SHELF_BOX_MATERIALS = {
    "upright": "shelf_frame_steel",
    "bracket": "shelf_frame_steel",
    "channel": "shelf_price_channel",
    "back": "shelf_panel_steel",
    "shelf_base": "shelf_panel_steel",
    "_collision": "shelf_panel_steel",
}

# Items resting inside another item, which a front view never shows.
HIDDEN_SUPPORTS = ("detergent_case_1",)

# Visual-only can lids, left out of the merged shelf meshes to save triangles.
LID_MATERIALS = ("product_metal_lid",)

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


def cylinder_mesh(radius, half_height, segments=8):
    angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
    bottom = np.hstack([ring, np.full((segments, 1), -half_height)])
    top = np.hstack([ring, np.full((segments, 1), half_height)])
    v = np.vstack([bottom, top, [[0, 0, -half_height], [0, 0, half_height]]])
    f = []
    for i in range(segments):
        j = (i + 1) % segments
        f += [[i, j, segments + j], [i, segments + j, segments + i]]
        f += [[2 * segments, j, i], [2 * segments + 1, segments + i, segments + j]]
    return v, np.array(f)


def icosphere(subdivisions=1):
    t = (1 + math.sqrt(5)) / 2
    v = [
        [-1, t, 0],
        [1, t, 0],
        [-1, -t, 0],
        [1, -t, 0],
        [0, -1, t],
        [0, 1, t],
        [0, -1, -t],
        [0, 1, -t],
        [t, 0, -1],
        [t, 0, 1],
        [-t, 0, -1],
        [-t, 0, 1],
    ]
    f = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ]
    v = [np.array(p) / np.linalg.norm(p) for p in v]
    for _ in range(subdivisions):
        cache, new_faces = {}, []

        def midpoint(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = (v[a] + v[b]) / 2
                v.append(m / np.linalg.norm(m))
                cache[key] = len(v) - 1
            return cache[key]

        for a, b, c in f:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            new_faces += [[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]
        f = new_faces
    return np.array(v), np.array(f)


def primitive_mesh(geom, carrot, low=False):
    """Triangles for one MJCF visual geom, in its body frame; ``low`` uses the coarsest shapes."""
    kind = geom.get("type", "sphere")
    size = [float(s) for s in geom.get("size", "0").split()]
    round_shape = icosphere(0 if low else 1)
    if kind == "box":
        v, f = box_mesh(size)
    elif kind == "cylinder":
        v, f = cylinder_mesh(size[0], size[1], 6 if low else 8)
    elif kind == "sphere":
        v, f = round_shape
        v = v * size[0]
    elif kind == "ellipsoid":
        v, f = round_shape
        v = v * np.array(size)
    elif kind == "capsule":
        v, f = round_shape
        v = v * np.array([size[0], size[0], size[1] + size[0]])
    elif kind == "mesh" and low:
        # The carrot's own frame is its bounding-box centre, long along X.
        extent = carrot[0].max(axis=0) - carrot[0].min(axis=0)
        v, f = round_shape
        v = v * extent / 2
    elif kind == "mesh":
        v, f = carrot
    else:
        raise ValueError(f"unsupported geom type {kind}")
    rotation = quat_to_matrix([float(q) for q in geom.get("quat", "1 0 0 0").split()])
    position = np.array([float(p) for p in geom.get("pos", "0 0 0").split()])
    return v @ rotation.T + position, f


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


def load_products():
    catalog = json.loads((ASSET_DIR / "products" / "product_catalog.json").read_text())
    materials = {}
    for mat in ET.parse(ASSET_DIR / "products" / "products_assets.xml").iter(
        "material"
    ):
        materials[mat.get("name")] = [float(c) for c in mat.get("rgba").split()][:3]
    bodies = {}
    for path in sorted((ASSET_DIR / "products" / "bodies").glob("*.xml")):
        bodies[path.stem] = ET.parse(path).getroot().find("body")
    return catalog, materials, bodies


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


def hidden_from_aisle(stocked, catalog):
    """Stocked items that a front view cannot see: behind a front-row item, or inside the case."""
    hidden = set()
    for entry in stocked:
        if entry["support"] in HIDDEN_SUPPORTS:
            hidden.add(entry["body"])
            continue
        size = catalog["items"][entry["item"]]["size"]
        half_x = abs(quat_to_matrix(entry["quat"]) @ np.array(size) / 2)[0]
        for other in stocked:
            same_shelf = other["support"] == entry["support"]
            in_front = other["pos"][1] < entry["pos"][1] - 0.01
            covers = abs(other["pos"][0] - entry["pos"][0]) < half_x
            level = abs(other["pos"][2] - entry["pos"][2]) < 0.01
            if same_shelf and in_front and covers and level:
                hidden.add(entry["body"])
                break
    return hidden


def unit_parts(shelf_parts, stocked, products, skip, carrot, low):
    """One stocked shelf unit as (vertices, faces, colour) parts; ``low`` cuts its triangles."""
    catalog, materials, bodies = products
    if low:
        skip = set(skip) | hidden_from_aisle(stocked, catalog)
    parts = list(shelf_parts)
    for entry in stocked:
        if entry["body"] in skip:
            continue
        body = bodies[f"{entry['item']}_v{entry['variant']}"]
        rotation = quat_to_matrix(entry["quat"])
        position = np.array(entry["pos"])
        for geom in body.findall("geom"):
            if geom.get("group") != "2" or geom.get("material") in LID_MATERIALS:
                continue
            v, f = primitive_mesh(geom, carrot, True)
            parts.append(
                (v @ rotation.T + position, f, materials[geom.get("material")])
            )
    return parts


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


def loose_spots(layout):
    """The gondolas the loose shelf units fill in each arrangement, and the empty A to D ones."""
    by_key = {(g["aisle"], g["index"]): g for g in layout.gondolas}
    spots = {"e_i": [by_key[k] for k in sorted(by_key) if k[0] in EI_REAL], "a_d": []}
    empty = set()
    for aisle in AD_REAL:
        count = max(k[1] for k in by_key if k[0] == aisle)
        columns = AD_LOOSE_COLUMNS[aisle]
        for column in columns:
            # Not at a row end, and never next to another loose gondola.
            assert 1 < column < count and column + 1 not in columns, (aisle, column)
            empty |= {(aisle, column - 1), (aisle, column + 1)}
            spots["a_d"].append(by_key[(aisle, column)])
    assert len(spots["e_i"]) == len(spots["a_d"])
    return spots, [by_key[k] for k in sorted(empty)]


def build_world(layout, shelf, products):
    """Return the worldbody XML lines, the loose items, the moving shelves and the map footprints."""
    _, collisions, geometry = shelf
    catalog, _, bodies = products
    overall = geometry["overall"]
    back_depth = overall["y"][1]
    gondola_half_depth = overall["y"][1] - overall["y"][0]
    assert abs(2 * gondola_half_depth - GONDOLA_DEPTH) < 1e-6
    half_x = (overall["x"][1] - overall["x"][0]) / 2
    half_z = (overall["z"][1] - overall["z"][0]) / 2
    stocked = {entry["body"]: entry for entry in catalog["stocked_scene"]}
    spots, empty = loose_spots(layout)
    moving = [g["name"] for g in spots["e_i"] + spots["a_d"]]
    empty_names = {g["name"] for g in empty}
    nodes, loose, footprints = [], [], []

    def visual(mesh):
        return element(
            "geom",
            [
                ("type", "mesh"),
                ("mesh", mesh),
                ("material", "market_palette"),
                ("contype", "0"),
                ("conaffinity", "0"),
                ("group", "2"),
            ],
        )

    def gondola_body(name, pose, yaw, mesh, mocap):
        box = element(
            "geom",
            [
                ("type", "box"),
                ("pos", fmt((0.0, 0.0, half_z))),
                ("size", fmt((half_x, gondola_half_depth, half_z))),
                ("group", "3"),
            ],
        )
        attrs = [("name", name)] + ([("mocap", "true")] if mocap else [])
        attrs += [("pos", fmt(pose)), ("quat", fmt(yaw_quat(yaw)))]
        return element("body", attrs, [visual(mesh), box])

    for gondola in layout.gondolas:
        pose, yaw = gondola_pose(layout, gondola)
        footprints.append(
            [
                transform(pose, yaw, (sx * half_x, sy * gondola_half_depth, 0.0))
                for sx in (-1, 1)
                for sy in (-1, 1)
            ]
        )
        if gondola["name"] in moving:
            continue
        mesh = (
            "market_pair_empty"
            if gondola["name"] in empty_names
            else "market_pair_full"
        )
        nodes.append(gondola_body(gondola["name"], pose, yaw, mesh, False))

    # Mocap bodies, so a keyframe can move them: the loose units first, then the solid ones.
    shelves = {"units": [], "gondolas": []}
    for a_d, e_i in zip(spots["a_d"], spots["e_i"]):
        units = zip(
            unit_poses(layout, a_d, back_depth), unit_poses(layout, e_i, back_depth)
        )
        for home, other in units:
            name = f"loose_shelf_{len(shelves['units']) + 1}"
            shelves["units"].append({"a_d": home, "e_i": other})
            geoms = [visual("market_unit_real")]
            for geom in collisions:
                geoms.append(
                    element(
                        "geom",
                        [
                            ("type", "box"),
                            ("pos", geom.get("pos")),
                            ("size", geom.get("size")),
                            ("group", "3"),
                        ],
                    )
                )
            attrs = [("name", name), ("mocap", "true")]
            attrs += [("pos", fmt(home[0])), ("quat", fmt(yaw_quat(home[1])))]
            nodes.append(element("body", attrs, geoms))
            for item in LOOSE_ITEMS:
                entry = stocked[item]
                poses = {
                    key: (
                        transform(pose, yaw, entry["pos"]),
                        quat_mul(yaw_quat(yaw), entry["quat"]),
                    )
                    for key, (pose, yaw) in (("a_d", home), ("e_i", other))
                }
                loose.append(
                    {
                        "name": f"{name}_{item}",
                        "body": bodies[f"{entry['item']}_v{entry['variant']}"],
                        "poses": poses,
                    }
                )
    for a_d, e_i in zip(spots["a_d"], spots["e_i"]):
        name = f"solid_pair_{len(shelves['gondolas']) + 1}"
        home, other = gondola_pose(layout, e_i), gondola_pose(layout, a_d)
        shelves["gondolas"].append({"e_i": other, "a_d": home})
        nodes.append(gondola_body(name, home[0], home[1], "market_pair_full", True))

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

    nodes += [loose_item_body(item) for item in loose]
    return nodes, loose, shelves, footprints


def loose_item_body(item):
    """A loose product: one geom that both collides and renders, so it costs one visible geom."""
    body = item["body"]
    inertial = body.find("inertial")
    collide = [g for g in body.findall("geom") if g.get("group") == "3"]
    assert len(collide) == 1, f"{item['name']}: expected one collision geom"
    geom = collide[0]
    attributes = [(key, geom.get(key)) for key in ("type", "size", "material")]
    attributes += [(key, geom.get(key)) for key in ("pos", "quat") if geom.get(key)]
    attributes.append(("friction", LOOSE_FRICTION))
    inertial_attributes = [(k, inertial.get(k)) for k in ("pos", "mass", "fullinertia")]
    position, quat = item["poses"]["a_d"]
    return element(
        "body",
        [("name", item["name"]), ("pos", fmt(position)), ("quat", fmt(quat))],
        [
            element("freejoint"),
            element("inertial", inertial_attributes),
            element("geom", attributes + [("group", "2")]),
        ],
    )


def keyframes(loose, shelves):
    """One key per arrangement: product poses in qpos, the moving shelves in mpos and mquat."""
    keys = []
    for name, arrangement in KEYFRAMES:
        qpos = list(ROBOT_QPOS)
        for item in loose:
            position, quat = item["poses"][arrangement]
            qpos += list(position) + list(quat)
        mpos, mquat = [], []
        for shelf in shelves["units"] + shelves["gondolas"]:
            position, yaw = shelf[arrangement]
            mpos += list(position)
            mquat += list(yaw_quat(yaw))
        attrs = [("name", name), ("qpos", fmt(qpos)), ("mpos", fmt(mpos))]
        attrs += [("mquat", fmt(mquat)), ("ctrl", fmt(ROBOT_CTRL))]
        keys.append(element("key", attrs))
    return keys


# ---------------------------------------------------------------- objectives


def aisle_goals(layout, back_depth):
    """A base pose facing the middle loose gondola of each real aisle, in the map frame.

    The map frame equals the world frame. E gondolas are worked from their west side, the
    others from their south side.
    """
    goals = {}
    for aisle in REAL_AISLES:
        if aisle in AD_REAL:
            middle = sorted(AD_LOOSE_COLUMNS[aisle])[1:2]
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
        f"Stow the arms, then drive the base with Nav2 to the middle of aisle {aisle}, "
        "facing its shelves, where the products are loose and can be picked."
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
                    # The Nav2 footprint covers the robot only with its arms stowed.
                    element(
                        "SubTree",
                        [("ID", "Stow Arms for Navigation"), ("_collapsed", "true")],
                    ),
                    # Nav2 drives the base through base_jgvc; a base jog leaves it inactive.
                    element(
                        "Action",
                        [
                            ("ID", "SwitchController"),
                            ("activate_controllers", "base_jgvc"),
                        ],
                    ),
                    element(
                        "Action",
                        [
                            ("ID", "NavigateToPoseAction"),
                            ("action_name", "/navigate_to_pose"),
                            ("behavior_tree_path", NAV_TO_POSE_TREE),
                            ("ignore_stamp_time", "true"),
                            ("pose_stamped", "{goal}"),
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


def write_map(footprints, map_out):
    points = np.array([p[:2] for fp in footprints for p in fp])
    x_min, y_min = points.min(axis=0) - MAP_MARGIN
    x_max, y_max = points.max(axis=0) + MAP_MARGIN
    width = int(math.ceil((x_max - x_min) / MAP_RESOLUTION))
    height = int(math.ceil((y_max - y_min) / MAP_RESOLUTION))
    grid = np.full((height, width), 254, dtype=np.uint8)
    xs = x_min + (np.arange(width) + 0.5) * MAP_RESOLUTION
    ys = y_min + (np.arange(height) + 0.5) * MAP_RESOLUTION
    gx, gy = np.meshgrid(xs, ys)
    for fp in footprints:
        corners = np.array([p[:2] for p in fp])
        lo, hi = corners.min(axis=0), corners.max(axis=0)
        grid[(gx >= lo[0]) & (gx <= hi[0]) & (gy >= lo[1]) & (gy <= hi[1])] = 0
    # Image row 0 is the top of the picture, which is the map's largest y.
    write_png(map_out / "market.png", grid[::-1])
    (map_out / "market.yaml").write_text(
        "# GENERATED by scripts/generate_market.py. Do not edit.\n"
        "image: market.png\n"
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
    products = load_products()
    carrot = load_obj(ASSET_DIR / "products" / "meshes" / "carrot.obj")
    for folder in (mjcf_out, mesh_out, map_out, objectives_out):
        folder.mkdir(parents=True, exist_ok=True)

    back_depth = shelf[2]["overall"]["y"][1]
    stocked = products[0]["stocked_scene"]
    palette = Palette()
    full = unit_parts(shelf[0]["low"], stocked, products, (), carrot, low=True)
    empty = unit_parts(shelf[0]["low"], [], products, (), carrot, low=True)
    real = unit_parts(
        shelf[0]["full"], stocked, products, LOOSE_ITEMS, carrot, low=False
    )
    meshes = {
        "pair_full": mesh_builder(palette, gondola_parts(full, back_depth)),
        "pair_empty": mesh_builder(palette, gondola_parts(empty, back_depth)),
        "unit_real": mesh_builder(palette, real),
    }
    faces = {
        name: b.write(mesh_out / f"{name}.obj", GENERATED) for name, b in meshes.items()
    }
    palette.write_png(mesh_out / "palette.png")

    world, loose, shelves, footprints = build_world(layout, shelf, products)
    write_xml(mjcf_out / "market_world.xml", world)

    used_materials = sorted(
        {item["body"].find("geom[@group='3']").get("material") for item in loose}
    )
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
    ]
    for name in meshes:
        assets.append(
            element(
                "mesh", [("name", f"market_{name}"), ("file", f"market/{name}.obj")]
            )
        )
    _, materials, _ = products
    for name in used_materials:
        rgba = f"{fmt(materials[name])} 1"
        assets.append(element("material", [("name", name), ("rgba", rgba)] + shiny))
    write_xml(mjcf_out / "market_assets.xml", [element("asset", children=assets)])
    write_xml(
        mjcf_out / "keyframes.xml",
        [element("keyframe", children=keyframes(loose, shelves))],
    )

    width, height = write_map(footprints, map_out)
    for aisle, goal in aisle_goals(layout, back_depth).items():
        path = objectives_out / f"navigate_to_aisle_{aisle.lower()}.xml"
        write_aisle_objective(path, aisle, goal)
    moving = len(shelves["units"]) + len(shelves["gondolas"])
    print(f"gondolas: {len(layout.gondolas)}, registers: {len(layout.registers)}")
    print(f"moving shelves: {moving}, loose items: {len(loose)}, mesh faces: {faces}")
    print(f"palette colours: {len(palette.colors)}, map: {width} x {height} cells")
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
