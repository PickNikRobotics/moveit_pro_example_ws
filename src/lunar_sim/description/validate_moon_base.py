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

"""Check base clearance and visibility, compare recorded state, render a label sheet.

Run after validate_and_render.py on the original and changed scenes, using the
same MuJoCo version for both. Requires MuJoCo, numpy, and Pillow, as that script does.
No scene assets or cameras are generated or modified.

  MUJOCO_GL=egl python3 validate_moon_base.py husky_scene.xml /out \
    --baseline /out/baseline_calibration.json \
    --calibration /out/moon_base_calibration.json
"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from validate_and_render import load


def world_bounds(model, data, geom_id):
    """Conservative world AABB for the compiled geom's local bounding box."""
    center, half = model.geom_aabb[geom_id].reshape(2, 3)
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    center = data.geom_xpos[geom_id] + rotation @ center
    half = np.abs(rotation) @ half
    return center - half, center + half


def _drivable_half_m():
    """The drivable square's half-extent: the outer edge of generate_band_terrain.py's strips around
    the centre (the access road through the north berm reaches further, but no base body stands on it).
    """
    here = Path(__file__).resolve().parent
    sizes = {
        h.get("name"): [float(v) for v in h.get("size").split()]
        for h in ET.parse(here / "husky_scene.xml").getroot().iter("hfield")
    }
    sides = {f"lunar_band_{side}" for side in "nsew"}
    edges = []
    for geom in ET.parse(here / "band_geoms_generated.xml").getroot().iter("geom"):
        if geom.get("name") not in sides:
            continue
        sides.discard(geom.get("name"))
        hx, hy = sizes[geom.get("hfield")][:2]
        x, y, _ = (float(v) for v in geom.get("pos").split())
        edges.append(max(abs(x) + hx, abs(y) + hy))
    if sides:
        raise SystemExit(f"band_geoms_generated.xml has no {sorted(sides)}")
    return max(edges)


DRIVABLE_HALF_M = _drivable_half_m()


def band_ground(model, data, x, y):
    """Top of the band's collision heightfields under (x, y)."""
    band = {
        i
        for i in range(model.ngeom)
        if model.geom(i).name.startswith("lunar_band_") and model.geom_contype[i]
    }
    groups = np.zeros(6, dtype=np.uint8)
    for i in band:
        groups[model.geom_group[i]] = 1
    hit = np.array([-1], dtype=np.int32)
    origin = np.array([x, y, 50.0])
    distance = mujoco.mj_ray(
        model, data, origin, np.array([0, 0, -1.0]), groups, True, -1, hit
    )
    assert distance > 0 and hit[0] in band, f"no band ground under ({x:.2f}, {y:.2f})"
    return origin[2] - distance


ROLL_OVER_M = 0.10  # under the A300's 0.136 m belly and 0.165 m wheel radius


def _front_scan_plane_m():
    """Front lidar height above the ground: chassis_link, lidar link and lidar camera z."""
    root = ET.parse(Path(__file__).resolve().parent / "husky_a300.xml").getroot()

    def z(element):
        return float((element.get("pos") or "0 0 0").split()[2])

    body = root.find(".//body[@name='lidar_front_link']")
    return (
        z(root.find(".//body[@name='chassis_link']"))
        + z(body)
        + z(body.find("camera[@name='lidar_front']"))
    )


def _inside(model, data, geom_id, points):
    """Which world points lie inside a primitive geom; anything else fails loudly."""
    local = (points - data.geom_xpos[geom_id]) @ data.geom_xmat[geom_id].reshape(3, 3)
    size = model.geom_size[geom_id]
    kind = model.geom_type[geom_id]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return np.all(np.abs(local) <= size, axis=1)
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return (np.hypot(local[:, 0], local[:, 1]) <= size[0]) & (
            np.abs(local[:, 2]) <= size[1]
        )
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        z = np.clip(local[:, 2], -size[1], size[1])
        return np.hypot(np.hypot(local[:, 0], local[:, 1]), local[:, 2] - z) <= size[0]
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return np.linalg.norm(local, axis=1) <= size[0]
    if kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        return np.sum((local / size) ** 2, axis=1) <= 1.0
    raise AssertionError(
        f"{model.geom(geom_id).name}: no inside test for geom type {kind}"
    )


LOW_PARTS_CELL_M = 0.02
LOW_PARTS_SEEN_RADIUS_M = 0.1  # how far around a seen cell the costmap's mark covers
# The base, the rocks and the collision terrain; group 2 holds the render-only terrain.
COLLISION_GROUPS = np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8)


def _rasterise_body(model, data, own, ground, plane):
    """Grid axes, which cells are solid at the scan plane, and each cell's highest solid point below it."""
    cell = LOW_PARTS_CELL_M
    bounds = [world_bounds(model, data, i) for i in own]
    # Padded past _dilate's reach, so its wrap-around never reaches a solid cell.
    pad = LOW_PARTS_SEEN_RADIUS_M + 2 * cell
    lo = np.min([b[0] for b in bounds], axis=0) - pad
    hi = np.max([b[1] for b in bounds], axis=0) + pad
    xs = np.arange(lo[0], hi[0], cell)
    ys = np.arange(lo[1], hi[1], cell)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    flat = np.c_[gx.ravel(), gy.ravel()]
    zs = np.arange(max(lo[2], ground - 0.3), plane, 0.01)
    seen = np.zeros(flat.shape[0], dtype=bool)
    top = np.full(flat.shape[0], -np.inf)
    for i, (glo, ghi) in zip(own, bounds):
        sel = np.nonzero(np.all((flat >= glo[:2]) & (flat <= ghi[:2]), axis=1))[0]
        pts = flat[sel]
        if glo[2] <= plane <= ghi[2]:
            seen[sel] |= _inside(model, data, i, np.c_[pts, np.full(len(pts), plane)])
        for z in zs[(zs >= glo[2]) & (zs <= ghi[2])]:
            hits = sel[_inside(model, data, i, np.c_[pts, np.full(len(pts), z)])]
            top[hits] = np.maximum(top[hits], z)
    return xs, ys, seen.reshape(gx.shape), top.reshape(gx.shape)


def _dilate(mask, radius_cells):
    """Cells within radius_cells of a set cell."""
    out = np.zeros_like(mask)
    offsets = range(-radius_cells, radius_cells + 1)
    for dx, dy in ((dx, dy) for dx in offsets for dy in offsets):
        if dx * dx + dy * dy <= radius_cells * radius_cells:
            out |= np.roll(np.roll(mask, dx, 0), dy, 1)
    return out


def _hidden_steps(model, data, body, xs, ys, top, hidden, plane):
    """(step, x, y) wherever a hidden solid cell meets open ground and stands over ROLL_OVER_M above it."""
    solid = np.isfinite(top)
    found = []
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        edge = np.roll(np.roll(hidden, dx, 0), dy, 1) & ~solid
        for ix, iy in zip(*np.nonzero(edge)):
            hit = np.array([-1], dtype=np.int32)
            origin = np.array([xs[ix], ys[iy], plane])
            distance = mujoco.mj_ray(
                model,
                data,
                origin,
                np.array([0, 0, -1.0]),
                COLLISION_GROUPS,
                True,
                body,
                hit,
            )
            assert (
                distance >= 0
            ), f"{model.body(body).name}: no ground beside it at ({xs[ix]:.2f}, {ys[iy]:.2f})"
            step = top[ix - dx, iy - dy] - (origin[2] - distance)
            if step > ROLL_OVER_M + 1e-3:
                found.append((step, xs[ix], ys[iy]))
    return found


def check_low_parts(model, data, ids):
    """A base part the front lidar cannot see, reaching past the parts it can, is low enough to roll
    over: otherwise the rover drives onto it unseen and stalls or tips. Each body is rasterised on a
    2 cm grid: a cell is seen where the body is solid at the front plane, and every solid cell below
    the plane further than 0.1 m from a seen cell must stand at most ROLL_OVER_M above the surface
    beside it (the terrain, or what the body stands on)."""
    front = _front_scan_plane_m()
    steps = {}  # body -> [(step, x, y)]
    for body in {int(model.geom_bodyid[i]) for i in ids}:
        own = [i for i in ids if model.geom_bodyid[i] == body]
        ground = (
            data.xpos[body][2] + 0.04
        )  # each datum sits 4 cm below its ground sample
        plane = ground + front
        xs, ys, seen, top = _rasterise_body(model, data, own, ground, plane)
        # Cells within 0.1 m of a seen cell are covered by the costmap's mark there.
        hidden = np.isfinite(top) & ~_dilate(
            seen, int(round(LOW_PARTS_SEEN_RADIUS_M / LOW_PARTS_CELL_M))
        )
        found = _hidden_steps(model, data, body, xs, ys, top, hidden, plane)
        if found:
            steps[model.body(body).name] = found
    assert not steps, (
        "parts below the front scan plane, past what it can see, too tall to roll over: "
        + "; ".join(
            f"{name} {max(found)[0]:.3f} m at ({max(found)[1]:.2f}, {max(found)[2]:.2f}), {len(found)} edge cells"
            for name, found in sorted(steps.items())
        )
    )


def check_base(model, data, scene, baseline):
    ids = [i for i in range(model.ngeom) if model.geom(i).name.startswith("moon_base_")]
    assert ids, "no moon base geoms loaded"
    bodies = {int(model.geom_bodyid[i]) for i in ids}
    assert len(bodies) == 10, f"expected ten fixed bodies, got {len(bodies)}"
    for body in bodies:
        assert model.body_jntnum[body] == 0, model.body(body).name
        assert model.body_parentid[body] == 0, model.body(body).name
    for i in ids:
        assert model.geom_group[i] in (0, 1, 2), model.geom(i).name
        assert model.geom_contype[i] and model.geom_conaffinity[i], model.geom(i).name

    corners = np.array(baseline["corners_xy"])
    far_corner = corners[2]
    # A 0.6 m disc contains the rover footprint, including bumpers and wheels.
    rover_radius = 0.6
    route_low = corners.min(axis=0) - rover_radius
    route_high = corners.max(axis=0) + rover_radius
    clearances = []
    for i in ids:
        low, high = world_bounds(model, data, i)
        gap = np.maximum(np.maximum(low[:2] - far_corner, far_corner - high[:2]), 0)
        clearances.append(float(np.linalg.norm(gap) - rover_radius))
        assert np.any(high[:2] < route_low) or np.any(
            low[:2] > route_high
        ), f"{model.geom(i).name} overlaps the recorded route's footprint envelope"
        assert np.all(low[:2] >= -DRIVABLE_HALF_M) and np.all(
            high[:2] <= DRIVABLE_HALF_M
        ), f"{model.geom(i).name} extends beyond the drivable terrain"
    minimum = min(clearances)
    assert minimum >= 2, f"far-corner driveable clearance is only {minimum:.3f} m"

    root = ET.parse(scene).getroot()
    hfield = root.find("asset/hfield[@name='lunar_hfield']")
    half_x, half_y, scale, _ = map(float, hfield.get("size").split())
    ground = root.find("worldbody/geom[@name='ground_plane']")
    offset = float(ground.get("pos").split()[2])
    meshdir = root.find("compiler").get("meshdir", "")
    with Image.open(scene.parent / meshdir / hfield.get("file")) as image:
        heights = np.asarray(image)[::-1, :].T / 255.0 * scale + offset
    for body in bodies:
        x, y, z = data.xpos[body]
        if max(abs(x), abs(y)) < half_x:
            # The centre's bodies keep their recorded nearest-cell datums.
            ix = round((x + half_x) / (2 * half_x) * heights.shape[0])
            iy = round((y + half_y) / (2 * half_y) * heights.shape[1])
            expected = heights[ix, iy] - 0.04
            tolerance = 1e-6
        else:
            # Band bodies sit 4 cm below the band's bilinear ground under their origin.
            expected = band_ground(model, data, x, y) - 0.04
            tolerance = 2e-3
        assert (
            abs(z - expected) < tolerance
        ), f"{model.body(body).name} datum differs from the committed terrain sample"

    check_low_parts(model, data, ids)

    # A visible-group ray from a future rover sensor must hit real base collision
    # geometry. Aim at elevated parts so the terrain and the rover do not mask it.
    origin = np.array([-0.30, 0.50, 1.2])
    groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    visible = []
    for name in (
        "moon_base_door",
        "moon_base_vertical_solar_panel",
        "moon_base_lander_payload",
        "moon_base_garage_wall_right",
    ):
        target_id = model.geom(name).id
        direction = data.geom_xpos[target_id] - origin
        direction /= np.linalg.norm(direction)
        hit = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, origin, direction, groups, True, -1, hit)
        assert (
            distance > 0 and model.geom_bodyid[hit[0]] == model.geom_bodyid[target_id]
        ), f"{name} is occluded from the start-corner sensor position"
        visible.append(model.geom(int(hit[0])).name)
    return {
        "fixed_bodies": len(bodies),
        "visible_collision_geoms": len(ids),
        "far_corner_clearance_beyond_rover_m": minimum,
        "start_corner_ray_hits": visible,
    }


def font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def render_sheet(model, data, out):
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0
    model.vis.global_.fovy = 75
    views = [
        (
            "Chase view | looking north across the rover toward the habitat and lander",
            [0, -4.5, 6],
            [0, 5.5, 1.0],
        ),
        (
            "Outer band | looking south from the centre toward the logistics module, mesa and garage",
            [0, 2.0, 4.5],
            [3.0, -11.0, 0.5],
        ),
    ]
    landmarks = [
        [
            ("1", "moon_base_habitat_shell"),
            ("2", "moon_base_vertical_solar_panel"),
            ("3", "moon_base_lander_payload"),
            ("4", "moon_base_cargo_c"),
        ],
        [
            ("5", "moon_base_logistics_module"),
            ("6", "landmark_mesa"),
            ("7", "moon_base_garage_roof_4"),
        ],
    ]
    sheet = Image.new("RGB", (1280, 1640), "#15191e")
    draw = ImageDraw.Draw(sheet)
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        for index, (title, eye, target) in enumerate(views):
            camera = mujoco.MjvCamera()
            delta = np.array(target) - eye
            camera.lookat[:] = target
            camera.distance = np.linalg.norm(delta)
            camera.azimuth = math.degrees(math.atan2(delta[1], delta[0]))
            camera.elevation = math.degrees(math.asin(delta[2] / camera.distance))
            renderer.update_scene(data, camera=camera, scene_option=option)
            frame = Image.fromarray(renderer.render())
            frame_draw = ImageDraw.Draw(frame)
            glcam = renderer.scene.camera[0]
            eye_actual = np.mean([c.pos for c in renderer.scene.camera], axis=0)
            forward, up = np.array(glcam.forward), np.array(glcam.up)
            right = np.cross(forward, up)
            focal = 360 / math.tan(math.radians(model.vis.global_.fovy) / 2)
            for label, name in landmarks[index]:
                offset = data.geom_xpos[model.geom(name).id] - eye_actual
                depth = offset @ forward
                assert depth > 0, f"{name} behind evidence camera"
                x = 640 + focal * (offset @ right) / depth
                y = 360 - focal * (offset @ up) / depth
                assert 20 < x < 1260 and 20 < y < 700, f"{name} outside evidence frame"
                frame_draw.ellipse(
                    (x - 15, y - 15, x + 15, y + 15),
                    fill="#f3db80",
                    outline="black",
                    width=2,
                )
                frame_draw.text((x, y), label, font=font(20), fill="black", anchor="mm")
            top = index * 820
            draw.text((22, top + 15), title, font=font(25), fill="white")
            sheet.paste(frame, (0, top + 56))
            draw.text(
                (22, top + 788),
                [
                    "1 Habitat    2 Solar mast    3 Lander    4 Cargo",
                    "5 Logistics module    6 Mesa    7 Rover garage",
                ][index],
                font=font(21),
                fill="#f3db80",
            )
            frame.save(out / f"moon_base_{'chase' if index == 0 else 'outer_band'}.png")
    sheet.save(out / "moon_base_render_sheet.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    args = parser.parse_args()
    baseline = json.loads(args.baseline.read_text())
    changed = json.loads(args.calibration.read_text())
    for key in ("closure_error_m", "corner_turns_deg", "corners_xy"):
        np.testing.assert_allclose(
            changed[key],
            baseline[key],
            rtol=0,
            atol=1e-10,
            err_msg=f"recorded-state regression: {key}",
        )
    model, data = load(str(args.scene))
    mujoco.mj_forward(model, data)
    result = check_base(model, data, args.scene, baseline)
    result["recorded_state_matches_baseline"] = True
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "moon_base_checks.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    for _ in range(2000):
        mujoco.mj_step(model, data)
    render_sheet(model, data, args.out)
    print("PASS: moon base checks and labelled render sheet")


if __name__ == "__main__":
    main()
