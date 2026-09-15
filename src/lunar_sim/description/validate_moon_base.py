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


def check_base(model, data, scene, baseline):
    ids = [i for i in range(model.ngeom) if model.geom(i).name.startswith("moon_base_")]
    assert ids, "no moon base geoms loaded"
    bodies = {int(model.geom_bodyid[i]) for i in ids}
    assert len(bodies) == 5, f"expected five fixed bodies, got {len(bodies)}"
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
        assert np.all(low[:2] >= -10) and np.all(
            high[:2] <= 10
        ), f"{model.geom(i).name} extends beyond the terrain"
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
        ix = round((x + half_x) / (2 * half_x) * heights.shape[0])
        iy = round((y + half_y) / (2 * half_y) * heights.shape[1])
        assert (
            abs(z - (heights[ix, iy] - 0.04)) < 1e-6
        ), f"{model.body(body).name} datum differs from the committed terrain sample"

    # A visible-group ray from a future rover sensor must hit real base collision
    # geometry. Aim at elevated parts so the terrain and the rover do not mask it.
    origin = np.array([-0.30, 0.50, 1.2])
    groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    visible = []
    for name in (
        "moon_base_door",
        "moon_base_solar_left",
        "moon_base_antenna_panel",
        "moon_base_lander_payload",
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
            "Chase view | looking across the rover toward the base",
            [-5, -4, 5],
            [1, 4.4, 1.1],
        ),
        (
            "Route start corner | world (-0.30, 0.50), sensor height 1.2 m",
            [-0.30, 0.50, 1.2],
            [0.2, 6.5, 1.5],
        ),
    ]
    landmarks = [
        ("1", "moon_base_habitat_shell"),
        ("2", "moon_base_solar_left"),
        ("3", "moon_base_antenna_panel"),
        ("4", "moon_base_lander_payload"),
        ("5", "moon_base_pad_mark_right"),
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
            for label, name in landmarks:
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
                "1 Habitat / door    2 Solar array    3 Antenna    4 Lander    5 Landing pad",
                font=font(21),
                fill="#f3db80",
            )
            frame.save(
                out / f"moon_base_{'chase' if index == 0 else 'start_corner'}.png"
            )
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
