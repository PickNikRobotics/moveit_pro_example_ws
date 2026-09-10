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

"""Phase-1 validation: MJCF smoke test, Dead Reckon Square calibration recheck
(RECORDED STATE - freejoint chassis pose, not /odom), and comparison renders.

Run inside the picknikciuser/moveit-pro container (has MuJoCo 3.6 + PIL):
  docker run --rm --entrypoint python3 \
    -v "$(pwd)/src/lunar_sim/description:/mjcf:ro" \
    -v "<this script>:/validate.py:ro" \
    -v "<out dir>:/out" \
    -e MUJOCO_GL=egl \
    picknikciuser/moveit-pro:main-jazzy-amd64-cuda13.2-cudnn9 /validate.py /mjcf/husky_scene.xml /out/new
Pass a second scene path (e.g. an earlier husky_scene.xml, mounted read-only
alongside) as a third arg to also render it for the comparison sheet. That scene
must define the current camera and wheel-actuator names (chase_camera,
scene_camera, <side>_<pos>_wheel_joint) and a keyframe named "default", which
load() applies via reset_to_keyframe; an older one missing any of them raises
KeyError at load rather than rendering something misleading.
"""
import json
import sys
import math
import mujoco
import numpy as np
from PIL import Image

WHEEL_RADIUS = 0.1645
WHEEL_SEPARATION = 0.562
MULTIPLIER = 2.57
V_CMD = 0.3
W_CMD = 0.5235988  # 30 deg/s

# The keyframe husky_a300_mujoco.xacro's mujoco_keyframe arg selects at hardware init.
KEYFRAME_NAME = "default"

# Actuator names track the wheel joint names in husky_a300.xml.
WHEEL_ACTUATORS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]


def require_id(model, objtype, name):
    """mj_name2id returns -1 for an unknown name, and every consumer of that id
    accepts -1 silently: data.ctrl[-1] drives the last actuator, data.xpos[-1] reads
    the last body, and update_scene(camera=-1) falls back to the free camera. Resolve
    names through here so a renamed model element fails loudly instead."""
    obj_id = mujoco.mj_name2id(model, objtype, name)
    if obj_id < 0:
        raise KeyError(f"no {objtype.name} named {name!r} in the model")
    return obj_id


def wheel_actuator_ids(model):
    return {
        name: require_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in WHEEL_ACTUATORS
    }


def reset_to_keyframe(model, data):
    """Put the base where the simulation actually boots. mj_resetData restores qpos0
    (husky_a300.xml's chassis_link pos, i.e. the world origin), which is not the pose
    husky_a300_mujoco.xacro's mujoco_keyframe selects - only mj_resetDataKeyframe
    applies a <keyframe>."""
    key_id = require_id(model, mujoco.mjtObj.mjOBJ_KEY, KEYFRAME_NAME)
    mujoco.mj_resetDataKeyframe(model, data, key_id)


def quat_to_yaw(q):
    w, x, y, z = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def load(scene_path):
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    reset_to_keyframe(model, data)
    return model, data


def smoke_test(model, data):
    chassis_id = require_id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")
    mujoco.mj_forward(model, data)
    spawn_xy = data.xpos[chassis_id][:2].copy()
    for _ in range(2000):
        mujoco.mj_step(model, data)
    settled_z = data.xpos[chassis_id][2]
    drift = data.xpos[chassis_id][:2].copy() - spawn_xy
    print(f"[smoke] settled chassis pos={data.xpos[chassis_id]}")
    assert 0.05 < settled_z < 0.30, f"implausible settle height z={settled_z}"
    assert np.linalg.norm(drift) < 0.05, f"drifted while settling: {drift}"

    aid = wheel_actuator_ids(model)
    for a in aid.values():
        data.ctrl[a] = 3.0
    start_xy = data.xpos[chassis_id][:2].copy()
    for _ in range(2000):
        mujoco.mj_step(model, data)
    dist = np.linalg.norm(data.xpos[chassis_id][:2].copy() - start_xy)
    print(f"[smoke] drove {dist:.4f} m under wheel commands")
    assert dist > 0.3, f"base barely moved ({dist:.4f} m) - wheels may be pinned"
    print("[smoke] PASS")


def dead_reckon_square(model, data):
    """RECORDED STATE calibration recheck: ground-truth chassis freejoint pose
    (xpos/xquat), never /odom (which round-trips the same wrong parameters by
    construction - see README's Skid-steer calibration section)."""
    reset_to_keyframe(model, data)
    chassis_id = require_id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")
    aid = wheel_actuator_ids(model)

    def set_wheels(l, r):
        data.ctrl[aid["front_left_wheel_joint"]] = l
        data.ctrl[aid["rear_left_wheel_joint"]] = l
        data.ctrl[aid["front_right_wheel_joint"]] = r
        data.ctrl[aid["rear_right_wheel_joint"]] = r

    set_wheels(0, 0)
    for _ in range(2000):
        mujoco.mj_step(model, data)

    w_straight = V_CMD / WHEEL_RADIUS
    L = WHEEL_SEPARATION * MULTIPLIER
    wheel_diff_turn = W_CMD * L / 2 / WHEEL_RADIUS
    n_straight = round(6.0 / model.opt.timestep)
    n_turn = round(3.0 / model.opt.timestep)

    start_xy = data.xpos[chassis_id][:2].copy()
    corners = [start_xy]
    yaws = [quat_to_yaw(data.xquat[chassis_id])]
    for _ in range(4):
        set_wheels(w_straight, w_straight)
        for _ in range(n_straight):
            mujoco.mj_step(model, data)
        corners.append(data.xpos[chassis_id][:2].copy())
        set_wheels(-wheel_diff_turn, wheel_diff_turn)
        for _ in range(n_turn):
            mujoco.mj_step(model, data)
        yaws.append(quat_to_yaw(data.xquat[chassis_id]))

    set_wheels(0, 0)
    for _ in range(200):
        mujoco.mj_step(model, data)
    final_xy = data.xpos[chassis_id][:2].copy()
    closure_error = float(np.linalg.norm(final_xy - start_xy))
    # quat_to_yaw wraps to (-180, 180]; unwrap each per-corner diff into the same
    # range so a turn that crosses the +-180 boundary isn't misreported as ~-260deg.
    raw_diffs = [yaws[i + 1] - yaws[i] for i in range(len(yaws) - 1)]
    diffs = [((d + 180) % 360) - 180 for d in raw_diffs]

    print(f"[square] closure_error={closure_error:.4f} m")
    for i, c in enumerate(corners):
        print(f"[square] corner {i} pos: {c}")
    print(f"[square] per-corner turn amount (deg): {[f'{d:.2f}' for d in diffs]}")
    return {
        "closure_error_m": closure_error,
        "corner_turns_deg": diffs,
        "corners_xy": [c.tolist() for c in corners],
    }


def render(scene_path, out_prefix):
    model, data = load(scene_path)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    cam_id = require_id(model, mujoco.mjtObj.mjOBJ_CAMERA, "chase_camera")
    mast_cam_id = require_id(model, mujoco.mjtObj.mjOBJ_CAMERA, "scene_camera")
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[3] = 0

    for _ in range(2000):
        mujoco.mj_step(model, data)
    renderer.update_scene(data, camera=cam_id, scene_option=scene_option)
    Image.fromarray(renderer.render()).save(f"{out_prefix}_settled.png")
    renderer.update_scene(data, camera=mast_cam_id, scene_option=scene_option)
    Image.fromarray(renderer.render()).save(f"{out_prefix}_scene_camera.png")

    for aid in wheel_actuator_ids(model).values():
        data.ctrl[aid] = 3.0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    renderer.update_scene(data, camera=cam_id, scene_option=scene_option)
    Image.fromarray(renderer.render()).save(f"{out_prefix}_driving.png")
    print(f"[render] saved {out_prefix}_{{settled,scene_camera,driving}}.png")


def main():
    new_scene = sys.argv[1]
    out_prefix = sys.argv[2]
    old_scene = sys.argv[3] if len(sys.argv) > 3 else None

    model, data = load(new_scene)
    smoke_test(model, data)
    stats = dead_reckon_square(model, data)

    # Persist the calibration before any rendering: the renders are optional
    # artifacts, and require_id makes the old-scene comparison render a hard
    # failure on a scene that predates the current camera/actuator names. Writing
    # first keeps a minutes-long calibration run from being thrown away by it.
    with open(out_prefix + "_calibration.json", "w") as f:
        json.dump(stats, f, indent=2)

    render(new_scene, out_prefix)
    if old_scene:
        render(old_scene, out_prefix + "_OLDFLAT")
    print("DONE")


if __name__ == "__main__":
    main()
