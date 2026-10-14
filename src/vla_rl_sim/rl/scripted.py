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
"""Privileged-state scripted stacking demos: damped least-squares IK on the pinch site, joint-space
interpolation between waypoints at 10 Hz. Used to build the SFT dataset for the base VLA.
"""

import argparse
import os

os.environ.setdefault("MUJOCO_GL", "egl")
import time
from pathlib import Path

import mujoco
import numpy as np

import stack_env as se

# Top-down grasp: pinch-site z axis along world -z.
R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
APPROACH, GRASP_Z, LIFT_Z = 0.10, 0.118, 0.25
PLACE_LIFT = 0.006  # release the top cube this far above resting on the bottom one


def ik(model, data, arm_joints, q_seed, pos, rot=R_DOWN, iters=200, damping=1e-3):
    site = model.site("pinch").id
    qadr = np.array([model.joint(j).qposadr[0] for j in arm_joints])
    dadr = np.array([model.joint(j).dofadr[0] for j in arm_joints])
    q = q_seed.copy()
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for _ in range(iters):
        data.qpos[qadr] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        xr = data.site_xmat[site].reshape(3, 3)
        ep = pos - data.site_xpos[site]
        er = 0.5 * sum(np.cross(xr[:, k], rot[:, k]) for k in range(3))
        err = np.r_[ep, er]
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(er) < 1e-3:
            break
        mujoco.mj_jacSite(model, data, jacp, jacr, site)
        J = np.vstack((jacp, jacr))[:, dadr]
        dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), err)
        q = q + np.clip(dq, -0.2, 0.2)
        lo, hi = model.jnt_range[[model.joint(j).id for j in arm_joints]].T
        limited = model.jnt_limited[[model.joint(j).id for j in arm_joints]].astype(
            bool
        )
        q[limited] = np.clip(q[limited], lo[limited], hi[limited])
    return q, np.linalg.norm(ep), np.linalg.norm(er)


def plan(env, data, q_home, red_xy, green_xy, rng):
    """Waypoints [(arm q, gripper, seconds)] for one episode; small timing jitter for diversity."""
    model, arm_joints = env.model, env.spec["policy_interface"]["arm_joints"]
    rest = env.spec["randomization"]["rest_height"]
    place_z = rest + 2 * model.geom("cube_green_geom").size[2] + PLACE_LIFT

    def at(xy, z, seed):
        q, ep, er = ik(model, data, arm_joints, seed, np.r_[xy, z])
        if ep > 2e-3 or er > 2e-2:
            raise RuntimeError(f"IK failed at {xy}, {z}: {ep:.4f} m, {er:.3f} rad")
        return q

    j = lambda lo, hi: rng.uniform(lo, hi)  # noqa: E731
    q_above_red = at(red_xy, GRASP_Z + APPROACH, q_home)
    q_grasp = at(red_xy, GRASP_Z, q_above_red)
    q_lift = at(red_xy, LIFT_Z, q_grasp)
    q_above_green = at(green_xy, place_z + APPROACH, q_lift)
    q_place = at(green_xy, place_z, q_above_green)
    O, C = env.grip_lo, env.grip_hi
    return [
        (q_above_red, O, j(2.0, 2.6)),
        (q_grasp, O, j(1.0, 1.4)),
        (q_grasp, C, j(0.8, 1.0)),
        (q_lift, C, j(1.0, 1.4)),
        (q_above_green, C, j(1.4, 1.8)),
        (q_place, C, j(1.0, 1.3)),
        (q_place, O, j(0.6, 0.8)),
        (q_above_green, O, j(0.8, 1.0)),
    ]


def to_actions(q_start, waypoints, horizon, dt, grip_open):
    """(horizon, 8) 10 Hz absolute targets; joint-space smoothstep between waypoints, hold at end."""
    acts, q_prev, g_prev = [], q_start, grip_open
    for q, g, dur in waypoints:
        k = max(1, round(dur / dt))
        for s in range(1, k + 1):
            a = s / k
            a = a * a * (3 - 2 * a)
            acts.append(np.r_[q_prev + a * (q - q_prev), g_prev + a * (g - g_prev)])
        q_prev, g_prev = q, g
    acts = np.array(acts)
    if len(acts) < horizon:
        acts = np.vstack((acts, np.repeat(acts[-1:], horizon - len(acts), 0)))
    return acts[:horizon]


def generate(n, seed, horizon, record_images, image_size=256):
    rng = np.random.default_rng(seed)
    env = se.StackEnv(n, image_size=image_size)
    env.reset(rng)
    model, data = env.model, mujoco.MjData(env.model)
    mujoco.mj_resetDataKeyframe(model, data, env.keyframe)
    q_home = data.qpos[env.arm_qadr].copy()
    acts = np.zeros((n, horizon, 8))
    ok = np.ones(n, bool)
    for i in range(n):
        try:
            wps = plan(env, data, q_home, env.cube_xy[i, 0], env.cube_xy[i, 1], rng)
            acts[i] = to_actions(q_home, wps, horizon, env.spec.dt, env.grip_lo)
        except RuntimeError as e:
            ok[i] = False
            print("plan failed", i, e)
    states = np.zeros((n, horizon, 8), np.float32)
    sim_states = np.zeros((n, horizon, env.batch.nstate))
    images = (
        np.zeros(
            (n, horizon, len(env.spec.cameras), image_size, image_size, 3), np.uint8
        )
        if record_images
        else None
    )
    done_at = np.full(n, -1)
    for t in range(horizon):
        states[:, t] = env.joint_state()
        sim_states[:, t] = env.state
        if record_images:
            images[:, t] = env.render()
        env.step(acts[:, t])
        s = env.success()
        done_at[(done_at < 0) & s] = t
    succ = env.success() & ok
    return dict(
        sim_states=sim_states,
        states=states,
        actions=acts.astype(np.float32),
        images=images,
        success=succ,
        done_at=done_at,
        cube_xy=env.cube_xy.copy(),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--n", type=int, default=64)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--horizon", type=int, default=120)
    p.add_argument("--images", action="store_true")
    p.add_argument("--out", type=Path)
    a = p.parse_args()
    t0 = time.perf_counter()
    r = generate(a.n, a.seed, a.horizon, a.images)
    print(
        f"success {r['success'].mean():.3f} ({r['success'].sum()}/{a.n}), mean done step {r['done_at'][r['success']].mean():.1f}, {time.perf_counter() - t0:.1f}s"
    )
    if a.out:
        np.savez_compressed(a.out, **{k: v for k, v in r.items() if v is not None})
