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
"""The StackEnv interface on plain MuJoCo (one MjData per env, thread pool), for evaluating under a
different MuJoCo version than mjbatch pins - the Runtime's 3.6.0. Runs as a worker process
(`python plain_env.py`) speaking pickled (method, args) over stdin/stdout; RemoteEnv is the client.
"""

import os
import pickle
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent


class PlainEnv:
    def __init__(self, num_envs, spec_raw, scene_path, image_size, threads=16):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.spec = spec_raw
        m, pi, rnd = self.model, spec_raw["policy_interface"], spec_raw["randomization"]
        self.n = num_envs
        self.data = [mujoco.MjData(m) for _ in range(num_envs)]
        self.keyframe = m.key(spec_raw["scene"]["keyframe"]).id
        self.arm_qadr = np.array([m.joint(j).qposadr[0] for j in pi["arm_joints"]])
        self.grip_qadr = m.joint(pi["gripper_joint"]).qposadr[0]
        self.grip_lo, self.grip_hi = pi["gripper_range"]
        self.colors = list(rnd["cube_free_joints"])
        self.cube_qadr = {
            c: m.joint(j).qposadr[0] for c, j in rnd["cube_free_joints"].items()
        }
        self.pinch = m.site("pinch").id
        self.cams = list(pi["cameras"])
        self.substeps = round(float(pi["dt"]) / m.opt.timestep)
        self.image_size = image_size
        self.pool = ThreadPoolExecutor(threads)
        self.renderer = mujoco.Renderer(m, image_size, image_size)

    def reset_xy(self, xy):
        rnd = self.spec["randomization"]
        for i, d in enumerate(self.data):
            self.mj.mj_resetDataKeyframe(self.model, d, self.keyframe)
            for k, c in enumerate(self.colors):
                a = self.cube_qadr[c]
                d.qpos[a : a + 2] = xy[i, k]
                d.qpos[a + 2] = rnd["rest_height"]
                d.qpos[a + 3 : a + 7] = [1, 0, 0, 0]
            self.mj.mj_forward(self.model, d)

    def _step_one(self, args):
        d, goal = args
        start = d.ctrl.copy()
        blocks = 10
        for k in range(1, blocks + 1):
            d.ctrl[:] = start + k / blocks * (goal - start)
            self.mj.mj_step(self.model, d, nstep=self.substeps // blocks)

    def step(self, action):
        goal = np.concatenate(
            (action[:, :7], np.clip(action[:, 7:8], self.grip_lo, self.grip_hi)), axis=1
        )
        list(self.pool.map(self._step_one, zip(self.data, goal, strict=True)))

    def joint_state(self):
        return np.stack(
            [np.r_[d.qpos[self.arm_qadr], d.qpos[self.grip_qadr]] for d in self.data]
        ).astype(np.float32)

    def render(self):
        out = np.empty(
            (self.n, len(self.cams), self.image_size, self.image_size, 3), np.uint8
        )
        for i, d in enumerate(self.data):
            for c, cam in enumerate(self.cams):
                self.renderer.update_scene(d, cam)
                out[i, c] = self.renderer.render()
        return out

    def success(self):
        t = self.spec["task"]
        ta, ba = self.cube_qadr[t["top"]], self.cube_qadr[t["bottom"]]
        edge = 2 * self.model.geom(f"cube_{t['bottom']}_geom").size[2]
        top = np.stack([d.qpos[ta : ta + 3] for d in self.data])
        bottom = np.stack([d.qpos[ba : ba + 3] for d in self.data])
        pinch = np.stack([d.site_xpos[self.pinch] for d in self.data])
        stacked = (
            np.linalg.norm(top - (bottom + [0, 0, edge]), axis=1) < t["stack_tolerance"]
        )
        return stacked & (
            np.linalg.norm(pinch - top, axis=1) > t["min_release_distance"]
        )

    def version(self):
        return self.mj.__version__


class RemoteEnv:
    """Client for a PlainEnv worker on another MuJoCo install (extra_path first on PYTHONPATH)."""

    def __init__(self, num_envs, spec, extra_path, image_size=480):
        import stack_env as se

        self.spec, self.n, self.image_size = spec, num_envs, image_size
        self._sample = se.sample_cube_xy
        env = {**os.environ, "PYTHONPATH": f"{extra_path}:{HERE}", "MUJOCO_GL": "egl"}
        self.p = subprocess.Popen(
            [sys.executable, str(HERE / "plain_env.py")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            env=env,
        )
        self.call("init", num_envs, spec.raw, str(spec.scene_path), image_size)
        self.mujoco_version = self.call("version")

    def call(self, name, *args):
        pickle.dump((name, args), self.p.stdin)
        self.p.stdin.flush()
        ok, val = pickle.load(self.p.stdout)
        if not ok:
            raise RuntimeError(f"PlainEnv worker failed in {name}: {val}")
        return val

    def reset(self, rng):
        self.cube_xy = self._sample(rng, self.n, self.spec["randomization"])
        self.call("reset_xy", self.cube_xy)

    def step(self, action):
        self.call("step", np.ascontiguousarray(action))

    def joint_state(self):
        return self.call("joint_state")

    def render(self):
        return self.call("render")

    def success(self):
        return self.call("success")


def _serve():
    env = None
    inp, out = sys.stdin.buffer, sys.stdout.buffer
    while True:
        try:
            name, args = pickle.load(inp)
        except EOFError:
            return
        try:
            if name == "init":
                env = PlainEnv(*args)
                val = None
            else:
                val = getattr(env, name)(*args)
            pickle.dump((True, val), out)
        except Exception as e:  # report to the client, which raises
            pickle.dump((False, repr(e)), out)
        out.flush()


if __name__ == "__main__":
    _serve()
