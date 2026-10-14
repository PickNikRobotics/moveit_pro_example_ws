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
"""Batched cube-stacking env on mjbatch, configured entirely by a robot config package's
rl_post_training.yaml (vla_rl_sim), so training sees the Runtime's model, cameras, rates and
joint order."""

import os
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import yaml
from mjbatch import Batch

WS_SRC = Path(
    os.environ.get("EXAMPLE_WS_SRC", Path(__file__).resolve().parents[2])
)  # the workspace src/
SPEC = WS_SRC / "vla_rl_sim" / "config" / "rl_post_training.yaml"
INTERP_BLOCKS = 10  # ramp targets across the control period, as the trajectory controller interpolates


@dataclass(frozen=True)
class Spec:
    raw: dict

    @classmethod
    def load(cls, path: Path = SPEC) -> "Spec":
        return cls(yaml.safe_load(path.read_text()))

    def __getitem__(self, k):
        return self.raw[k]

    @property
    def scene_path(self) -> Path:
        # Source tree stands in for the installed share directory.
        s = self["scene"]
        return WS_SRC / s["package"] / s["path"]

    @property
    def cameras(self) -> list[str]:
        return list(self["policy_interface"]["cameras"])

    @property
    def image_names(self) -> list[str]:
        return list(self["policy_interface"]["cameras"].values())

    @property
    def prompt(self) -> str:
        return self["policy_interface"]["prompt"]

    @property
    def dt(self) -> float:
        return float(self["policy_interface"]["dt"])

    @property
    def committed(self) -> int:
        return int(self["policy_interface"]["committed_action_steps"])

    @property
    def episode_steps(self) -> int:
        return int(self["task"]["episode_steps"])


def sample_cube_xy(rng: np.random.Generator, n: int, r: dict) -> np.ndarray:
    """(n, 3 cubes, 2) table positions with pairwise separation >= min_separation."""
    out = np.empty((n, 3, 2))
    for i in range(n):
        while True:
            xy = np.column_stack(
                (rng.uniform(*r["x_range"], 3), rng.uniform(*r["y_range"], 3))
            )
            d = np.linalg.norm(xy[:, None] - xy[None], axis=-1)[np.triu_indices(3, 1)]
            if d.min() >= r["min_separation"]:
                out[i] = xy
                break
    return out


class StackEnv:
    def __init__(
        self,
        num_envs: int,
        spec: Spec | None = None,
        image_size: int = 480,
        render_threads: int = 12,
    ):
        self.spec = spec or Spec.load()
        self.model = mujoco.MjModel.from_xml_path(str(self.spec.scene_path))
        m, pi, rnd = (
            self.model,
            self.spec["policy_interface"],
            self.spec["randomization"],
        )
        self.keyframe = m.key(self.spec["scene"]["keyframe"]).id
        self.n = num_envs
        self.batch = Batch(m, num_envs, forward=True)
        self.qpos, self.ctrl, self.state = (
            self.batch.bind(k) for k in ("qpos", "ctrl", "state")
        )
        self.arm_qadr = np.array([m.joint(j).qposadr[0] for j in pi["arm_joints"]])
        self.grip_qadr = m.joint(pi["gripper_joint"]).qposadr[0]
        self.grip_lo, self.grip_hi = pi["gripper_range"]
        self.colors = list(rnd["cube_free_joints"])
        self.cube_qadr = {
            c: m.joint(j).qposadr[0] for c, j in rnd["cube_free_joints"].items()
        }
        self.pinch = self.batch.site("pinch").xpos
        self.image_size = image_size
        self._local = threading.local()
        self._pool = ThreadPoolExecutor(render_threads)
        self.substeps = round(self.spec.dt / m.opt.timestep)
        self.cube_xy = np.zeros((num_envs, len(self.colors), 2))

    def reset(self, rng: np.random.Generator, ids: np.ndarray | None = None) -> None:
        ids = np.arange(self.n) if ids is None else np.asarray(ids)
        rnd = self.spec["randomization"]
        self.batch.reset(ids, keyframe=self.keyframe)
        xy = sample_cube_xy(rng, len(ids), rnd)
        self.cube_xy[ids] = xy
        for k, c in enumerate(self.colors):
            a = self.cube_qadr[c]
            self.qpos[ids, a : a + 2] = xy[:, k]
            self.qpos[ids, a + 2] = rnd["rest_height"]
            self.qpos[ids, a + 3 : a + 7] = [1, 0, 0, 0]
        self.batch.forward(ids)

    def step(self, action: np.ndarray) -> None:
        """action (n, 8): 7 absolute arm joint targets + gripper target, ramped linearly from the
        previous target over the control period."""
        start = self.ctrl.copy()
        goal = np.concatenate(
            (action[:, :7], np.clip(action[:, 7:8], self.grip_lo, self.grip_hi)), axis=1
        )
        per = self.substeps // INTERP_BLOCKS
        for k in range(1, INTERP_BLOCKS + 1):
            self.ctrl[:] = start + (k / INTERP_BLOCKS) * (goal - start)
            self.batch.step(nstep=per)

    def joint_state(self) -> np.ndarray:
        """(n, 8): arm joints + gripper knuckle, the order ExecutePolicy sends (gripper last)."""
        return np.concatenate(
            (self.qpos[:, self.arm_qadr], self.qpos[:, [self.grip_qadr]]), axis=1
        ).astype(np.float32)

    def cube_pos(self, color: str) -> np.ndarray:
        a = self.cube_qadr[color]
        return self.qpos[:, a : a + 3].copy()

    def _renderer(self):
        loc = self._local
        if not hasattr(loc, "r"):
            loc.r = mujoco.Renderer(self.model, self.image_size, self.image_size)
            loc.d = mujoco.MjData(self.model)
        return loc.r, loc.d

    def _render_row(self, row: np.ndarray) -> np.ndarray:
        r, d = self._renderer()
        mujoco.mj_setState(self.model, d, row, mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(self.model, d)
        frames = []
        for cam in self.spec.cameras:
            r.update_scene(d, cam)
            frames.append(r.render())
        return np.stack(frames)

    def render(self, ids=None) -> np.ndarray:
        """(len(ids), n_cams, H, W, 3) uint8 of the live sims, cameras in spec order."""
        ids = range(self.n) if ids is None else ids
        return self.render_states(self.state[list(ids)])

    def render_states(self, rows: np.ndarray) -> np.ndarray:
        """Render stored integration states (a states-only dataset), (B, n_cams, H, W, 3) uint8."""
        return np.stack(list(self._pool.map(self._render_row, rows)))

    def potential(self) -> np.ndarray:
        """Task progress in [0, 1] for potential-based reward shaping: gripper to top cube, then top cube
        to its stacked position. Shaping with a potential leaves the optimal policy unchanged.
        """
        t = self.spec["task"]
        top, bottom = self.cube_pos(t["top"]), self.cube_pos(t["bottom"])
        edge = 2 * self.model.geom(f"cube_{t['bottom']}_geom").size[2]
        reach = np.linalg.norm(self.pinch - top, axis=1)
        place = np.linalg.norm(top - (bottom + [0, 0, edge]), axis=1)
        return 0.5 * (1 - np.tanh(10 * reach)) + 0.5 * (1 - np.tanh(10 * place))

    def success(self) -> np.ndarray:
        """The "Check if One Cube Is on Another" test: top center within tolerance of the point one
        cube above the bottom center, and the gripper away from the top cube."""
        t = self.spec["task"]
        top, bottom = self.cube_pos(t["top"]), self.cube_pos(t["bottom"])
        edge = 2 * self.model.geom(f"cube_{t['bottom']}_geom").size[2]
        offset = top - (bottom + [0, 0, edge])
        stacked = np.linalg.norm(offset, axis=1) < t["stack_tolerance"]
        released = np.linalg.norm(self.pinch - top, axis=1) > t["min_release_distance"]
        return stacked & released
