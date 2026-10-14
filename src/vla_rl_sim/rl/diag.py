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
"""Diagnose a checkpoint: open-loop action error on demo frames, and closed-loop task progress."""
import sys, numpy as np, torch
import stack_env as se
from vla import SmolVLA

ck = sys.argv[1]
spec = se.Spec.load()
vla = SmolVLA(ck, spec.image_names, 480)
z = np.load("demos_256.npz")
ok = np.flatnonzero(z["success"])
env = se.StackEnv(16)
rng = np.random.default_rng(1)
ep = rng.choice(ok, 16)
t = rng.integers(0, 100, 16)
imgs = env.render_states(z["sim_states"][ep, t])
pred, _ = vla.sample(imgs, z["states"][ep, t], spec.prompt)
gt = np.stack([z["actions"][e, ti : ti + 20] for e, ti in zip(ep, t)])
err = np.abs(pred[:, :20] - gt)
print(
    "open-loop |pred-gt| over 20 steps: arm mean %.4f rad, max %.3f; gripper mean %.3f"
    % (err[..., :7].mean(), err[..., :7].max(), err[..., 7].mean())
)
print(
    "per-horizon arm err (steps 0,5,10,19):",
    np.round(err[:, [0, 5, 10, 19], :7].mean((0, 2)), 4),
)
# closed loop progress
env.reset(np.random.default_rng(5))
red0 = env.cube_pos("red")
min_d = np.full(16, 9.0)
max_lift = np.zeros(16)
grip_closed = np.zeros(16)
for k in range(6):
    imgs = env.render()
    st = env.joint_state()
    chunk, _ = vla.sample(imgs, st, spec.prompt)
    for j in range(20):
        env.step(chunk[:, j])
        min_d = np.minimum(
            min_d, np.linalg.norm(env.pinch - env.cube_pos("red"), axis=1)
        )
        max_lift = np.maximum(max_lift, env.cube_pos("red")[:, 2] - red0[:, 2])
        grip_closed = np.maximum(grip_closed, env.joint_state()[:, 7])
fin = np.linalg.norm(env.cube_pos("red")[:, :2] - env.cube_pos("green")[:, :2], axis=1)
print("closed-loop: min pinch-red dist (cm)", np.round(min_d * 100, 1))
print("max red lift (cm)", np.round(max_lift * 100, 1))
print("max gripper knuckle", np.round(grip_closed, 2))
print(
    "final red-green xy dist (cm)",
    np.round(fin * 100, 1),
    "success",
    env.success().sum(),
)
