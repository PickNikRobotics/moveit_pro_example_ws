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
"""Replay recorded scripted actions open-loop with plain MuJoCo (no mjbatch) and report outcomes,
to compare physics across MuJoCo versions. Usage: replay_parity.py <scene.xml> <demos.npz> <out.npz>
"""
import sys
import mujoco
import numpy as np

scene, demos, out = sys.argv[1:4]
m = mujoco.MjModel.from_xml_path(scene)
d = mujoco.MjData(m)
z = np.load(demos)
acts, s0 = z["actions"], z["sim_states"][:, 0]
sub = round(0.1 / m.opt.timestep)
blocks = 10
red = m.joint("cube_red_free").qposadr[0]
green = m.joint("cube_green_free").qposadr[0]
pinch = m.site("pinch").id
n = int(sys.argv[4]) if len(sys.argv) > 4 else len(acts)
final = np.zeros((n, 3, 3))
for i in range(n):
    mujoco.mj_resetDataKeyframe(m, d, 0)
    # Start from the recorded qpos (cube layout); version-independent part of the state.
    d.qpos[:] = s0[i, 1 : 1 + m.nq]
    mujoco.mj_forward(m, d)
    for a in acts[i]:
        start = d.ctrl.copy()
        goal = np.r_[a[:7], np.clip(a[7], 0, 0.8)]
        for k in range(1, blocks + 1):
            d.ctrl[:] = start + k / blocks * (goal - start)
            mujoco.mj_step(m, d, nstep=sub // blocks)
    final[i] = [d.qpos[red : red + 3], d.qpos[green : green + 3], d.site_xpos[pinch]]
off = final[:, 0] - (final[:, 1] + [0, 0, 0.03])
succ = (np.linalg.norm(off, axis=1) < 0.015) & (
    np.linalg.norm(final[:, 2] - final[:, 0], axis=1) > 0.05
)
print(f"mujoco {mujoco.__version__}: success {succ.mean():.3f} ({succ.sum()}/{n})")
np.savez(out, final=final, succ=succ)
