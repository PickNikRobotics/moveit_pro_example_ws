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
"""Measure mjbatch physics throughput and EGL render throughput on the vla_sim Kinova scene."""
import os, sys, time

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco, numpy as np
from mjbatch import Batch

SCENE = os.path.join(
    os.path.dirname(__file__),
    "..",
    "..",
    "vla_sim",
    "description",
    "mujoco",
    "cube_stack_scene.xml",
)
m = mujoco.MjModel.from_xml_path(SCENE)
print(
    "mujoco",
    mujoco.__version__,
    "nq",
    m.nq,
    "nu",
    m.nu,
    "ncam",
    m.ncam,
    "timestep",
    m.opt.timestep,
    "sleep?",
    m.opt.enableflags,
)
for n in (64, 256, 1024):
    b = Batch(m, n)
    b.reset(keyframe=0)
    ctrl = b.bind("ctrl")
    b.step(nstep=10)
    t = time.perf_counter()
    steps = 0
    while time.perf_counter() - t < 3:
        ctrl[:, :7] += np.random.normal(0, 0.002, (n, 7))
        b.step(nstep=10)
        steps += 10
    dt = time.perf_counter() - t
    print(
        f"N={n:5d} threads={b.num_threads} physics steps/s={n*steps/dt:,.0f}  realtime factor x{n*steps/dt*m.opt.timestep:,.0f}"
    )
# rendering: one renderer, 3 cams, 224x224 (SmolVLA input after resize is 512; we render small and resize)
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
mujoco.mj_forward(m, d)
for res in (224, 480):
    r = mujoco.Renderer(m, res, res)
    t = time.perf_counter()
    k = 0
    while time.perf_counter() - t < 3:
        for cam in ("scene_camera", "wrist_camera", "overview_camera"):
            r.update_scene(d, cam)
            r.render()
        k += 1
    print(
        f"render {res}x{res}: {k/(time.perf_counter()-t):.1f} obs/s (3 cams) single-thread"
    )
    r.close()
