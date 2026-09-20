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

"""Odometry inflation, and the real-time factor it implies, for a recording.

AGENTS.md: the MuJoCo loop does not keep real time and the wheel velocity interface is
per SIM second while the controller integrates against a wall clock, so wheel odometry
over-reports travel by 1/RTF - 1.  Any localization number from this sim is meaningless
without it, which is why every arm reports it.
"""
import json, math, sys


def pathlen(rows, sub):
    tot, prev = 0.0, None
    for r in rows:
        if sub not in r:
            continue
        x, y = r[sub]["x"], r[sub]["y"]
        if prev is not None:
            d = math.hypot(x - prev[0], y - prev[1])
            if d < 1.0:
                tot += d
        prev = (x, y)
    return tot


def truthlen(rows):
    tot, prev = 0.0, None
    for r in rows:
        if "tx" not in r:
            continue
        if prev is not None:
            d = math.hypot(r["tx"] - prev[0], r["ty"] - prev[1])
            if d < 1.0:
                tot += d
        prev = (r["tx"], r["ty"])
    return tot


for p in sys.argv[1:]:
    rows = []
    for line in open(p):
        d = json.loads(line)
        if not d.get("ev"):
            rows.append(d)
    mv = [
        x
        for x in rows
        if abs(x.get("vx", 0)) + abs(x.get("vy", 0)) + abs(x.get("wz", 0)) > 0.02
    ]
    t = truthlen(mv)
    w = pathlen(mv, "wo")
    if t > 1.0 and w > 1.0:
        print(
            f"{p}: nav odom {w:.2f} m vs truth {t:.2f} m -> inflation {100*(w/t-1):+.1f}%  "
            f"implied RTF {t/w:.3f}  ({len(mv)} moving samples)"
        )
    else:
        print(f"{p}: not enough motion (truth {t:.2f} m, odom {w:.2f} m)")
