#!/usr/bin/env python3
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
