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

"""Separate real particle-filter divergences from estimator stalls.

Two different things look identical in `err_yaw`:

  a real divergence  the particle filter converges on a wrong heading. The localizer's own
                     correction, map->odom, has to move -- either because it moved wrongly or
                     because it has to move back. The cloud reacts and position error grows.

  an estimator stall under load, fuse's fixed-lag smoother falls behind and keeps publishing a
                     FROZEN /odom_filtered with FRESH stamps (AGENTS.md documents this failure
                     mode). Navigation runs on that estimate, so while the base rotates its
                     believed heading does not move, and the error measured against truth is
                     simply however far it turned. map->odom never moves, the cloud never
                     reacts, and the whole thing vanishes when fuse catches up in one jump.

The second is not the failure this task is about, and counting it in either arm would be
counting the wrong thing. The test is mechanism-specific and observable: was /odom_filtered
bit-frozen while the base was turning?
"""
import json, math, os, statistics as st, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hfresh import load_rows, annotate  # noqa: E402
from armsumm import episodes, load  # noqa: E402

D = math.degrees
THRESH = float(os.environ.get("THRESH", "20.0"))
FROZEN_S = 1.0  # /odom_filtered unchanged this long while turning = stalled


def fo_frozen_span(rows, i0, i1):
    """Longest stretch inside [i0,i1] where fo yaw is bit-identical while the base turns."""
    best = 0.0
    run_start, last = None, None
    for r in rows[i0 : i1 + 1]:
        if "fo" not in r:
            continue
        y = r["fo"]["yaw"]
        if last is not None and y == last:
            if run_start is None:
                run_start = prev_t
            best = max(best, r["t"] - run_start)
        else:
            run_start = None
        last, prev_t = y, r["t"]
    return best


def ang_span(angles):
    """Smallest arc containing every sample, in radians.

    max - min is wrong on a circle: map->odom yaw is an atan2 output in (-pi, pi], so a
    correction merely jittering either side of +/-pi spans ~2*pi that way and reads as a
    ~360 deg move of a correction that never actually moved.
    """
    if not angles:
        return 0.0
    a = sorted(math.atan2(math.sin(x), math.cos(x)) for x in angles)
    if len(a) == 1:
        return 0.0
    gaps = [a[i + 1] - a[i] for i in range(len(a) - 1)]
    gaps.append(2.0 * math.pi - (a[-1] - a[0]))
    return 2.0 * math.pi - max(gaps)


def classify(rows, i0, i1):
    """Return (verdict, evidence) for one episode."""
    w = rows[max(0, i0 - 5) : min(len(rows), i1 + 40)]  # include the filter's reaction
    mo = [r["mo_yaw"] for r in w if "mo_yaw" in r]
    mo_span = D(ang_span(mo))
    upd = [r.get("amcl_upd", 0) for r in w]
    d_upd = (max(upd) - min(upd)) if upd else 0
    ed = [r["err_d"] for r in rows[i0 : i1 + 1] if "err_d" in r]
    frozen = fo_frozen_span(rows, max(0, i0 - 25), i1)
    turning = max((abs(r.get("wz", 0)) for r in rows[i0 : i1 + 1]), default=0.0)
    ev = dict(
        mo_span=mo_span,
        d_upd=d_upd,
        err_d=max(ed) if ed else 0.0,
        fo_frozen=frozen,
        wz=turning,
    )
    if frozen >= FROZEN_S and turning > 0.1:
        return "STALL(fuse odom frozen)", ev
    if mo_span < 2.0:
        return "STALL(map->odom never moved)", ev
    return "DIVERGENCE", ev


def main(paths):
    tot = {}
    for p in paths:
        rows, evs = load(p)
        rows = annotate(rows)
        eps_all = episodes(rows, THRESH)
        eps = [e for e in eps_all if e["corroborated"]]
        unc = [e for e in eps_all if not e["corroborated"]]
        if not eps and not unc:
            continue
        t0 = rows[0]["t"]
        nm = os.path.basename(p).replace(".jsonl", "")
        print(f"\n--- {nm} ---")
        if unc:
            print(
                f"  {len(unc)} episode(s) had no /pose update in their window and are "
                f"UNCORROBORATED, not adjudicated (no AMCL publisher in this recording):"
            )
            for e in unc:
                print(
                    f"       t={e['t']-t0:7.1f}  peak {e['pk']:6.1f}d  -> UNCORROBORATED"
                )
        for e in eps:
            i0, i1 = e["i0"], e["i1"]
            v, ev = classify(rows, i0, i1)
            hf = max(
                (abs(D(r["h_fresh"])) for r in rows[i0 : i1 + 1] if "h_fresh" in r),
                default=float("nan"),
            )
            print(
                f"  t={e['t']-t0:7.1f}  peak {e['pk']:6.1f}d  h_fresh {hf:6.1f}d  "
                f"-> {v}"
            )
            print(
                f"       map->odom yaw moved {ev['mo_span']:5.2f}d, amcl updates {ev['d_upd']:2d}, "
                f"err_d max {ev['err_d']:4.2f} m, /odom_filtered frozen {ev['fo_frozen']:4.2f} s, "
                f"|wz| max {ev['wz']:.2f}"
            )
            tot.setdefault(v.split("(")[0], 0)
            tot[v.split("(")[0]] += 1
    print("\n  totals:", tot)


if __name__ == "__main__":
    main(sys.argv[1:])
