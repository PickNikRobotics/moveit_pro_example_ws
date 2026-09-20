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

"""What the filter actually holds when it is converged, measured on THIS stack.

Reads the quiet stretches of an arm recording -- robot stopped, no re-seed within 15 s --
and reports the particle cloud's own spread and AMCL's own published covariance.  That is
the number a re-seed should be asserting; anything wider manufactures doubt the filter did
not have.
"""
import json, math, statistics as st, sys


def main():
    for path in sys.argv[1:]:
        rows, evs = [], []
        for line in open(path):
            d = json.loads(line)
            if d.get("ev") == "initialpose":
                evs.append(d["stamp"])
            elif not d.get("ev") and "cloud" in d:
                rows.append(d)

        # AMCL only resamples on motion (update_min_d/update_min_a), so a cloud that has
        # never driven is still at its seeded width, not converged.  Take samples that are
        # stopped, well after the most recent re-seed, and after some driving has happened.
        # Deliberately NOT conditioned on the pose already being close to truth: selecting
        # settled samples on the answer would bias the derived seed downward.
        def since_seed(t):
            past = [t - s for s in evs if s <= t]
            return min(past) if past else 1e9

        quiet = [
            r
            for r in rows
            if abs(r.get("vx", 0)) + abs(r.get("vy", 0)) + abs(r.get("wz", 0)) < 0.01
            and since_seed(r["t"]) > 15.0
            and r.get("amcl_upd", 0) > 5
        ]
        if not quiet:
            print(f"{path}: no settled samples")
            continue

        def q(k, f=lambda v: v):
            xs = sorted(f(r["cloud"][k]) for r in quiet if k in r["cloud"])
            return xs

        r95 = q("r95")
        yr95 = q("yaw_r95", math.degrees)
        ystd = q("yaw_std", math.degrees)
        caa = sorted(r["ap"]["caa"] for r in quiet if "ap" in r and "caa" in r["ap"])
        cxx = sorted(r["ap"]["cxx"] for r in quiet if "ap" in r and "cxx" in r["ap"])
        ed = sorted(abs(r.get("err_d", 0)) for r in quiet)
        ey = sorted(abs(math.degrees(r.get("err_yaw", 0))) for r in quiet)
        n = len(quiet)

        def pct(v, p):
            return v[min(len(v) - 1, int(p * (len(v) - 1)))] if v else float("nan")

        print(
            f"\n== settled state in {path}  ({n} samples, robot stopped, >15 s from any re-seed) =="
        )
        print(
            f"  cloud r95        median {st.median(r95):.3f} m    p90 {pct(r95,0.9):.3f} m"
        )
        print(
            f"  cloud yaw_r95    median {st.median(yr95):.2f} deg   p90 {pct(yr95,0.9):.2f} deg"
        )
        print(
            f"  cloud yaw_std    median {st.median(ystd):.2f} deg   p90 {pct(ystd,0.9):.2f} deg"
        )
        if caa:
            print(
                f"  AMCL /pose sigma_yaw  median {math.degrees(math.sqrt(st.median(caa))):.2f} deg"
                f"   p90 {math.degrees(math.sqrt(pct(caa,0.9))):.2f} deg"
            )
            print(
                f"  AMCL /pose sigma_xy   median {math.sqrt(st.median(cxx)):.3f} m"
                f"    p90 {math.sqrt(pct(cxx,0.9)):.3f} m"
            )
        print(
            f"  |err| to truth   pos median {st.median(ed):.3f} m   "
            f"yaw median {st.median(ey):.2f} deg"
        )

        # What a seed matching the settled CLOUD would have to be. r95 of a 2D Gaussian is
        # 2.448 sigma; the 95th percentile of |N(0,s)| is 1.96 s.
        def seed_from(rv, yv):
            return rv / 2.448, yv / 1.96

        mx, my = seed_from(st.median(r95), st.median(yr95))
        px, py = seed_from(pct(r95, 0.9), pct(yr95, 0.9))
        print(
            f"  -> seed from the MEDIAN settled cloud: xy_variance {mx**2:.4f} "
            f"(sigma {mx:.3f} m), yaw_variance {math.radians(my)**2:.5f} (sigma {my:.2f} deg)"
        )
        print(
            f"  -> seed from the P90    settled cloud: xy_variance {px**2:.4f} "
            f"(sigma {px:.3f} m), yaw_variance {math.radians(py)**2:.5f} (sigma {py:.2f} deg)"
        )
        print(
            "     the committed seed uses the P90: the rule is that a seed must not be tighter"
        )
        print(
            "     than what the filter holds when converged. The median is the test's floor."
        )


if __name__ == "__main__":
    main()
