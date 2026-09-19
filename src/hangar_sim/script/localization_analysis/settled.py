#!/usr/bin/env python3
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
        def since_seed(t):
            past = [t - s for s in evs if s <= t]
            return min(past) if past else 1e9
        quiet = [r for r in rows
                 if abs(r.get("vx", 0)) + abs(r.get("vy", 0)) + abs(r.get("wz", 0)) < 0.01
                 and since_seed(r["t"]) > 15.0
                 and r.get("amcl_upd", 0) > 5
                 and abs(math.degrees(r.get("err_yaw", 9))) < 5.0]
        if not quiet:
            print(f"{path}: no settled samples"); continue

        def q(k, f=lambda v: v):
            xs = sorted(f(r["cloud"][k]) for r in quiet if k in r["cloud"])
            return xs
        r95 = q("r95"); yr95 = q("yaw_r95", math.degrees); ystd = q("yaw_std", math.degrees)
        caa = sorted(r["ap"]["caa"] for r in quiet if "ap" in r and "caa" in r["ap"])
        cxx = sorted(r["ap"]["cxx"] for r in quiet if "ap" in r and "cxx" in r["ap"])
        ed = sorted(abs(r.get("err_d", 0)) for r in quiet)
        ey = sorted(abs(math.degrees(r.get("err_yaw", 0))) for r in quiet)
        n = len(quiet)

        def pct(v, p):
            return v[min(len(v) - 1, int(p * (len(v) - 1)))] if v else float("nan")
        print(f"\n== settled state in {path}  ({n} samples, robot stopped, >15 s from any re-seed) ==")
        print(f"  cloud r95        median {st.median(r95):.3f} m    p90 {pct(r95,0.9):.3f} m")
        print(f"  cloud yaw_r95    median {st.median(yr95):.2f} deg   p90 {pct(yr95,0.9):.2f} deg")
        print(f"  cloud yaw_std    median {st.median(ystd):.2f} deg   p90 {pct(ystd,0.9):.2f} deg")
        if caa:
            print(f"  AMCL /pose sigma_yaw  median {math.degrees(math.sqrt(st.median(caa))):.2f} deg"
                  f"   p90 {math.degrees(math.sqrt(pct(caa,0.9))):.2f} deg")
            print(f"  AMCL /pose sigma_xy   median {math.sqrt(st.median(cxx)):.3f} m"
                  f"    p90 {math.sqrt(pct(cxx,0.9)):.3f} m")
        print(f"  |err| to truth   pos median {st.median(ed):.3f} m   "
              f"yaw median {st.median(ey):.2f} deg")
        # what a seed matching the settled CLOUD would have to be
        sx = st.median(r95) / 2.448          # r95 of a 2D Gaussian = 2.448 sigma
        sy = st.median(yr95) / 1.96          # 95th pct of |N(0,s)| = 1.96 s
        print(f"  -> a seed reproducing this cloud: xy_variance {sx**2:.4f} "
              f"(sigma {sx:.3f} m), yaw_variance {math.radians(sy)**2:.5f} (sigma {sy:.2f} deg)")


if __name__ == "__main__":
    main()
