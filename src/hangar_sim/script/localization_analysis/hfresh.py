#!/usr/bin/env python3
"""A heading error that a stalled pipeline cannot manufacture.

`err_yaw` in the recordings is the map->base transform the recorder looked up, differenced
against truth at that transform's own stamp. Under load that lookup comes back seconds old
(measured: >1 s for 15 % of samples at the captain's RTF, 0 % on an idle box), and while the
base is rotating a stale-then-fresh transition reads as a large heading error that the filter
never made.

This composes the same quantity fresh instead of trusting the cached composition:

    H(t) = wrap( mo_yaw + fo_yaw - truth_yaw(t) )

`map->odom` is the only part of the chain the localizer controls, and it is a correction that
stays valid between updates rather than a measurement that goes stale; `fo` is
`/odom_filtered`, the odom->base pose navigation actually runs on; truth is interpolated to
`fo`'s own stamp. A frozen estimate therefore cancels -- the robot's rotation appears in both
`fo_yaw` and `truth_yaw` -- while a filter that has genuinely diverged still shows up in full.
"""
import json, math, sys


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_rows(path):
    rows = []
    for line in open(path):
        d = json.loads(line)
        if not d.get("ev"):
            rows.append(d)
    return rows


def truth_track(rows):
    """(stamp, yaw, x, y) samples of ground truth, from each row's own truth lookup."""
    t = [
        (r["est_stamp"], r["tyaw"], r["tx"], r["ty"])
        for r in rows
        if "est_stamp" in r and "tyaw" in r
    ]
    t.sort()
    out = []
    for s in t:
        if not out or s[0] > out[-1][0]:
            out.append(s)
    return out


def interp(track, when):
    if not track or when < track[0][0] or when > track[-1][0]:
        return None
    lo, hi = 0, len(track) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if track[mid][0] <= when:
            lo = mid
        else:
            hi = mid
    a, b = track[lo], track[hi]
    span = b[0] - a[0]
    f = 0.0 if span <= 0 else (when - a[0]) / span
    return (
        a[1] + f * wrap(b[1] - a[1]),
        a[2] + f * (b[2] - a[2]),
        a[3] + f * (b[3] - a[3]),
    )


def annotate(rows):
    """Add h_fresh (rad) and est_age (s) to every row that can carry them."""
    tr = truth_track(rows)
    for r in rows:
        r["est_age"] = (r["t"] - r["est_stamp"]) if "est_stamp" in r else None
        if "mo_yaw" not in r or "fo" not in r:
            continue
        g = interp(tr, r["fo"]["stamp"])
        if g is None:
            continue
        r["h_fresh"] = wrap(r["mo_yaw"] + r["fo"]["yaw"] - g[0])
    return rows


if __name__ == "__main__":
    import statistics as st

    D = math.degrees
    for p in sys.argv[1:]:
        rows = annotate(load_rows(p))
        h = [abs(D(r["h_fresh"])) for r in rows if "h_fresh" in r]
        e = [abs(D(r["err_yaw"])) for r in rows if "err_yaw" in r]
        both = [
            (abs(D(r["h_fresh"])), abs(D(r["err_yaw"])), r.get("est_age", 0))
            for r in rows
            if "h_fresh" in r and "err_yaw" in r
        ]
        fresh = [x for x in both if x[2] is not None and x[2] < 0.5]
        print(f"\n{p}: {len(h)} rows with h_fresh")
        print(
            f"  |err_yaw| median {st.median(e):.2f}d  p99 {sorted(e)[int(.99*len(e))]:.2f}d  max {max(e):.2f}d"
        )
        print(
            f"  |h_fresh| median {st.median(h):.2f}d  p99 {sorted(h)[int(.99*len(h))]:.2f}d  max {max(h):.2f}d"
        )
        if fresh:
            d = [abs(a - b) for a, b, _ in fresh]
            print(
                f"  on rows where the lookup was FRESH (<0.5 s): |h_fresh - err_yaw| "
                f"median {st.median(d):.2f}d  p95 {sorted(d)[int(.95*len(d))]:.2f}d  (n={len(fresh)})"
            )
