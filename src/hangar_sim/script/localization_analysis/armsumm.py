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

"""Summarise one measurement arm: starts, excursions, rate, peak heading error.

Excursion detection is excur.py's: a 5-sample rolling median of |err_yaw| over the
threshold, corroborated by AMCL's own /pose (which never passes through a composed TF
lookup), so a single bad map->base sample cannot manufacture one.

An excursion is attributed to the Objective start whose window it falls in -- from that
start's re-seed (or its accept, if the arm has no re-seed) to the next start's.

Also reports the captain's pass condition directly: for every re-seed, the cloud's
HEADING spread just before against just after.
"""
import json, math, statistics as st, sys, os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seed_constants import COV_TOL, RESCUE_XY, RESCUE_YAW  # noqa: E402

THRESH = float(os.environ.get("THRESH", "20.0"))


def wr(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(p):
    rows, evs = [], []
    for line in open(p):
        d = json.loads(line)
        if d.get("ev") == "initialpose":
            evs.append(d)
        elif d.get("ev"):
            continue
        elif "err_yaw" in d:
            rows.append(d)
    return rows, evs


def med(v, k=5):
    h = k // 2
    return [st.median(v[max(0, i - h) : i + h + 1]) for i in range(len(v))]


def episodes(rows, thresh):
    if not rows:
        return []
    m = med([abs(math.degrees(r["err_yaw"])) for r in rows])
    runs, cur = [], []
    for i, v in enumerate(m):
        if v >= thresh:
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    eps = []
    for idx in runs:
        i0, i1 = idx[0], idx[-1]
        pk = max(range(i0, i1 + 1), key=lambda i: m[i])
        agree = 0.0
        for i in range(i0, i1 + 1):
            ap = rows[i].get("ap") or {}
            e = ap.get("err_yaw")
            if e is None and "yaw" in ap and "tyaw" in rows[i]:
                e = wr(ap["yaw"] - rows[i]["tyaw"])
            if e is not None:
                agree = max(agree, abs(math.degrees(e)))
        if agree < m[pk] / 2.0:
            continue  # not corroborated -> not counted
        r = rows[pk]
        c = r.get("cloud", {})
        eps.append(
            dict(
                t=rows[i0]["t"],
                t_end=rows[i1]["t"],
                dur=rows[i1]["t"] - rows[i0]["t"],
                pk=m[pk],
                raw=abs(math.degrees(r["err_yaw"])),
                ap=agree,
                errpos=r.get("err_d", float("nan")),
                n=c.get("n"),
                r95=c.get("r95"),
                yawr95=math.degrees(c.get("yaw_r95", float("nan"))),
            )
        )
    return eps


def is_rescue(e):
    xx, aa = e.get("cov_xx"), e.get("cov_aa")
    if xx is None or aa is None:
        return False
    return abs(xx - RESCUE_XY) < COV_TOL and abs(aa - RESCUE_YAW) < COV_TOL


def arm_key(e):
    """An arm is identified by the covariance it seeded with -- the measurement, not bookkeeping."""
    xx, aa = e.get("cov_xx"), e.get("cov_aa")
    if xx is None or aa is None:
        return None
    return (round(xx, 6), round(aa, 7))


def seed_effect(rows, evs, pre_s=6.0, post_s=4.0):
    """For each re-seed: heading spread and cloud radius just before vs just after."""
    out = []
    for e in evs:
        s = e["stamp"]
        pre = [r for r in rows if s - pre_s <= r["t"] < s - 0.2 and "cloud" in r]
        post = [r for r in rows if s + 0.4 < r["t"] <= s + post_s and "cloud" in r]
        if not pre or not post:
            continue

        def m(v, k):
            xs = [x["cloud"][k] for x in v if k in x["cloud"]]
            return st.median(xs) if xs else float("nan")

        out.append(
            dict(
                t=s,
                wall=e.get("wall"),
                cov_xx=e.get("cov_xx"),
                cov_aa=e.get("cov_aa"),
                pre_yawr95=math.degrees(m(pre, "yaw_r95")),
                post_yawr95=math.degrees(m(post, "yaw_r95")),
                pre_r95=m(pre, "r95"),
                post_r95=m(post, "r95"),
                pre_n=m(pre, "n"),
                post_n=m(post, "n"),
                seed_err_d=e.get("seed_err_d"),
                seed_err_yaw=e.get("seed_err_yaw"),
            )
        )
    return out


def main():
    for path in sys.argv[1:]:
        label = os.path.basename(path).replace(".jsonl", "")
        rows, evs = load(path)
        starts_path = path.replace(".jsonl", ".starts")
        starts = []
        if os.path.exists(starts_path):
            starts = [json.loads(l) for l in open(starts_path)]
        if not rows:
            print(f"{label}: no rows")
            continue
        t0 = rows[0]["t"]
        eps = episodes(rows, THRESH)
        # window each start: accept -> next accept (wall clock; rows carry sim-ish t, so
        # align on the recorder's own 'wall' field)
        wall = {r["t"]: r.get("wall") for r in rows}
        bounds = []
        for i, s in enumerate(starts):
            if not s.get("accepted"):
                continue
            a = s.get("t_accept", s["t_send"])
            b = (
                starts[i + 1].get("t_accept", starts[i + 1]["t_send"])
                if i + 1 < len(starts)
                else 1e18
            )
            bounds.append((s["i"], s.get("wp"), a, b))

        def start_of(ep_t):
            w = wall.get(ep_t)
            if w is None:
                return None
            for idx, wp, a, b in bounds:
                if a <= w < b:
                    return idx
            return None

        hit = {}
        for e in eps:
            k = start_of(e["t"])
            hit.setdefault(k, []).append(e)
        if not bounds:
            # no driver log (e.g. the captain's own session): window by re-seed instead
            ws = sorted(e["wall"] for e in evs)
            bounds = [
                (i, None, w, (ws[i + 1] if i + 1 < len(ws) else 1e18))
                for i, w in enumerate(ws)
            ]
            hit = {}
            for e in eps:
                hit.setdefault(start_of(e["t"]), []).append(e)
        n_starts = len(bounds)
        bad = len([k for k in hit if k is not None])

        print(f"\n================ {label} ================")
        print(
            f"rows={len(rows)}  duration={rows[-1]['t']-t0:.0f}s  re-seeds={len(evs)}  "
            f"Objective starts accepted={n_starts}"
        )
        print(
            f"EXCURSIONS (|err_yaw| >= {THRESH:.0f} deg, corroborated by /pose): {len(eps)}  "
            f"in {bad} distinct starts  -> rate {bad}/{n_starts} = "
            f"{(100.0*bad/n_starts if n_starts else 0):.0f}% of starts"
        )
        if eps:
            print(
                f"  peak heading error over arm: {max(e['pk'] for e in eps):.1f} deg   "
                f"max position error at a peak: {max(e['errpos'] for e in eps):.2f} m"
            )
            print(
                f"  {'t_rel':>8} {'start':>5} {'dur':>5} {'peak':>7} {'/pose':>7} {'errPos':>7} "
                f"{'cloud n':>7} {'r95':>6} {'yawR95':>7}"
            )
            for e in eps:
                print(
                    f"  {e['t']-t0:8.1f} {str(start_of(e['t'])):>5} {e['dur']:5.1f} "
                    f"{e['pk']:6.1f}d {e['ap']:6.1f}d {e['errpos']:6.2f}m "
                    f"{e['n'] or -1:7d} {e['r95'] or float('nan'):6.2f} {e['yawr95']:6.1f}d"
                )
        for t in (5.0, 10.0, 45.0, 90.0):
            print(f"  (at >= {t:4.0f} deg: {len(episodes(rows, t))} episodes)")

        se_all = seed_effect(rows, evs)
        # An interleaved session writes several different covariances into one file. Pooling
        # them would report a median over mixed populations under one arm's label, which is the
        # opposite of what the A/B exists to measure -- so group by the seeded covariance and
        # drop rescues, which are not an Objective's own seed.
        n_rescue = len([x for x in se_all if is_rescue(x)])
        se_arms = [x for x in se_all if not is_rescue(x) and arm_key(x) is not None]
        groups = {}
        for x in se_arms:
            groups.setdefault(arm_key(x), []).append(x)
        if n_rescue:
            print(
                f"\n  ({n_rescue} rescue re-seed(s) excluded from the per-arm statistics below)"
            )

        def starts_for(seeds):
            return [
                idx
                for idx, wp, a, b in bounds
                if any(x["wall"] is not None and a <= x["wall"] < b for x in seeds)
            ]

        # The driver records the arm it ran per start; that is ground truth. Attribution by
        # seed window is only a measurement of it, and the two can disagree -- t_accept is
        # stamped client-side while SetInitialPose is the server's first action, so a seed can
        # land just outside its own start's window. Never infer an arm from a missed
        # attribution: that would manufacture a 'noseed' control in a session that ran none.
        arm_of = {s["i"]: s.get("arm") for s in starts if s.get("accepted")}
        has_arm_labels = any(v for v in arm_of.values())
        seeded_starts = set(starts_for(se_arms))
        noseed_starts = [idx for idx, wp, a, b in bounds if arm_of.get(idx) == "noseed"]
        multi_arm = len(groups) + (1 if noseed_starts else 0) > 1
        for key in sorted(groups, reverse=True):
            se = groups[key]
            widened = [s for s in se if s["post_yawr95"] > s["pre_yawr95"]]
            print(
                f"\n  RE-SEED EFFECT on the cloud's HEADING spread  "
                f"(seed sigma_yaw={math.degrees(math.sqrt(key[1])):.2f} deg, "
                f"sigma_xy={math.sqrt(key[0]):.3f} m)"
            )
            print(f"  widened the heading spread in {len(widened)}/{len(se)} re-seeds")
            pre = [s["pre_yawr95"] for s in se]
            post = [s["post_yawr95"] for s in se]
            print(
                f"  yaw_r95  pre  median {st.median(pre):5.2f} deg   "
                f"post median {st.median(post):5.2f} deg   "
                f"ratio {st.median(post)/st.median(pre):.2f}x"
            )
            pr = [s["pre_r95"] for s in se]
            po = [s["post_r95"] for s in se]
            print(
                f"  r95      pre  median {st.median(pr):5.2f} m     "
                f"post median {st.median(po):5.2f} m     "
                f"ratio {st.median(po)/st.median(pr):.2f}x"
            )
            pn = [s["pre_n"] for s in se]
            pon = [s["post_n"] for s in se]
            print(
                f"  n        pre  median {st.median(pn):5.0f}       post median {st.median(pon):5.0f}"
            )
            # Attribute each start to an arm through the seed that fired inside its window,
            # so the interleaved A/B finally reports the per-arm rate it exists to produce.
            arm_starts = starts_for(se)
            n_bad = len([k for k in arm_starts if hit.get(k)])
            if multi_arm and arm_starts:
                print(
                    f"  excursions in this arm: {n_bad}/{len(arm_starts)} starts = "
                    f"{100.0*n_bad/len(arm_starts):.0f}%"
                )

        # The 'noseed' control emits no /initialpose at all, so it cannot be found by covariance.
        # It is reported when the driver log says it ran, and not otherwise.
        if multi_arm and noseed_starts:
            n_bad = len([k for k in noseed_starts if hit.get(k)])
            print(
                "\n  RE-SEED EFFECT on the cloud's HEADING spread  "
                "(seed sigma_yaw=n/a, sigma_xy=n/a -- the 'noseed' control, no re-seed fired)"
            )
            print("  widened the heading spread in 0/0 re-seeds (nothing to widen it)")
            print(
                f"  excursions in this arm: {n_bad}/{len(noseed_starts)} starts = "
                f"{100.0*n_bad/len(noseed_starts):.0f}%"
            )

        # Where the recorded arm and the observed covariance disagree, say so rather than
        # silently trusting either -- it means the per-arm rows above cannot be relied on.
        if has_arm_labels:
            missing = sorted(
                idx
                for idx, wp, a, b in bounds
                if arm_of.get(idx) not in (None, "noseed") and idx not in seeded_starts
            )
            extra = sorted(idx for idx in noseed_starts if idx in seeded_starts)
            if missing or extra:
                print(
                    "\n  LABEL MISMATCH -- the per-arm rows above may misattribute these starts:"
                )
                if missing:
                    print(
                        f"    {len(missing)} start(s) recorded as a seeding arm with no seed "
                        f"observed in their window: {missing}"
                    )
                if extra:
                    print(
                        f"    {len(extra)} start(s) recorded as 'noseed' with a seed observed "
                        f"in their window: {extra}"
                    )


if __name__ == "__main__":
    main()
