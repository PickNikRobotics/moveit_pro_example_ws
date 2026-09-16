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

"""AMCL must weight particles on the whole merged scan, not a slice of it.

`max_beams` and the merged scan's ray count live in two different files with nothing
tying them together: the count follows from the `dual_laser_merger` parameters in
`launch/sim/localization_launch.py`, and `max_beams` sits in `params/nav2_params.yaml`.

Getting this wrong is silent and expensive. beluga subsamples with `take_evenly()` over
the *raw* ranges array, before the invalid returns are filtered out
(`beluga_ros/laser_scan.hpp`), and the hangar scans are deliberately sparse — open space
plus self-hit filtering leave finite returns surrounded by `inf`, which is why the
merger's shadow and average filters are disabled. So a `max_beams` below the ray count
does not cost you a proportional number of beams; it costs you that fraction of the
*finite* returns too, and it costs most where returns are sparsest, which is the
featureless stretch where localization already has the least to work with.

`take_evenly()` returns the whole range whenever the requested count is at least the
range's size, so the check is a lower bound: any value at or above the ray count means
"every ray".
"""

import ast
import math
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent.parent
LAUNCH = PKG / "launch" / "sim" / "localization_launch.py"
NAV2_PARAMS = PKG / "params" / "nav2_params.yaml"


def _merger_params():
    """Pull the dual_laser_merger parameter dict out of the launch file's AST.

    The launch file is not importable on its own — it builds substitutions against a
    launch context — so read it as source, the way test_base_geometry.py reads the
    description sources.
    """
    tree = ast.parse(LAUNCH.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) != "ComposableNode":
            continue
        kwargs = {kw.arg: kw.value for kw in node.keywords}
        name = kwargs.get("name")
        if not isinstance(name, ast.Constant) or name.value != "dual_laser_merger":
            continue
        # parameters=[{...}] — a single literal dict of merger settings.
        (params_dict,) = kwargs["parameters"].elts
        out = {}
        for key, value in zip(params_dict.keys, params_dict.values):
            try:
                out[key.value] = ast.literal_eval(value)
            except ValueError:
                pass  # LaunchConfiguration and friends; not a literal we need
        return out
    raise AssertionError(f"no dual_laser_merger ComposableNode found in {LAUNCH}")


def _amcl_params():
    return yaml.safe_load(NAV2_PARAMS.read_text())["amcl"]["ros__parameters"]


def test_max_beams_covers_every_ray_of_the_merged_scan():
    """A `max_beams` below the ray count discards finite returns, not just rays."""
    merger = _merger_params()
    span = merger["angle_max"] - merger["angle_min"]
    # The merger lays out one ray per increment across the span, plus the closing ray.
    rays = math.ceil(span / merger["angle_increment"]) + 1

    max_beams = _amcl_params()["max_beams"]
    assert max_beams >= rays, (
        f"amcl max_beams={max_beams} subsamples a {rays}-ray /scan_merged "
        f"({math.degrees(span):.0f} deg at {merger['angle_increment']} rad). "
        "beluga's take_evenly() runs before invalid returns are dropped, so this "
        "throws away that share of the finite returns as well — worst exactly where "
        "the scan is sparsest."
    )


def test_amcl_laser_range_window_matches_the_merged_scan():
    """AMCL's range gate must not be narrower than the scan the merger publishes.

    beluga clamps to the intersection of the two (`beluga_ros/laser_scan.hpp`), so an
    AMCL window tighter than the merger's silently discards returns the scan did carry.
    `laser_max_range` does double duty as the likelihood field's `max_laser_distance`,
    the denominator of the `z_rand` background term, so it is not free to overshoot
    either.
    """
    merger = _merger_params()
    amcl = _amcl_params()
    assert amcl["laser_max_range"] >= merger["range_max"]
    assert amcl["laser_min_range"] <= merger["range_min"]
