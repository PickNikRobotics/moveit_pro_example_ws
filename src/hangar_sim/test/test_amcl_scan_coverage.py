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

SCOPE, plainly: these checks read `params/nav2_params.yaml` — machine-consumed config —
and assert AMCL's own settings against the merged scan's measured shape, recorded here
as constants. They guard the AMCL parameters against being narrowed. They do NOT prove
that the published `/scan_merged` carries that many rays, or that its range window is
what it is; nothing here runs the merger. Re-measure the constants below if the
`dual_laser_merger` configuration in `launch/sim/localization_launch.py` changes.
"""

from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
NAV2_PARAMS = PKG / "params" / "nav2_params.yaml"

# Measured live off the running stack at the robot's spawn pose: `/scan_merged` carries
# 723 rays, of which 283 are finite and in range. Not re-derived from the merger's
# angle_min/angle_max/angle_increment — ceil(span / increment) + 1 gives 724, which is
# not what the merger actually publishes.
MERGED_SCAN_RAYS = 723
# The same measurement's range window, i.e. the merger's range_min / range_max.
MERGED_SCAN_RANGE_MIN = 0.05
MERGED_SCAN_RANGE_MAX = 25.0


def _amcl_params():
    return yaml.safe_load(NAV2_PARAMS.read_text())["amcl"]["ros__parameters"]


def test_max_beams_covers_every_ray_of_the_merged_scan():
    """A `max_beams` below the ray count discards finite returns, not just rays."""
    max_beams = _amcl_params()["max_beams"]
    assert max_beams >= MERGED_SCAN_RAYS, (
        f"amcl max_beams={max_beams} subsamples a {MERGED_SCAN_RAYS}-ray /scan_merged. "
        "beluga's take_evenly() runs before invalid returns are dropped, so this "
        "throws away that share of the finite returns as well — worst exactly where "
        "the scan is sparsest."
    )


def test_amcl_laser_range_window_matches_the_merged_scan():
    """AMCL's range gate must match the scan the merger publishes.

    beluga clamps to the intersection of the two (`beluga_ros/laser_scan.hpp`), so an
    AMCL window tighter than the merger's silently discards returns the scan did carry —
    `laser_min_range` is therefore a bound. `laser_max_range` is not: it does double duty
    as the likelihood field's `max_laser_distance`, the denominator of the `z_rand`
    background term, so overshooting the merger's `range_max` reweights every particle
    just as silently. It has to be the merger's number exactly.
    """
    amcl = _amcl_params()
    assert amcl["laser_max_range"] == pytest.approx(MERGED_SCAN_RANGE_MAX), (
        f"amcl laser_max_range={amcl['laser_max_range']} must equal the merged scan's "
        f"range_max {MERGED_SCAN_RANGE_MAX}: it is also the likelihood field's "
        "max_laser_distance and the z_rand background denominator."
    )
    assert amcl["laser_min_range"] <= MERGED_SCAN_RANGE_MIN
