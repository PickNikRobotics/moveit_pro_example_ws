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

"""The Nav2 footprint of both costmaps, and against the store map. No ROS."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = PACKAGE / "params" / "nav2_params.yaml"
COSTMAPS = ("local_costmap", "global_costmap")
# Nav2's footprint_padding when a costmap leaves it unset.
NAV2_DEFAULT_PADDING = 0.01
# Interior sample spacing when testing the footprint against the map, in metres.
SAMPLE_STEP = 0.01
TURN_STEP_DEG = 3


def costmap(params, name):
    return params[name][name]["ros__parameters"]


def footprint(params, name):
    """The costmap's footprint polygon, with Nav2's padding applied, in base_link."""
    costmap_params = costmap(params, name)
    padding = costmap_params.get("footprint_padding", NAV2_DEFAULT_PADDING)
    points = yaml.safe_load(costmap_params["footprint"])
    return np.array(
        [
            [x + math.copysign(padding, x), y + math.copysign(padding, y)]
            for x, y in points
        ]
    )


def test_costmaps_share_the_a300_footprint():
    market = yaml.safe_load(PARAMS.read_text())
    local, global_ = (footprint(market, name) for name in COSTMAPS)
    assert np.array_equal(local, global_)


def store_map():
    meta = yaml.safe_load((PACKAGE / "maps" / "market.yaml").read_text())
    pixels = np.array(Image.open(PACKAGE / "maps" / meta["image"]).convert("L"))
    occupied = (255 - pixels) / 255.0 > meta["occupied_thresh"]
    return occupied[::-1], meta["resolution"], meta["origin"][:2]


def aisle_goals():
    goals = {}
    for path in sorted((PACKAGE / "objectives").glob("navigate_to_aisle_*.xml")):
        goal = ET.parse(path).find(".//Action[@ID='CreatePoseStamped']")
        x, y, _ = map(float, goal.get("position_xyz").split(";"))
        qz, qw = map(float, goal.get("orientation_xyzw").split(";")[2:])
        goals[path.stem] = (x, y, 2 * math.atan2(qz, qw))
    return goals


def interior_samples(polygon):
    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
    xs, ys = np.meshgrid(
        np.arange(lo[0], hi[0] + SAMPLE_STEP, SAMPLE_STEP),
        np.arange(lo[1], hi[1] + SAMPLE_STEP, SAMPLE_STEP),
    )
    points = np.stack([xs.ravel(), ys.ravel()], axis=1)
    inside = np.zeros(len(points), dtype=bool)
    for (x1, y1), (x2, y2) in zip(polygon, np.roll(polygon, -1, axis=0)):
        crosses = (y1 > points[:, 1]) != (y2 > points[:, 1])
        with np.errstate(divide="ignore", invalid="ignore"):
            edge_x = (x2 - x1) * (points[:, 1] - y1) / (y2 - y1) + x1
        inside ^= crosses & (points[:, 0] < edge_x)
    return points[inside]


def test_aisle_goals_clear_the_racks_with_the_a300():
    """At each goal, and on a quarter turn into it from either aisle direction."""
    occupied, resolution, (origin_x, origin_y) = store_map()
    samples = interior_samples(
        footprint(yaml.safe_load(PARAMS.read_text()), "global_costmap")
    )
    goals = aisle_goals()
    assert goals
    hits = []
    for name, (x, y, yaw) in goals.items():
        for turn in range(-90, 91, TURN_STEP_DEG):
            heading = yaw + math.radians(turn)
            c, s = math.cos(heading), math.sin(heading)
            world_x = x + c * samples[:, 0] - s * samples[:, 1]
            world_y = y + s * samples[:, 0] + c * samples[:, 1]
            columns = ((world_x - origin_x) / resolution).astype(int)
            rows = ((world_y - origin_y) / resolution).astype(int)
            if occupied[rows, columns].any():
                hits.append((name, turn))
    assert not hits, f"The A300 footprint overlaps a rack (goal, turn in deg): {hits}"


def test_inflation_radius_is_the_circumscribed_radius_rounded_up():
    """Nav2's potential-field shortcut needs it; rounded up to the next 0.1 m."""
    params = yaml.safe_load(PARAMS.read_text())
    for name in COSTMAPS:
        radius = float(np.hypot(*footprint(params, name).T).max())
        expected = math.ceil(round(radius * 10, 6)) / 10
        assert costmap(params, name)["inflation_layer"][
            "inflation_radius"
        ] == pytest.approx(expected), name


def test_demo_b2_south_run_keeps_the_a300_footprint_in_free_space():
    """Check the shelf-side run continuously, including between navigation goals."""
    tree = ET.parse(PACKAGE / "objectives" / "demo_d4_b2_south_return.xml")
    poses = tree.findall(".//Action[@ID='CreatePoseStamped']")
    run = []
    for pose in poses:
        x, y, _ = map(float, pose.get("position_xyz").split(";"))
        if -21.0 <= x <= 3.0 and abs(y - 10.6) < 1e-6:
            run.append((x, y))
    assert len(run) >= 2, "The demo must include a shelf-side B2 run."
    occupied, resolution, origin = store_map()
    samples = interior_samples(
        footprint(yaml.safe_load(PARAMS.read_text()), "global_costmap")
    )
    for start, end in zip(run, run[1:]):
        for fraction in np.linspace(0, 1, int(math.dist(start, end) / resolution) + 2):
            centre = np.array(start) * (1 - fraction) + np.array(end) * fraction
            # The base drives west, so rotate its footprint by pi.
            world = centre - samples
            columns = ((world[:, 0] - origin[0]) / resolution).astype(int)
            rows = ((world[:, 1] - origin[1]) / resolution).astype(int)
            assert not occupied[
                rows, columns
            ].any(), f"B2 demo footprint overlaps an obstacle at {centre.tolist()}"


def test_demo_does_not_reseed_localization_after_leaving_the_dock():
    """Navigation must use lidar localization throughout both route variants."""
    for filename in (
        "demo_d4_b2_south_return.xml",
        "continue_b2_south_return_d4.xml",
    ):
        actions = list(ET.parse(PACKAGE / "objectives" / filename).iter("Action"))
        first_navigation = next(
            i
            for i, action in enumerate(actions)
            if action.get("ID")
            in ("NavigateToPoseAction", "NavigateThroughPosesAction")
        )
        assert all(
            action.get("ID") != "SetInitialPose"
            for action in actions[first_navigation:]
        ), filename
        if filename.startswith("continue_"):
            assert all(action.get("ID") != "SetInitialPose" for action in actions)
            assert all(
                action.get("source_frame_id") != "chassis_link_ground_truth"
                for action in actions
            )


def test_dock_demo_undocks_and_refines_before_navigation():
    """A dock departure must back straight out and stop before relocalizing."""
    sequence = ET.parse(PACKAGE / "objectives/demo_d4_b2_south_return.xml").find(
        ".//BehaviorTree/Control"
    )
    steps = list(sequence)
    undock = next(i for i, n in enumerate(steps) if n.get("ID") == "Undock From D4")
    refine = next(
        i for i, n in enumerate(steps) if n.get("ID") == "Refine Localization In Place"
    )
    navigate = next(
        i for i, n in enumerate(steps) if n.get("ID") == "NavigateToPoseAction"
    )
    assert undock < refine < navigate
    subtree = ET.parse(PACKAGE / "objectives/undock_from_d4.xml")
    twists = subtree.findall(".//Action[@ID='CreateTwistStamped']")
    assert all(n.get("angular_velocity_xyz") == "0;0;0" for n in twists)
    assert float(twists[0].get("linear_velocity_xyz").split(";")[0]) < 0
    assert twists[-1].get("linear_velocity_xyz") == "0;0;0"


def test_waypoint_pruning_cannot_skip_a_passed_goal_at_the_speed_limit() -> None:
    """A robot crossing a waypoint must get a pruning tick inside its acceptance radius."""
    params = yaml.safe_load(PARAMS.read_text())
    tree = ET.parse(PACKAGE / "params/navigate_through_poses.xml")
    parents = {child: parent for parent in tree.iter() for child in parent}
    prune = tree.find(".//RemovePassedGoals")
    radius = float(prune.get("radius"))
    period = params["bt_navigator"]["ros__parameters"]["bt_loop_duration"] / 1000.0
    ancestor = parents.get(prune)
    while ancestor is not None:
        if ancestor.tag == "RateController":
            period = max(period, 1.0 / float(ancestor.get("hz")))
        ancestor = parents.get(ancestor)
    speed = params["controller_server"]["ros__parameters"]["FollowPath"]["vx_max"]
    assert period * speed < 2 * radius, (
        f"The base can travel {period * speed:.2f} m between pruning ticks, "
        f"skipping the waypoint's {2 * radius:.2f} m acceptance window."
    )
