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

"""Opt-in live regression for MPPI's exact-timestamp map-to-odom lookup.

Run against an isolated lunar backend with localization=true and slam=false:
LUNAR_LOCALIZATION_LIVE_TEST=true python3 -m pytest test_localization_tf_freshness.py
No ground-truth subscription, pose correction, or motion command is used.
"""

import os
import threading
import time

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.time import Time
import tf2_geometry_msgs  # noqa: F401 - Registers PoseStamped's TF transform implementation.
from tf2_ros import Buffer, TransformException, TransformListener


@pytest.mark.skipif(
    os.environ.get("LUNAR_LOCALIZATION_LIVE_TEST") != "true",
    reason="Requires an isolated running lunar localization backend",
)
def test_localization_tf_covers_controller_pose_timestamps():
    """Every controller-time lookup succeeds within MPPI's configured 0.1 s deadline."""
    rclpy.init()
    node = rclpy.create_node("lunar_localization_tf_regression")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin)
    thread.start()
    try:
        deadline = time.monotonic() + 120.0
        while True:
            if buffer.can_transform("odom", "footprint", Time()):
                stamp = buffer.lookup_transform(
                    "odom", "footprint", Time()
                ).header.stamp
                if buffer.can_transform("map", "odom", Time.from_msg(stamp)):
                    break
            assert time.monotonic() < deadline, "Localization TF did not initialize"
            time.sleep(0.05)
        names = set(node.get_node_names())
        assert {"amcl", "map_server", "state_estimator"} <= names
        assert "slam_toolbox" not in names

        failures = []
        lookups = 0
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            start = time.monotonic()
            odom_pose = buffer.lookup_transform("odom", "footprint", Time())
            goal = PoseStamped()
            goal.header.frame_id = "map"
            # MPPI transforms its map-frame goal at the latest controller robot-pose stamp,
            # not time zero: a latest-only lookup would conceal the reproduced failure.
            goal.header.stamp = odom_pose.header.stamp
            goal.pose.orientation.w = 1.0
            try:
                transformed = buffer.transform(
                    goal, "odom", timeout=Duration(seconds=0.1)
                )
                assert transformed.header.frame_id == "odom"
            except TransformException as exc:
                failures.append(str(exc))
            lookups += 1
            time.sleep(max(0.0, 0.05 - (time.monotonic() - start)))
        assert lookups >= 100, f"Only {lookups} controller-time lookups ran"
        assert not failures, (
            f"{len(failures)}/{lookups} controller-time map-to-odom lookups failed; "
            f"verify hardware EGL and lidar delivery before tuning TF: {failures[:3]}"
        )
    finally:
        executor.shutdown()
        thread.join(timeout=5.0)
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()
