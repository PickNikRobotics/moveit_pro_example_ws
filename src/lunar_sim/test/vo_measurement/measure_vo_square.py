#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Measure an existing lunar_sim VO run; optionally save a ground-truth/VO SVG plot.

Wait for Ready, run VO Estimation Run in the Desktop App, then press Enter here.
JSON metrics go to stdout; --plot route.svg saves the paired camera paths.
Never starts, drives, or cancels the robot. Setup and example visualization:
https://docs.picknik.ai/how_to/mobile_navigation/stereo_visual_odometry/
"""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import select
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from tf_lookup import dynamic_tf_qos_profile, wait_for_transform
from trajectory_metrics import (
    PoseSample,
    QualificationResult,
    RigidTransform,
    evaluate_trajectory,
    route_interval_from_commands,
)


def pose_sample(message: Odometry) -> PoseSample:
    """Convert source-stamped odometry into scorer input."""
    pose = message.pose.pose
    stamp = message.header.stamp
    return PoseSample(
        stamp.sec + stamp.nanosec * 1e-9,
        np.array([pose.position.x, pose.position.y, pose.position.z]),
        np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        ),
    )


def measure(timeout: float) -> QualificationResult:
    """Observe an existing Runtime without sending goals, services, or motion commands."""
    truth: list[PoseSample] = []
    vo: list[PoseSample] = []
    commands: list[tuple[float, bool]] = []
    recording = False
    with ExitStack() as cleanup:
        rclpy.init()
        cleanup.callback(rclpy.try_shutdown)
        node = rclpy.create_node("vo_square_measurement")
        cleanup.callback(node.destroy_node)
        buffer = Buffer(cache_time=Duration(seconds=timeout + 10))
        listener = TransformListener(
            buffer, node, spin_thread=False, qos=dynamic_tf_qos_profile()
        )
        cleanup.callback(listener.unregister)

        def record_truth(message: Odometry) -> None:
            if recording:
                if (message.header.frame_id, message.child_frame_id) != (
                    "mj_world",
                    "chassis_link",
                ):
                    raise ValueError("Ground truth must use mj_world -> chassis_link.")
                truth.append(pose_sample(message))

        def record_vo(message: Odometry) -> None:
            if recording:
                if (message.header.frame_id, message.child_frame_id) != (
                    "camera_odom",
                    "oakd_left_optical_frame",
                ):
                    raise ValueError(
                        "VO must use camera_odom -> oakd_left_optical_frame."
                    )
                vo.append(pose_sample(message))

        def record_command(message: TwistStamped) -> None:
            if recording:
                stamp = message.header.stamp
                twist = message.twist
                moving = any(
                    abs(value) > 1e-9
                    for value in (
                        twist.linear.x,
                        twist.linear.y,
                        twist.linear.z,
                        twist.angular.x,
                        twist.angular.y,
                        twist.angular.z,
                    )
                )
                commands.append((stamp.sec + stamp.nanosec * 1e-9, moving))

        for message_type, topic, callback, qos in (
            (Odometry, "/ground_truth/odom", record_truth, qos_profile_sensor_data),
            (
                Odometry,
                "/stereo_visual_odometry/odometry",
                record_vo,
                qos_profile_sensor_data,
            ),
            (TwistStamped, "/cmd_vel", record_command, 10),
        ):
            subscription = node.create_subscription(message_type, topic, callback, qos)
            cleanup.callback(node.destroy_subscription, subscription)

        deadline = time.monotonic() + 10
        while not all(
            node.count_publishers(topic)
            for topic in (
                "/ground_truth/odom",
                "/stereo_visual_odometry/odometry",
            )
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Missing measurement publishers. Start lunar_sim with VO enabled and use the Runtime's ROS environment/domain."
                )
            rclpy.spin_once(node, timeout_sec=0.05)

        print(
            "Ready. Run VO Estimation Run in the Desktop App. Press Enter AFTER it finishes (success, failure, or cancellation).",
            flush=True,
            file=sys.stderr,
        )
        recording = True
        deadline = time.monotonic() + timeout
        while not select.select([sys.stdin], [], [], 0)[0]:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "Measurement timed out; no robot commands were sent. Retry with a longer --timeout if needed."
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        sys.stdin.readline()

        deadline = time.monotonic() + 5
        while not (
            vo
            and truth
            and commands
            and not commands[-1][1]
            and truth[-1].stamp >= vo[-1].stamp
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Missing VO/truth samples or final STOP. Check the Objective result and stereo VO diagnostics."
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        interval = route_interval_from_commands(commands)
        anchor = next(
            (
                sample.stamp
                for sample in vo
                if interval.start_stamp <= sample.stamp <= interval.end_stamp
            ),
            None,
        )
        if anchor is None:
            raise ValueError("No VO poses were published during the measured motion.")
        transform = wait_for_transform(
            node,
            buffer,
            "chassis_link",
            "oakd_left_optical_frame",
            Time(nanoseconds=round(anchor * 1e9)),
            timeout_sec=5,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        result = evaluate_trajectory(
            vo,
            truth,
            RigidTransform(
                np.array([translation.x, translation.y, translation.z]),
                np.array([rotation.x, rotation.y, rotation.z, rotation.w]),
            ),
            interval,
        )
        return result


def report_result(result: QualificationResult, plot: Path | None) -> int:
    """Emit metrics before attempting an optional plot so write errors cannot lose them."""
    print(
        json.dumps(
            {
                "accepted": result.accepted,
                "violations": result.violations,
                "metrics": result.metrics,
            },
            indent=2,
            allow_nan=False,
        ),
        flush=True,
    )
    if plot is not None:
        from trajectory_plot import save_plot

        try:
            save_plot(result, plot)
        except (OSError, ValueError) as error:
            print(
                f"Metrics saved, but trajectory plot failed: {error}", file=sys.stderr
            )
            return 1
        print(f"Saved trajectory plot to {plot}", file=sys.stderr)
    return 0 if result.accepted else 1


def main() -> int:
    """Print measurement results or a diagnostic; never control the Runtime."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeout",
        type=float,
        default=300,
        help="Maximum recording duration in seconds (default: 300).",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        help="Save overhead camera trajectories and a finish close-up to a new SVG file.",
    )
    args = parser.parse_args()
    if args.plot is not None:
        if args.plot.suffix.lower() != ".svg":
            parser.error("--plot must name an .svg file")
        if args.plot.exists():
            parser.error(
                "--plot must name a new file; existing files are not overwritten"
            )
        if not args.plot.parent.is_dir():
            parser.error("--plot parent directory must already exist")
    if not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if not sys.stdin.isatty():
        parser.error(
            "Run interactively; Enter ends recording after the Objective finishes."
        )
    try:
        return report_result(measure(args.timeout), args.plot)
    except (RuntimeError, ValueError, TimeoutError, OSError) as error:
        print(f"VO measurement failed: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(
            "Measurement stopped. The script did not cancel the Objective.",
            file=sys.stderr,
        )
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
