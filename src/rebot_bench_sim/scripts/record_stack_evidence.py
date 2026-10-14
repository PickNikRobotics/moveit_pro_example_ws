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
"""Record live MuJoCo cube poses and a continuous 30 fps camera video."""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import yaml

import cv2
import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from cv_bridge import CvBridge
from moveit_studio_internal_msgs.msg import MujocoQpos, MujocoQposSchema
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from rosidl_runtime_py.convert import message_to_ordereddict

from picknik_nyx_renderer.qpos_mirror import qpos_model_hash, apply_qpos


def cleanup(actions, primary_error=None):
    """Attempt every action, preserving the first error and noting later failures."""
    first_error = primary_error
    for name, action in actions:
        try:
            action()
        except BaseException as error:
            if first_error is None:
                first_error = error
            else:
                first_error.add_note(f"{name}: {type(error).__name__}: {error}")
    if primary_error is None and first_error is not None:
        raise first_error


def stack_metrics(cubes):
    red = np.array(cubes["red"]["position"])
    blue = np.array(cubes["blue"]["position"])
    tilt = {}
    for name, pose in cubes.items():
        q = np.array(pose["quaternion_wxyz"])
        q = q / np.linalg.norm(q)
        tilt[name] = math.degrees(
            math.acos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1))
        )
    xy = float(np.linalg.norm((blue - red)[:2]))
    height = float(blue[2] - red[2])
    return dict(
        horizontal_error_m=xy,
        height_difference_m=height,
        tilt_deg=tilt,
        stacked=xy <= 0.005 and abs(height - 0.05) <= 0.005 and max(tilt.values()) <= 5,
    )


class Recorder(Node):
    def __init__(self):
        super().__init__("rebot_bench_evidence")
        share = Path(get_package_share_directory("rebot_bench_sim"))
        mujoco.mj_loadAllPluginLibraries(
            str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
        )
        self.model = mujoco.MjModel.from_xml_path(str(share / "description/scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.schema_hash = qpos_model_hash(mujoco, self.model)
        shutil.copyfile(
            share / "description/bench_layout.yaml", args.output / "bench_layout.yaml"
        )
        self.grasp_proxy = bool(
            yaml.safe_load((share / "description/bench_layout.yaml").read_text()).get(
                "sim_grasp_proxy", False
            )
        )
        self.bridge = CvBridge()
        self.markers = []
        self.create_subscription(MarkerArray, "/visual_markers", self.marker, 10)
        self.samples = []
        self.frame_stamps = []
        self.last_frame = None
        self.video = None
        self.ready = False
        self.truth_file = (args.output / "qpos.jsonl").open("w")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MujocoQposSchema, "/mujoco/qpos_schema", self.schema, latched
        )
        self.create_subscription(
            MujocoQpos, "/mujoco/qpos_stamped", self.truth, latched
        )
        self.create_subscription(
            Image, "/scene_camera/color/image_raw", self.frame, qos_profile_sensor_data
        )

    def marker(self, msg):
        entries = [message_to_ordereddict(m) for m in msg.markers if m.action == 0]
        if entries:
            self.markers.extend(entries)
            (args.output / "markers.json").write_text(
                json.dumps(self.markers, indent=2)
            )
            print("MARKERS", json.dumps(entries), flush=True)

    def schema(self, msg):
        if msg.schema_hash != self.schema_hash:
            raise RuntimeError("Live simulator model differs from the loaded model")
        (args.output / "qpos_schema.json").write_text(
            json.dumps(
                dict(
                    schema_hash=msg.schema_hash,
                    schema=msg.schema,
                    publish_rate_hz=msg.publish_rate_hz,
                ),
                indent=2,
            )
        )

    def truth(self, msg):
        if msg.schema_hash != self.schema_hash:
            raise RuntimeError("Live simulator model hash changed")
        apply_qpos(self.data.qpos, msg.qpos)
        mujoco.mj_kinematics(self.model, self.data)
        cubes = {
            name: dict(
                position=self.data.body(name + "_cube").xpos.tolist(),
                quaternion_wxyz=self.data.body(name + "_cube").xquat.tolist(),
            )
            for name in ("red", "blue")
        }
        sample = dict(
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            sim_time=msg.sim_time,
            cubes=cubes,
            qpos=list(msg.qpos),
            schema_hash=msg.schema_hash,
        )
        self.samples.append(sample)
        self.truth_file.write(json.dumps(sample) + "\n")
        if self.last_frame is not None and not self.ready:
            self.ready = True
            print("READY", json.dumps(cubes), flush=True)

    def frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.frame_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if self.last_frame is None:
            cv2.imwrite(
                str(args.output / "before.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            )
            if args.video:
                self.video = subprocess.Popen(
                    [
                        "ffmpeg",
                        "-hide_banner",
                        "-loglevel",
                        "error",
                        "-f",
                        "rawvideo",
                        "-pixel_format",
                        "rgb24",
                        "-video_size",
                        f"{msg.width}x{msg.height}",
                        "-framerate",
                        "30",
                        "-i",
                        "pipe:0",
                        "-an",
                        "-c:v",
                        "libx264",
                        "-preset",
                        "veryfast",
                        "-crf",
                        "20",
                        "-pix_fmt",
                        "yuv420p",
                        str(args.output / "run.mp4"),
                    ],
                    stdin=subprocess.PIPE,
                )
        if self.video is not None:
            self.video.stdin.write(frame.tobytes())
        self.last_frame = frame

    def finish(self):
        actions = [("close truth stream", self.truth_file.close)]
        if self.video is not None:

            def wait_for_encoder():
                if self.video.wait() != 0:
                    raise RuntimeError("Video encoder failed")

            actions.extend(
                [
                    ("close video stream", self.video.stdin.close),
                    ("wait for encoder", wait_for_encoder),
                ]
            )
        cleanup(actions)
        if (
            self.last_frame is None
            or len(self.frame_stamps) < 2
            or len(self.samples) < 2
        ):
            raise RuntimeError("No live image or simulator pose evidence received")
        frame_duration = self.frame_stamps[-1] - self.frame_stamps[0]
        if frame_duration <= 0:
            raise RuntimeError(
                "Nonpositive elapsed camera frame time; invalid evidence"
            )
        cv2.imwrite(
            str(args.output / "after.png"),
            cv2.cvtColor(self.last_frame, cv2.COLOR_RGB2BGR),
        )
        end = self.samples[-1]["stamp"]
        tail = [s for s in self.samples if s["stamp"] >= end - 2]
        stable = max(
            float(
                np.linalg.norm(
                    np.array(s["cubes"][colour]["position"])
                    - np.array(tail[-1]["cubes"][colour]["position"])
                )
            )
            for s in tail
            for colour in ("red", "blue")
        )
        metrics = stack_metrics(self.samples[-1]["cubes"])
        frame_diffs = np.diff(self.frame_stamps)
        report = dict(
            sim_only_grasp_proxy=self.grasp_proxy,
            validation_scope=(
                "perception and Objective execution; contact-gated weld assists grasp"
                if self.grasp_proxy
                else "physical gripper contact"
            ),
            before=self.samples[0],
            after=self.samples[-1],
            metrics=metrics,
            tail_duration_s=tail[-1]["stamp"] - tail[0]["stamp"],
            tail_max_displacement_m=stable,
            tail_all_stacked=all(stack_metrics(s["cubes"])["stacked"] for s in tail),
            camera=dict(
                frame_count=len(self.frame_stamps),
                source_fps=(len(self.frame_stamps) - 1) / frame_duration,
                max_source_gap_s=float(max(frame_diffs)),
            ),
            wall_duration_s=self.samples[-1]["stamp"] - self.samples[0]["stamp"],
            sim_duration_s=self.samples[-1]["sim_time"] - self.samples[0]["sim_time"],
        )
        report["passed"] = (
            metrics["stacked"]
            and report["tail_all_stacked"]
            and report["tail_duration_s"] >= 1.9
            and stable <= 0.001
        )
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)


def main():
    global args
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = None
    primary_error = None
    try:
        node = Recorder()
        deadline = time.monotonic() + args.duration
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not (args.stop_file and args.stop_file.exists())
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException as error:
        primary_error = error
        raise
    finally:

        def shutdown_ros():
            if rclpy.ok():
                rclpy.shutdown()

        actions = []
        if node is not None:
            actions.extend(
                [
                    ("finish evidence", node.finish),
                    ("destroy node", node.destroy_node),
                ]
            )
        actions.append(("shutdown ROS", shutdown_ros))
        cleanup(actions, primary_error)


if __name__ == "__main__":
    main()
