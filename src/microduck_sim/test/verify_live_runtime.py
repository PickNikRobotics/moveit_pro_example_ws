"""Opt-in motion check: run manually inside the isolated microduck_sim Runtime."""

import json
from functools import partial
import os
import time

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    if os.environ.get("MOVEIT_CONFIG_PACKAGE") != "microduck_sim":
        raise RuntimeError(
            "This motion check is only for the microduck_sim configuration"
        )
    rclpy.init()
    node = rclpy.create_node("verify_microduck_runtime")
    samples = []
    frames = {"overview": [], "head": []}
    calibration = {}

    def state(message):
        values = dict(zip(message.name, message.position, strict=True))
        assert np.isfinite(message.position).all()
        samples.append(
            (time.monotonic(), np.array([values["base_x"], values["base_y"]]))
        )

    def camera(stream, message):
        assert (message.width, message.height, message.encoding) == (640, 480, "rgb8")
        assert len(message.data) == message.step * message.height
        frames[stream].append((message.header, time.monotonic()))

    def camera_info(stream, message):
        assert (message.width, message.height) == (640, 480)
        assert message.k[0] > 0 and message.k[4] > 0
        assert message.k[2] == 320 and message.k[5] == 240
        calibration[stream] = message

    node.create_subscription(JointState, "/joint_states", state, 10)
    for stream in frames:
        node.create_subscription(
            Image, f"/microduck/{stream}/image_raw", partial(camera, stream), 2
        )
        node.create_subscription(
            CameraInfo,
            f"/microduck/{stream}/camera_info",
            partial(camera_info, stream),
            2,
        )
    command = node.create_publisher(String, "/microduck/command", 1)
    reset = node.create_client(Trigger, "/microduck/reset")
    ready = node.create_client(Trigger, "/microduck/ready")

    def call(client):
        assert client.wait_for_service(timeout_sec=8)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=8)
        assert future.done() and future.result().success

    def spin(seconds, motion=None):
        end = time.monotonic() + seconds
        last_publish = 0
        while time.monotonic() < end:
            if motion and time.monotonic() - last_publish > 0.05:
                command.publish(String(data=motion))
                last_publish = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        call(reset)
        spin(2)
        call(ready)
        assert len(samples) > 10
        assert set(calibration) == set(frames)
        for stream, images in frames.items():
            assert len(images) > 3
            assert images[-1][0].frame_id == calibration[stream].header.frame_id
            assert images[0][0].stamp != images[-1][0].stamp
        assert (
            calibration["head"].header.frame_id
            != calibration["overview"].header.frame_id
        )
        start = samples[-1][1]
        spin(3, "forward")
        distance = float(np.linalg.norm(samples[-1][1] - start))
        assert distance > 0.025, distance
        # No explicit stop is published: this exercises loss of command refresh.
        spin(1)
        stopped = samples[-1][1]
        spin(2)
        drift = float(np.linalg.norm(samples[-1][1] - stopped))
        assert drift < 0.015, drift
        call(ready)
        call(reset)
        spin(1)
        assert np.linalg.norm(samples[-1][1]) < 0.01
        print(
            json.dumps(
                {
                    "forward_metres": distance,
                    "drift_after_watchdog_metres": drift,
                    "joint_samples": len(samples),
                    "camera_frames": {
                        stream: len(images) for stream, images in frames.items()
                    },
                    "reset": "passed",
                    "ready": "passed",
                },
                indent=2,
            )
        )
    finally:
        command.publish(String(data="stand"))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
