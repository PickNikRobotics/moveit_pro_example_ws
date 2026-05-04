#!/usr/bin/env python3
"""
pylon_pub_test.py — Diagnostic: open both Basler cameras in ONE process
via pypylon and publish to ROS topics, to test whether single-process
pypylon + ROS publish can sustain 15 Hz dual-cam in the moveit_pro
container (the existing pylon_ros2_camera_wrapper uses TWO processes
and can't sustain 15 Hz).

Both cameras are externally-triggered via the Teensy. This script:
- Opens both cameras with the production UserSet1 config
- Starts grabbing
- For each frame, publishes sensor_msgs/Image on diagnostic topics
  (/test/cam1/image_raw, /test/cam2/image_raw)

Run from inside moveit_pro shell. Stop existing camera nodes first
(both pylon_ros2_camera_node instances must NOT be running, otherwise
they'll hold the cameras exclusive).

To verify rate, in another moveit_pro shell:
    ros2 topic hz /test/cam1/image_raw
    ros2 topic hz /test/cam2/image_raw

Stop the script with Ctrl+C.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from pypylon import pylon


CAM1_SERIAL = '25233972'
CAM2_SERIAL = '25435376'

GRAB_TIMEOUT_MS = 2000


def open_basler_by_serial(serial):
    tl = pylon.TlFactory.GetInstance()
    for info in tl.EnumerateDevices():
        if info.GetSerialNumber() == serial:
            return pylon.InstantCamera(tl.CreateDevice(info))
    return None


def setup_basler(cam):
    cam.Open()
    # Cameras boot with UserSet1 as the default (set by provision_basler.py),
    # which contains the production trigger config. No explicit override needed.


def basler_to_image_msg(arr, frame_id, stamp):
    """Wrap a HxW bayer_rggb8 numpy array into a sensor_msgs/Image."""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = 'bayer_rggb8'
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1])
    msg.data = arr.tobytes()
    return msg


class PylonPubNode(Node):
    def __init__(self):
        super().__init__('pylon_pub_test')

        # Use BEST_EFFORT to match typical camera publishers and avoid backpressure
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub1 = self.create_publisher(Image, '/test/cam1/image_raw', qos)
        self.pub2 = self.create_publisher(Image, '/test/cam2/image_raw', qos)

        self.get_logger().info(f"Opening Basler {CAM1_SERIAL}...")
        self.bc1 = open_basler_by_serial(CAM1_SERIAL)
        self.get_logger().info(f"Opening Basler {CAM2_SERIAL}...")
        self.bc2 = open_basler_by_serial(CAM2_SERIAL)
        if not self.bc1 or not self.bc2:
            self.get_logger().error("Could not enumerate both Baslers — "
                                    "make sure pylon_ros2_camera_node "
                                    "instances are NOT running.")
            sys.exit(1)
        setup_basler(self.bc1)
        setup_basler(self.bc2)

        self.bc1.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.bc2.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.get_logger().info("Both cameras grabbing.")

        # Stats
        self.t_start = time.perf_counter()
        self.n1 = 0
        self.n2 = 0
        self.n1_drop = 0
        self.n2_drop = 0
        self.last_log = self.t_start

        # Use a high-frequency timer to poll for new frames.
        # 200 Hz polling >> 15 Hz capture rate, so we never miss a frame.
        self.timer = self.create_timer(1.0 / 200.0, self.poll)

    def poll(self):
        now = self.get_clock().now().to_msg()

        try:
            r1 = self.bc1.RetrieveResult(0, pylon.TimeoutHandling_Return)
        except pylon.TimeoutException:
            r1 = None
        if r1 and r1.GrabSucceeded():
            arr = r1.GetArray()
            self.pub1.publish(basler_to_image_msg(arr, 'basler_cam_1', now))
            self.n1 += 1
        if r1:
            r1.Release()

        try:
            r2 = self.bc2.RetrieveResult(0, pylon.TimeoutHandling_Return)
        except pylon.TimeoutException:
            r2 = None
        if r2 and r2.GrabSucceeded():
            arr = r2.GetArray()
            self.pub2.publish(basler_to_image_msg(arr, 'basler_cam_2', now))
            self.n2 += 1
        if r2:
            r2.Release()

        # Log every 5 seconds
        t = time.perf_counter()
        if t - self.last_log >= 5.0:
            elapsed = t - self.t_start
            r1_hz = self.n1 / elapsed
            r2_hz = self.n2 / elapsed
            self.get_logger().info(
                f"[{elapsed:5.1f}s]  cam1={self.n1} ({r1_hz:5.2f} Hz)  "
                f"cam2={self.n2} ({r2_hz:5.2f} Hz)"
            )
            self.last_log = t

    def shutdown(self):
        try:
            self.bc1.StopGrabbing()
            self.bc1.Close()
        except Exception:
            pass
        try:
            self.bc2.StopGrabbing()
            self.bc2.Close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = PylonPubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
