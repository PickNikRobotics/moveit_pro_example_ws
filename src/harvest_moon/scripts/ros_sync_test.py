"""
ros_sync_test.py

Tests sync at the ROS topic level. Subscribes to ZED + both Basler image topics,
uses message_filters.ApproximateTimeSynchronizer to pair frames by hardware
timestamps, and saves N synced triplets as side-by-side composites.

Run from INSIDE the moveit_pro shell (where ROS topics are reachable).

Usage:
  python3 ~/user_ws/vision_debug/ros_sync_test.py --count 40

Each saved composite is labeled with all three frames' header timestamps and
the max pairwise offset, so you can verify ROS-level sync quantitatively.
"""

import argparse
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from PIL import Image as PILImage, ImageDraw

CAM1_TOPIC = '/basler_cam_1/pylon_ros2_camera_node/image_raw'
CAM2_TOPIC = '/basler_cam_2/pylon_ros2_camera_node/image_raw'
ZED_TOPIC  = '/zed_x/zed_node/stereo_raw/image_raw_color'

COMPOSITE_HEIGHT = 540
BANNER_H = 30
SLOP_S = 0.10   # 10 ms — comfortably wider than expected sync error


def bayer_rg8_to_rgb(bayer):
    r = bayer[0::2, 0::2]
    g1 = bayer[0::2, 1::2].astype(np.uint16)
    g2 = bayer[1::2, 0::2].astype(np.uint16)
    b = bayer[1::2, 1::2]
    g = ((g1 + g2) // 2).astype(np.uint8)
    return np.stack([r, g, b], axis=2)


def msg_to_pil(msg):
    h, w, enc = msg.height, msg.width, msg.encoding
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == 'bayer_rggb8' or enc == 'mono8':
        arr = arr.reshape((h, w))
        if enc == 'bayer_rggb8':
            arr = bayer_rg8_to_rgb(arr)
            return PILImage.fromarray(arr)
        return PILImage.fromarray(arr).convert('RGB')
    if enc == 'rgb8':
        return PILImage.fromarray(arr.reshape((h, w, 3)))
    if enc == 'bgr8':
        return PILImage.fromarray(arr.reshape((h, w, 3))[..., ::-1])
    if enc == 'rgba8':
        return PILImage.fromarray(arr.reshape((h, w, 4))[..., :3])
    if enc == 'bgra8':
        return PILImage.fromarray(arr.reshape((h, w, 4))[..., [2, 1, 0]])
    raise ValueError(f"Unhandled image encoding: {enc}")


def resize_h(img, h):
    w = int(img.width * h / img.height)
    return img.resize((w, h), PILImage.BILINEAR)


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SyncNode(Node):
    def __init__(self, args):
        super().__init__('ros_sync_test')
        self.target = args.count
        self.received = 0
        self.outdir = args.outdir
        os.makedirs(self.outdir, exist_ok=True)
        self.offsets_ms = []
        self.done = False

        zed_sub = message_filters.Subscriber(self, Image, args.zed_topic)

        # Basler stamps lag ZED stamps by ~40 ms (wrapper receive-time stamping
        # vs ZED's hardware-aligned stamp). To get ApproximateTimeSynchronizer
        # to pair the SAME-tick ZED with each Basler, we subtract that offset
        # from Basler stamps before they reach the synchronizer. Without this,
        # ApproxTimeSync prefers the next-tick ZED (smaller stamp diff), which
        # for DIV=3 means a non-strobe-firing tick.
        c1_filter = message_filters.SimpleFilter()
        c2_filter = message_filters.SimpleFilter()

        def shift_stamp(msg, dt_s):
            # Negative dt_s = move stamp earlier in time
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            adj_ns = stamp_ns + int(dt_s * 1e9)
            msg.header.stamp.sec = adj_ns // 1_000_000_000
            msg.header.stamp.nanosec = adj_ns % 1_000_000_000
            return msg

        def make_relay(out_filter, dt_s):
            def cb(msg):
                out_filter.signalMessage(shift_stamp(msg, dt_s))
            return cb

        self.create_subscription(Image, args.cam1_topic,
                                 make_relay(c1_filter, -args.cam1_offset), 30)
        self.create_subscription(Image, args.cam2_topic,
                                 make_relay(c2_filter, -args.cam2_offset), 30)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [zed_sub, c1_filter, c2_filter],
            queue_size=30,
            slop=args.slop,
        )
        self.sync.registerCallback(self.callback)
        self.get_logger().info(
            f"Subscribed.  Saving {self.target} triplets to {self.outdir}, slop={args.slop*1000:.0f} ms"
        )

    def callback(self, zed_msg, c1_msg, c2_msg):
        if self.received >= self.target:
            return
        i = self.received
        self.received += 1

        z_img = msg_to_pil(zed_msg)
        c1_img = msg_to_pil(c1_msg).rotate(-90, expand=True)
        c2_img = msg_to_pil(c2_msg).rotate(-90, expand=True)

        z = resize_h(z_img, COMPOSITE_HEIGHT)
        c1 = resize_h(c1_img, COMPOSITE_HEIGHT)
        c2 = resize_h(c2_img, COMPOSITE_HEIGHT)

        comp = PILImage.new("RGB", (z.width + c1.width + c2.width,
                                    COMPOSITE_HEIGHT + BANNER_H), (20, 20, 20))
        comp.paste(z, (0, BANNER_H))
        comp.paste(c1, (z.width, BANNER_H))
        comp.paste(c2, (z.width + c1.width, BANNER_H))

        zt = stamp_to_seconds(zed_msg.header.stamp)
        c1t = stamp_to_seconds(c1_msg.header.stamp)
        c2t = stamp_to_seconds(c2_msg.header.stamp)
        offsets = [abs(zt - c1t), abs(zt - c2t), abs(c1t - c2t)]
        max_off_ms = max(offsets) * 1000.0
        self.offsets_ms.append(max_off_ms)

        d = ImageDraw.Draw(comp)
        d.text((10, 8), f"ZED  t={zt:.4f}", fill=(255, 255, 255))
        d.text((z.width + 10, 8), f"Cam1  t={c1t:.4f}", fill=(255, 255, 255))
        d.text((z.width + c1.width + 10, 8),
               f"Cam2  dt_max={max_off_ms:.2f} ms", fill=(255, 255, 255))

        comp.save(f"{self.outdir}/triplet_{i:03d}.jpg", quality=95)
        # Signed offsets: positive = Basler stamp is LATER than ZED (Basler "behind")
        cam1_minus_zed_ms = (c1t - zt) * 1000.0
        cam2_minus_zed_ms = (c2t - zt) * 1000.0
        # Lag relative to real time: how old is each frame's stamp at the moment we got it?
        now_s = self.get_clock().now().nanoseconds / 1e9
        zed_lag_ms  = (now_s - zt)  * 1000.0
        cam1_lag_ms = (now_s - c1t) * 1000.0
        cam2_lag_ms = (now_s - c2t) * 1000.0
        print(f"[{i:03d}/{self.target}]  cam1-zed={cam1_minus_zed_ms:+7.2f} ms  "
              f"cam2-zed={cam2_minus_zed_ms:+7.2f} ms  Δt_max={max_off_ms:6.2f} ms")
        print(f"           lag_vs_now: zed={zed_lag_ms:7.1f}  cam1={cam1_lag_ms:7.1f}  cam2={cam2_lag_ms:7.1f} ms")

        if self.received >= self.target:
            print("\n=== Summary ===")
            arr = np.array(self.offsets_ms)
            print(f"  triplets:          {len(arr)}")
            print(f"  Δt_max:  min={arr.min():.2f} ms  "
                  f"median={np.median(arr):.2f} ms  max={arr.max():.2f} ms")
            print(f"  Saved to {self.outdir}")
            # Translate container path to host path for eog hint.
            # In moveit_pro: ~/user_ws (container) == ~/moveit_pro/moveit_pro_example_ws (host)
            host_path = self.outdir.replace(
                '/home/picknik/user_ws',
                '/home/picknik/moveit_pro/moveit_pro_example_ws')
            print("\nView composites (run from a HOST terminal — eog isn't in the container):")
            print(f"  eog {host_path}/triplet_000.jpg")
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--outdir",
                    default=os.path.expanduser(
                        f"~/user_ws/vision_debug/ros_sync_{datetime.now():%Y%m%d_%H%M%S}"))
    ap.add_argument("--slop", type=float, default=SLOP_S,
                    help="message_filters slop in seconds (default: 0.010)")
    ap.add_argument("--cam1-topic", default=CAM1_TOPIC)
    ap.add_argument("--cam2-topic", default=CAM2_TOPIC)
    ap.add_argument("--zed-topic", default=ZED_TOPIC)
    ap.add_argument("--cam1-offset", type=float, default=0.040,
                    help="Seconds to subtract from Cam 1 stamps before sync (default: 0.040)")
    ap.add_argument("--cam2-offset", type=float, default=0.035,
                    help="Seconds to subtract from Cam 2 stamps before sync (default: 0.035)")
    args = ap.parse_args()

    rclpy.init()
    node = SyncNode(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
