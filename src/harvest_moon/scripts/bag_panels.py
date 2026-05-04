#!/usr/bin/env python3
"""
bag_panels.py — Generate side-by-side image panels from a recorded harvest_moon
rosbag, for visual inspection of strobe illumination per ZED frame.

For each Basler-1 frame in the bag, finds the temporally closest Basler-2 and
ZED-left frames (within a slop window) and saves a composite JPEG showing all
three side by side, with a label banner that includes mean ZED brightness —
the easiest at-a-glance signal for whether a ZED frame caught the strobe.

Pairing uses raw stamps (no -40 ms Basler offset compensation) — the goal is
to surface the wrapper's actual stamp behavior, not hide it.

Run from inside `moveit_pro shell` or `moveit_pro dev` (needs ROS 2 sourced).

Examples:
    python3 bag_panels.py --bag /mnt/ssd/datasets/foo_2026.../bag
    python3 bag_panels.py --bag <path> --every 5 --max 50
    python3 bag_panels.py --bag <path> --outdir /tmp/quick_look --height 400
"""

import argparse
import bisect
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from cv_bridge import CvBridge


CAM1_TOPIC = '/basler_cam_1/pylon_ros2_camera_node/image_raw'
CAM2_TOPIC = '/basler_cam_2/pylon_ros2_camera_node/image_raw'
ZED_TOPIC  = '/zed_x/zed_node/left/image_rect_color'

DEFAULT_HEIGHT = 600
BANNER_H = 50
# Pass-1 (offset detection) wide window — wide enough to catch the matching
# strobed ZED even with substantial stamp lag, but narrow enough that adjacent
# strobe cycles (200 ms apart at 5 Hz Basler) don't dominate.
DETECT_WINDOW_NS = int(0.200 * 1e9)
# Pass-2 (panel emission) tight window — applied around the auto-detected
# offset. Tight enough to reject adjacent cycles, with slop for stamp jitter.
TIGHT_WINDOW_NS = int(0.050 * 1e9)
# Pass-1 (offset detection) wide window for Cam2 — wide enough to catch
# the matching Cam2 even with substantial wrapper-side stamp lag drift,
# but narrow enough not to grab adjacent Basler triggers (133 ms at 7.5 Hz).
DETECT_CAM2_WINDOW_NS = int(0.100 * 1e9)
# Pass-2 (panel emission) tight window for Cam2 — applied around the
# auto-detected Cam1-Cam2 offset. Wide enough to absorb realistic
# wrapper-stamp jitter (~50 ms observed in this rig), tight enough that
# adjacent Basler triggers (133 ms away at 7.5 Hz) never sneak in.
TIGHT_CAM2_WINDOW_NS = int(0.050 * 1e9)
# Brightness top-percentile used to estimate "these are the strobed frames"
# during offset detection. Top 25% by ZED brightness are assumed strobed.
DETECT_TOP_FRACTION = 0.25


def open_bag_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def collect_messages(reader, topics_of_interest):
    type_strs = {t.name: t.type for t in reader.get_all_topics_and_types()}
    available = [t for t in topics_of_interest if t in type_strs]
    type_classes = {t: get_message(type_strs[t]) for t in available}

    storage_filter = rosbag2_py.StorageFilter(topics=available)
    reader.set_filter(storage_filter)

    out = {t: [] for t in topics_of_interest}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in type_classes:
            continue
        msg = deserialize_message(data, type_classes[topic])
        out[topic].append(msg)
    return out


def build_stamp_index(msgs):
    """Return (sorted_stamps_list, stamp_aligned_msgs_list) for bisect lookup."""
    pairs = sorted(((stamp_ns(m), m) for m in msgs), key=lambda p: p[0])
    return [p[0] for p in pairs], [p[1] for p in pairs]


def find_nearest(target_ns, stamps, msgs, max_delta_ns):
    """Closest-in-time message within max_delta_ns. Used for tight Cam2 pairing."""
    if not stamps:
        return None, None
    idx = bisect.bisect_left(stamps, target_ns)
    candidates = []
    if idx < len(stamps):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    best, best_dt = None, None
    for c in candidates:
        dt = abs(stamps[c] - target_ns)
        if dt > max_delta_ns:
            continue
        if best_dt is None or dt < best_dt:
            best, best_dt = msgs[c], dt
    return best, best_dt


def find_brightest_in_window(target_ns, stamps, msgs, bridge, window_ns):
    """Among messages within [target-window, target+window], pick the brightest
    ZED frame. Returns (msg, brightness_value) or (None, None) if no candidates.

    This is the strobe-oracle pairing: timestamps drift, but a strobed ZED
    frame is unambiguously much brighter than ambient ones. Picking the
    brightest in a wide enough window finds the matching strobed frame
    regardless of stamp-pairing instability.
    """
    if not stamps:
        return None, None
    lo = bisect.bisect_left(stamps, target_ns - window_ns)
    hi = bisect.bisect_right(stamps, target_ns + window_ns)
    if lo == hi:
        return None, None
    best, best_b = None, -1.0
    for i in range(lo, hi):
        try:
            arr = img_to_rgb(msgs[i], bridge)
        except Exception:
            continue
        b = float(arr.mean())
        if b > best_b:
            best, best_b = msgs[i], b
    return best, best_b


def img_to_rgb(msg, bridge):
    """sensor_msgs/Image (any encoding cv_bridge knows) → HxWx3 uint8 RGB."""
    return bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')


def resize_height(arr, h):
    img = Image.fromarray(arr)
    new_w = int(img.width * h / img.height)
    return np.asarray(img.resize((new_w, h), Image.BILINEAR))


def mean_brightness(arr):
    return float(arr.mean())


def build_panel(zed_arr, c1_arr, c2_arr, idx,
                zed_stamp, c1_stamp, c2_stamp, height):
    z = resize_height(zed_arr, height)
    c1 = resize_height(c1_arr, height)
    c2 = resize_height(c2_arr, height)
    z_b = mean_brightness(z)
    c1_b = mean_brightness(c1)
    c2_b = mean_brightness(c2)

    c1_dt_ms = (c1_stamp - zed_stamp) / 1e6
    c2_dt_ms = (c2_stamp - zed_stamp) / 1e6

    w_total = z.shape[1] + c1.shape[1] + c2.shape[1]
    canvas = np.zeros((height + BANNER_H, w_total, 3), dtype=np.uint8)
    canvas[BANNER_H:, :z.shape[1]] = z
    canvas[BANNER_H:, z.shape[1]:z.shape[1] + c1.shape[1]] = c1
    canvas[BANNER_H:, z.shape[1] + c1.shape[1]:] = c2

    img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(img)

    draw.text((10, 5), f"#{idx:04d}", fill=(255, 255, 255))
    draw.text((10, 25), f"ZED   brightness={z_b:6.1f}", fill=(255, 255, 255))

    x1 = z.shape[1] + 10
    draw.text((x1, 5), f"Cam1  brightness={c1_b:6.1f}", fill=(255, 255, 255))
    draw.text((x1, 25), f"Cam1-ZED stamp = {c1_dt_ms:+7.1f} ms", fill=(255, 255, 255))

    x2 = z.shape[1] + c1.shape[1] + 10
    draw.text((x2, 5), f"Cam2  brightness={c2_b:6.1f}", fill=(255, 255, 255))
    draw.text((x2, 25), f"Cam2-ZED stamp = {c2_dt_ms:+7.1f} ms", fill=(255, 255, 255))

    return img


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--bag", required=True, type=Path,
                    help="Path to the rosbag2 directory (e.g. /mnt/ssd/datasets/foo/bag)")
    ap.add_argument("--outdir", type=Path, default=None,
                    help="Output dir for panel JPEGs (default: <bag>/../panels)")
    ap.add_argument("--every", type=int, default=1,
                    help="Emit one panel per N Basler-1 frames (default: 1)")
    ap.add_argument("--max", type=int, default=0,
                    help="Cap total panels written (default: unlimited)")
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT,
                    help=f"Per-subimage height in pixels (default: {DEFAULT_HEIGHT})")
    ap.add_argument("--cam1-topic", default=CAM1_TOPIC)
    ap.add_argument("--cam2-topic", default=CAM2_TOPIC)
    ap.add_argument("--zed-topic", default=ZED_TOPIC)
    ap.add_argument("--zed-offset-ms", type=float, default=None,
                    help="Manually override the Cam1→ZED stamp offset in ms "
                         "(positive = ZED stamp later than Cam1 stamp). "
                         "If omitted, auto-detected from top-25%% brightest "
                         "ZED frames in a ±200 ms wide window.")
    ap.add_argument("--cam2-offset-ms", type=float, default=None,
                    help="Manually override the Cam1→Cam2 stamp offset in ms. "
                         "If omitted, auto-detected as the median delta "
                         "between Cam1 and the temporally closest Cam2 frame.")
    args = ap.parse_args()

    if not args.bag.exists():
        sys.exit(f"error: bag path does not exist: {args.bag}")

    if args.outdir is None:
        args.outdir = args.bag.parent / "panels"
    args.outdir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    bridge = CvBridge()

    print(f"Reading {args.bag} ...")
    reader = open_bag_reader(str(args.bag))
    topics = [args.cam1_topic, args.cam2_topic, args.zed_topic]
    msgs = collect_messages(reader, topics)

    for t in topics:
        print(f"  {t}: {len(msgs[t])} messages")

    if not msgs[args.cam1_topic]:
        sys.exit(f"error: no messages on {args.cam1_topic}")
    if not msgs[args.zed_topic]:
        sys.exit(f"error: no messages on {args.zed_topic}")
    if not msgs[args.cam2_topic]:
        sys.exit(f"error: no messages on {args.cam2_topic}")

    # Build stamp indices for fast bisect lookup
    cam2_stamps, cam2_msgs = build_stamp_index(msgs[args.cam2_topic])
    zed_stamps, zed_msgs = build_stamp_index(msgs[args.zed_topic])

    # Pass 1: detect the actual Cam1→ZED stamp offset for this bag by
    # finding the brightest ZED in a wide window around each Cam1 frame,
    # then taking the median delta of the top-N% brightest hits (those are
    # the strobed frames; their deltas converge on the true offset).
    if args.zed_offset_ms is None:
        print("\nPass 1: detecting Cam1→ZED stamp offset...")
        deltas = []   # list of (brightness, delta_ns)
        for c1 in msgs[args.cam1_topic]:
            target = stamp_ns(c1)
            zed, brightness = find_brightest_in_window(
                target, zed_stamps, zed_msgs, bridge, DETECT_WINDOW_NS
            )
            if zed is None:
                continue
            deltas.append((brightness, stamp_ns(zed) - target))
        if not deltas:
            sys.exit("error: no ZED frames found within ±200 ms of any Cam1 frame")
        deltas.sort(reverse=True)   # brightest first
        top_n = max(1, int(len(deltas) * DETECT_TOP_FRACTION))
        top_deltas_ns = sorted(d for _, d in deltas[:top_n])
        offset_ns = top_deltas_ns[len(top_deltas_ns) // 2]   # median
        print(f"  Median Cam1→ZED delta on top {DETECT_TOP_FRACTION*100:.0f}% brightest: "
              f"{offset_ns / 1e6:+.1f} ms ({len(top_deltas_ns)} samples)")
    else:
        offset_ns = int(args.zed_offset_ms * 1e6)
        print(f"\nUsing user-supplied Cam1→ZED offset: {offset_ns / 1e6:+.1f} ms")

    # Cam1↔Cam2 pairing: STREAM-WALK with stamp-based resync. Both Baslers
    # fire on the same Teensy trigger, but per-camera wrapper jitter and
    # occasional missed triggers mean we can't trust either pure-index or
    # pure-stamp pairing. Strategy:
    #   - Compute baseline Cam1-Cam2 stamp delta from the first few frames
    #   - Walk both arrival-order lists with two pointers
    #   - At each step, compare current Cam1-Cam2 delta to baseline:
    #       * if within ±half-trigger-period → same trigger, pair, advance both
    #       * if Cam1 stamp ahead → Cam2 missed something, advance Cam2 only
    #       * if Cam2 stamp ahead → Cam1 missed something, advance Cam1 only
    # Result: paired lists where Cam1[k] and Cam2[k] are guaranteed same-trigger.
    cam2_arrival = msgs[args.cam2_topic]
    cam1_arrival = msgs[args.cam1_topic]
    cam1_count = len(cam1_arrival)
    cam2_count = len(cam2_arrival)

    # Baseline Cam1-Cam2 delta from the first ~10 in-sync frames.
    baseline_samples = []
    for k in range(min(20, cam1_count, cam2_count)):
        d = stamp_ns(cam1_arrival[k]) - stamp_ns(cam2_arrival[k])
        baseline_samples.append(d)
    baseline_samples.sort()
    cam1_cam2_baseline_ns = baseline_samples[len(baseline_samples) // 2]
    half_period_ns = int(0.067 * 1e9)   # half of 7.5 Hz period

    # Walk both streams.
    paired_cam1, paired_cam2 = [], []
    desync_events = 0
    i, j = 0, 0
    while i < cam1_count and j < cam2_count:
        d = stamp_ns(cam1_arrival[i]) - stamp_ns(cam2_arrival[j])
        if abs(d - cam1_cam2_baseline_ns) < half_period_ns:
            paired_cam1.append(cam1_arrival[i])
            paired_cam2.append(cam2_arrival[j])
            i += 1
            j += 1
        elif d > cam1_cam2_baseline_ns:
            # Cam1 stamp ahead of where it should be → Cam2 has an extra
            # frame OR Cam1 missed a trigger. Skip Cam2.
            j += 1
            desync_events += 1
        else:
            # Cam2 stamp ahead → Cam1 has an extra frame or Cam2 missed.
            i += 1
            desync_events += 1

    print(f"\nCam1↔Cam2 pairing: stream-walk with baseline delta "
          f"{cam1_cam2_baseline_ns / 1e6:+.1f} ms.")
    print(f"  Cam1 count={cam1_count}, Cam2 count={cam2_count}, "
          f"paired={len(paired_cam1)}, resync events={desync_events}.")

    # Pass 2: emit panels using a tight window centered on the offset.
    print(f"\nPass 2: generating panels into {args.outdir}/")
    saved = 0
    skipped_no_pair = 0
    for i, (c1, c2) in enumerate(zip(paired_cam1, paired_cam2)):
        if i % args.every != 0:
            continue
        if args.max and saved >= args.max:
            break
        target = stamp_ns(c1)
        zed_target = target + offset_ns
        zed, _ = find_brightest_in_window(zed_target, zed_stamps, zed_msgs, bridge, TIGHT_WINDOW_NS)
        if zed is None:
            skipped_no_pair += 1
            continue
        try:
            zed_rgb = img_to_rgb(zed, bridge)
            c1_rgb = img_to_rgb(c1, bridge)
            c2_rgb = img_to_rgb(c2, bridge)
        except Exception as e:
            print(f"  [{i:04d}] image conversion failed: {e}")
            continue
        panel = build_panel(zed_rgb, c1_rgb, c2_rgb, i,
                            stamp_ns(zed), target, stamp_ns(c2),
                            args.height)
        out_path = args.outdir / f"panel_{saved:04d}.jpg"
        panel.save(out_path, quality=90)
        saved += 1
        if saved % 25 == 0:
            print(f"  ... saved {saved}")

    print(f"\nDone. {saved} panels in {args.outdir}/")
    print(f"  Cam2 pair: stream-walk (paired={len(paired_cam1)} of "
          f"{cam1_count} cam1, {desync_events} resync events).")
    print(f"  ZED pair:  brightest within ±{TIGHT_WINDOW_NS / 1e6:.0f} ms of "
          f"(cam1_stamp {offset_ns / 1e6:+.1f} ms).")
    if skipped_no_pair:
        print(f"  ({skipped_no_pair} cam1 frames had no matching strobed ZED — "
              "wrapper dropped the matching ZED frame at NATIVE/ULTRA)")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
