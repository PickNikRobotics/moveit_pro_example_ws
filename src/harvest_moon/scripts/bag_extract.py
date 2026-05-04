#!/usr/bin/env python3
"""
bag_extract.py — Dump image frames from a recorded harvest_moon rosbag,
one folder per topic, with brightness in the filename for at-a-glance
identification of strobe-illuminated frames.

This is the simple sibling of bag_panels.py: no pairing, no syncing,
no time-domain anything. For each image-publishing topic, walk the bag,
compute mean brightness of each frame, save as JPEG.

Filename format: `frame_NNNN_b<brightness>.jpg`, where brightness is the
mean pixel value (0-255). Strobed frames stand out by brightness; sort
the directory by name and you can flip through to spot bright ones.

Run from inside `moveit_pro shell` or `moveit_pro dev` (needs ROS 2 sourced).

Examples:
    python3 bag_extract.py --bag /mnt/ssd/datasets/foo_2026.../bag
    python3 bag_extract.py --bag <path> --threshold 80
    python3 bag_extract.py --bag <path> --every 5 --max 100
    python3 bag_extract.py --bag <path> --topics /basler_cam_1/pylon_ros2_camera_node/image_raw
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import matplotlib
matplotlib.use('Agg')   # non-interactive backend for headless container
import matplotlib.pyplot as plt

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from cv_bridge import CvBridge


DEFAULT_TOPICS = [
    "/basler_cam_1/pylon_ros2_camera_node/image_raw",
    "/basler_cam_2/pylon_ros2_camera_node/image_raw",
    "/zed_x/zed_node/left/image_rect_color",
    "/zed_x/zed_node/right/image_rect_color",
]

DEFAULT_HEIGHT = 600    # px; 0 to keep native resolution


def open_bag_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def topic_to_dirname(topic):
    """Convert a ROS topic path like /a/b/c into a flat dirname a_b_c."""
    return topic.strip('/').replace('/', '_')


def otsu_threshold(values, n_bins=256):
    """Otsu's method for bimodal threshold selection.

    Given an array of values, return the threshold that maximizes
    between-class variance — the "natural" split between two clusters
    in a bimodal distribution.
    """
    arr = np.asarray(values, dtype=float)
    if arr.size < 2:
        return float(arr.mean()) if arr.size else 0.0
    hist, edges = np.histogram(arr, bins=n_bins)
    p = hist / hist.sum()
    centers = 0.5 * (edges[:-1] + edges[1:])
    cum_p = np.cumsum(p)
    cum_mean = np.cumsum(p * centers)
    total_mean = cum_mean[-1]
    # Between-class variance σ²_b = ω0(t) * ω1(t) * (μ0 - μ1)²
    # ω0 = cum_p, ω1 = 1 - cum_p
    # μ0 = cum_mean / cum_p, μ1 = (total_mean - cum_mean) / (1 - cum_p)
    with np.errstate(divide='ignore', invalid='ignore'):
        sigma_b2 = (cum_p * (1 - cum_p) *
                    ((cum_mean / np.where(cum_p == 0, 1, cum_p))
                     - ((total_mean - cum_mean) / np.where(cum_p == 1, 1, 1 - cum_p))) ** 2)
    sigma_b2 = np.nan_to_num(sigma_b2, nan=0.0, posinf=0.0, neginf=0.0)
    best_idx = int(np.argmax(sigma_b2))
    return float(centers[best_idx])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--bag", required=True, type=Path,
                    help="Path to the rosbag2 directory (e.g. /mnt/ssd/datasets/foo/bag)")
    ap.add_argument("--outdir", type=Path, default=None,
                    help="Output dir (default: <bag>/../extract)")
    ap.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS,
                    help="Image topics to extract (default: 2x Basler raw + ZED L/R rect_color)")
    ap.add_argument("--threshold", type=float, default=0.0,
                    help="Skip frames with mean brightness below this (0-255). "
                         "Default 0 = save all. Try ~80 to filter out ambient.")
    ap.add_argument("--auto-sort", action=argparse.BooleanOptionalAction, default=True,
                    help="After extraction, compute per-topic Otsu threshold and "
                         "move below-threshold frames into a low/ subfolder "
                         "(above-threshold stay at the top of the topic dir). "
                         "Skipped for Basler topics (their distribution is unimodal "
                         "since every frame is strobed). Use --no-auto-sort to disable.")
    ap.add_argument("--every", type=int, default=1,
                    help="Save one frame per N (default: 1 = save all).")
    ap.add_argument("--max", type=int, default=0,
                    help="Cap total saved frames per topic (default: unlimited).")
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT,
                    help=f"Resize saved frames to this height in px (default: {DEFAULT_HEIGHT}; "
                         "0 = native resolution).")
    args = ap.parse_args()

    if not args.bag.exists():
        sys.exit(f"error: bag path does not exist: {args.bag}")

    if args.outdir is None:
        args.outdir = args.bag.parent / "extract"
    args.outdir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    bridge = CvBridge()

    # Open bag and get topic types
    print(f"Reading {args.bag} ...")
    reader = open_bag_reader(str(args.bag))
    type_strs = {t.name: t.type for t in reader.get_all_topics_and_types()}
    available_topics = [t for t in args.topics if t in type_strs]
    missing = [t for t in args.topics if t not in type_strs]
    if missing:
        print(f"WARNING: {len(missing)} requested topic(s) not in bag:")
        for t in missing:
            print(f"  - {t}")
    if not available_topics:
        sys.exit("error: no requested topics found in bag")

    type_classes = {t: get_message(type_strs[t]) for t in available_topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=available_topics))

    # Per-topic counters (in arrival order)
    topic_seen = {t: 0 for t in available_topics}
    topic_saved = {t: 0 for t in available_topics}
    topic_brightnesses = {t: [] for t in available_topics}   # ALL frames, even ones we skip
    topic_outdirs = {}
    for t in available_topics:
        d = args.outdir / topic_to_dirname(t)
        d.mkdir(parents=True, exist_ok=True)
        topic_outdirs[t] = d

    print(f"\nExtracting to {args.outdir}/")
    print(f"  Threshold: brightness >= {args.threshold:.0f}/255 "
          f"({'all frames' if args.threshold == 0 else 'filtering'})")
    print(f"  Every {args.every}, max {args.max if args.max else 'unlimited'} per topic, "
          f"height {args.height if args.height else 'native'} px")
    print()

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in type_classes:
            continue
        idx = topic_seen[topic]
        topic_seen[topic] += 1
        if args.max and topic_saved[topic] >= args.max:
            continue
        if idx % args.every != 0:
            continue
        try:
            msg = deserialize_message(data, type_classes[topic])
            arr = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            print(f"  [{topic}] frame {idx}: convert failed ({e})")
            continue
        brightness = float(arr.mean())
        topic_brightnesses[topic].append(brightness)   # log every frame, even ones we skip
        if brightness < args.threshold:
            continue
        if args.height and args.height != arr.shape[0]:
            new_w = int(arr.shape[1] * args.height / arr.shape[0])
            arr = np.asarray(
                Image.fromarray(arr).resize((new_w, args.height), Image.BILINEAR)
            )
        out_path = topic_outdirs[topic] / f"frame_{idx:04d}_b{brightness:03.0f}.jpg"
        Image.fromarray(arr).save(out_path, quality=90)
        topic_saved[topic] += 1
        if topic_saved[topic] % 25 == 0:
            print(f"  [{topic_to_dirname(topic)}] saved {topic_saved[topic]}")

    print()
    print("Per-topic summary:")
    topic_otsu = {}
    for t in available_topics:
        b = topic_brightnesses[t]
        if b:
            arr = np.asarray(b)
            otsu = otsu_threshold(arr)
            topic_otsu[t] = otsu
            stats = (f"  brightness: min={arr.min():.1f}, max={arr.max():.1f}, "
                     f"mean={arr.mean():.1f}, p25={np.percentile(arr, 25):.1f}, "
                     f"p50={np.percentile(arr, 50):.1f}, p75={np.percentile(arr, 75):.1f}, "
                     f"otsu_threshold={otsu:.1f}")
        else:
            stats = "  brightness: (no frames)"
        print(f"  {topic_to_dirname(t)}: seen={topic_seen[t]}, saved={topic_saved[t]} "
              f"in {topic_outdirs[t]}/")
        print(stats)

    # Auto-sort: move below-Otsu-threshold frames into a low/ subfolder per topic.
    # Skipped for Basler topics — their brightness distribution is unimodal
    # (every Basler frame is strobed) so Otsu would split it meaninglessly.
    if args.auto_sort:
        print("\nAuto-sorting by Otsu threshold (low/ subfolder = below threshold):")
        for t in available_topics:
            otsu = topic_otsu.get(t)
            if otsu is None:
                continue
            if "basler" in t.lower():
                print(f"  {topic_to_dirname(t)}: skipped (Basler topic — "
                      f"all frames strobed, no meaningful split).")
                continue
            topic_dir = topic_outdirs[t]
            low_dir = topic_dir / "low"
            low_dir.mkdir(exist_ok=True)
            moved = 0
            kept = 0
            for f in topic_dir.glob("frame_*.jpg"):
                # Filename format: frame_NNNN_bBBB.jpg — extract brightness from filename
                try:
                    b = float(f.stem.rsplit("_b", 1)[1])
                except (IndexError, ValueError):
                    continue
                if b < otsu:
                    f.rename(low_dir / f.name)
                    moved += 1
                else:
                    kept += 1
            print(f"  {topic_to_dirname(t)}: kept {kept} (>= {otsu:.1f}), "
                  f"moved {moved} to low/")

    # Generate brightness histograms — one per topic, plus a combined plot
    print("\nGenerating brightness histograms...")
    for t in available_topics:
        b = topic_brightnesses[t]
        if not b:
            continue
        arr = np.asarray(b)
        b_min = max(0, arr.min() - 5)
        b_max = min(255, arr.max() + 5)
        # Use 1.0-unit bins (or finer if range is small) so bimodal modes show
        n_bins = max(20, int(b_max - b_min))
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.hist(arr, bins=n_bins, range=(b_min, b_max),
                color='steelblue', edgecolor='black', alpha=0.85)
        ax.set_xlabel('Mean brightness (0-255)')
        ax.set_ylabel('Frame count')
        ax.set_title(f'{topic_to_dirname(t)}  (n={len(arr)}, '
                     f'mean={arr.mean():.1f}, std={arr.std():.2f})')
        ax.axvline(arr.mean(), color='red', linestyle='--', linewidth=1, label='mean')
        otsu = topic_otsu.get(t)
        if otsu is not None:
            ax.axvline(otsu, color='purple', linestyle='-', linewidth=1.5,
                       label=f'otsu={otsu:.1f}')
        if args.threshold > 0:
            ax.axvline(args.threshold, color='green', linestyle=':',
                       linewidth=1, label=f'threshold={args.threshold:.0f}')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        out_path = args.outdir / f"{topic_to_dirname(t)}_histogram.png"
        fig.savefig(out_path, dpi=80)
        plt.close(fig)
        print(f"  {out_path.name}")

    # Combined 2x2 grid of histograms (if there are multiple topics)
    if len(available_topics) > 1:
        n_topics = len(available_topics)
        cols = 2
        rows = (n_topics + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(14, 4 * rows))
        axes = np.atleast_2d(axes).flatten()
        for ax, t in zip(axes, available_topics):
            b = topic_brightnesses[t]
            if not b:
                ax.set_visible(False)
                continue
            arr = np.asarray(b)
            b_min = max(0, arr.min() - 5)
            b_max = min(255, arr.max() + 5)
            n_bins = max(20, int(b_max - b_min))
            ax.hist(arr, bins=n_bins, range=(b_min, b_max),
                    color='steelblue', edgecolor='black', alpha=0.85)
            ax.axvline(arr.mean(), color='red', linestyle='--', linewidth=1)
            otsu_t = topic_otsu.get(t)
            if otsu_t is not None:
                ax.axvline(otsu_t, color='purple', linestyle='-', linewidth=1.5)
            if args.threshold > 0:
                ax.axvline(args.threshold, color='green', linestyle=':', linewidth=1)
            ax.set_xlabel('Mean brightness')
            ax.set_ylabel('Count')
            ax.set_title(f'{topic_to_dirname(t)}\n(n={len(arr)}, '
                         f'µ={arr.mean():.1f}, σ={arr.std():.2f})', fontsize=9)
            ax.grid(True, alpha=0.3)
        for ax in axes[len(available_topics):]:
            ax.set_visible(False)
        fig.tight_layout()
        out_path = args.outdir / "histograms_all.png"
        fig.savefig(out_path, dpi=80)
        plt.close(fig)
        print(f"  {out_path.name}  (combined grid)")

    print(f"\nDone. Output root: {args.outdir}/")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
