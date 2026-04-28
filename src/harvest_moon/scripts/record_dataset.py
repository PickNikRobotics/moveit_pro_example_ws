#!/usr/bin/env python3
"""
record_dataset.py — Field data recorder for the harvest_moon vision rig.

Wraps `ros2 bag record` with sensible defaults for the harvest_moon camera
topics. Each invocation creates a self-contained session directory:

    <outdir>/<label>_<timestamp>/
        bag/                # rosbag2 directory written by `ros2 bag record`
        session.json        # metadata: label, topics, duration, size, host

Run from inside the moveit_pro shell, after `moveit_pro run` has the cameras
publishing. Stop with Ctrl+C, or pass --duration N for a timed capture.

Examples:
    python3 record_dataset.py --label barn_morning_run1
    python3 record_dataset.py --label calibration --duration 30
    python3 record_dataset.py --label custom --topics /basler_cam_1/pylon_ros2_camera_node/image_raw /tf
"""

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_TOPICS = [
    # Basler streams
    "/basler_cam_1/pylon_ros2_camera_node/image_raw",
    "/basler_cam_1/pylon_ros2_camera_node/camera_info",
    "/basler_cam_2/pylon_ros2_camera_node/image_raw",
    "/basler_cam_2/pylon_ros2_camera_node/camera_info",
    # ZED rectified stereo pair
    "/zed_x/zed_node/left/image_rect_color",
    "/zed_x/zed_node/left/camera_info",
    "/zed_x/zed_node/right/image_rect_color",
    "/zed_x/zed_node/right/camera_info",
    # ZED depth + derived
    "/zed_x/zed_node/depth/depth_registered",
    "/zed_x/zed_node/depth/camera_info",
    "/zed_x/zed_node/point_cloud/cloud_registered",
    "/zed_x/zed_node/confidence/confidence_map",
    "/zed_x/zed_node/disparity/disparity_image",
    # ZED IMU
    "/zed_x/zed_node/imu/data",
    "/zed_x/zed_node/imu/data_raw",
    "/zed_x/zed_node/temperature/imu",
    "/zed_x/zed_node/left_cam_imu_transform",
    # ZED positional tracking
    "/zed_x/zed_node/odom",
    "/zed_x/zed_node/pose",
    # ZED depth metadata
    "/zed_x/zed_node/depth/depth_info",
    # Frame tree
    "/tf",
    "/tf_static",
]

# Default to the NVMe scratch on this Jetson — the eMMC root only has
# ~15 GB free and can't keep up with native-HD1200 ZED streams. The
# customer rebuild may have a different storage layout; override with
# --outdir if so.
DEFAULT_OUTDIR = Path("/nvme/datasets")


def list_active_topics():
    try:
        out = subprocess.check_output(
            ["ros2", "topic", "list"], stderr=subprocess.DEVNULL, timeout=10
        )
        return set(out.decode().splitlines())
    except (subprocess.SubprocessError, FileNotFoundError):
        return set()


def bag_size_mb(bag_dir: Path) -> float:
    if not bag_dir.exists():
        return 0.0
    return sum(p.stat().st_size for p in bag_dir.rglob("*") if p.is_file()) / (1024 ** 2)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--label", required=True,
                    help="short identifier for this session (no spaces or slashes); "
                         "becomes part of the output directory name")
    ap.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR,
                    help=f"parent directory for sessions (default: {DEFAULT_OUTDIR})")
    ap.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS,
                    help="topics to record (default: all harvest_moon camera topics)")
    ap.add_argument("--duration", type=int, default=0,
                    help="auto-stop after N seconds (default: 0 = run until Ctrl+C)")
    ap.add_argument("--compress", action="store_true",
                    help="enable zstd compression (smaller bags, more CPU)")
    ap.add_argument("--no-check", action="store_true",
                    help="skip pre-flight check that topics are publishing")
    args = ap.parse_args()

    if " " in args.label or "/" in args.label:
        sys.exit("error: --label must not contain spaces or slashes")

    if not args.topics:
        sys.exit("error: --topics list is empty")

    if not args.no_check:
        print("Checking topics are publishing...")
        active = list_active_topics()
        if not active:
            print("WARNING: `ros2 topic list` returned nothing. "
                  "Is ROS sourced and the moveit_pro session running?")
        missing = [t for t in args.topics if t not in active]
        if missing:
            print("WARNING: the following requested topics are not currently active:")
            for t in missing:
                print(f"  - {t}")
            print()
            try:
                resp = input("Record anyway? [y/N] ").strip().lower()
            except KeyboardInterrupt:
                sys.exit("\nAborted.")
            if resp != "y":
                sys.exit("Aborted.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = args.outdir / f"{args.label}_{timestamp}"
    bag_dir = session_dir / "bag"

    if session_dir.exists():
        sys.exit(f"error: {session_dir} already exists")

    args.outdir.mkdir(parents=True, exist_ok=True)
    session_dir.mkdir()

    cmd = ["ros2", "bag", "record", "-o", str(bag_dir)]
    if args.compress:
        cmd += ["--compression-mode", "file", "--compression-format", "zstd"]
    cmd += list(args.topics)

    meta = {
        "label": args.label,
        "timestamp": timestamp,
        "topics": list(args.topics),
        "compressed": args.compress,
        "host": os.uname().nodename,
        "command": " ".join(shlex.quote(c) for c in cmd),
    }
    with (session_dir / "session.json").open("w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nSession: {session_dir}")
    print(f"Topics ({len(args.topics)}):")
    for t in args.topics:
        print(f"  {t}")
    if args.duration:
        print(f"Auto-stop after {args.duration} s.")
    else:
        print("Press Ctrl+C to stop.")
    print()

    t_start = time.time()
    proc = subprocess.Popen(cmd)
    last_log = 0.0

    try:
        while proc.poll() is None:
            time.sleep(0.5)
            elapsed = time.time() - t_start
            if elapsed - last_log >= 5.0:
                size = bag_size_mb(bag_dir)
                print(f"  [{elapsed:6.1f} s]  {size:8.1f} MB", flush=True)
                last_log = elapsed
            if args.duration and elapsed >= args.duration:
                print(f"\n{args.duration} s elapsed — stopping.")
                proc.send_signal(signal.SIGINT)
                break
    except KeyboardInterrupt:
        print("\nCtrl+C received — stopping cleanly...")
        proc.send_signal(signal.SIGINT)

    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        print("ros2 bag did not exit in 15 s; killing.")
        proc.kill()
        proc.wait()

    duration = time.time() - t_start
    size_mb = bag_size_mb(bag_dir)

    meta["duration_s"] = round(duration, 2)
    meta["size_mb"] = round(size_mb, 2)
    with (session_dir / "session.json").open("w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nStopped after {duration:.1f} s.")
    print(f"Bag size: {size_mb:.1f} MB ({size_mb / 1024:.2f} GB)")
    print(f"Session dir: {session_dir}")


if __name__ == "__main__":
    main()
