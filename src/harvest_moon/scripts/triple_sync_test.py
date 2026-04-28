"""
triple_sync_test.py

Qualitative sync check with ZED + both Basler cameras. Wave a hand across all
three cameras' FOVs while this captures N triplets. Each triplet is saved as:

  pair_NNN.jpg — side-by-side composite:
                 ZED (landscape, left) | Cam 1 (portrait) | Cam 2 (portrait)

Validation checks:
  - Hand appears at near-identical positions in all three panels
  - Basler BlockIDs increment by exactly 1 per frame on BOTH cameras
  - Cam 1 and Cam 2 BlockIDs stay in lockstep (same parallel wiring)
  - No glitches (double-triggers or missed triggers)

Before running:
1. Kill ROS camera processes:  pkill -f pylon_ros2_camera ; pkill -f zed_wrapper
2. Teensy in `r` mode (serial monitor → `r`)
3. From host:  python3 triple_sync_test.py --count 40
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np
from PIL import Image, ImageDraw

from pypylon import pylon
import pyzed.sl as sl

NUM_FRAMES_DEFAULT = 40
EXPOSURE_US = 500.0
GRAB_TIMEOUT_MS = 2000
COMPOSITE_HEIGHT = 540
BANNER_H = 30
BASLER_TRANSFER_WAIT_MS = 25

CAM1_SERIAL = '25233972'
CAM2_SERIAL = '25435376'

ZED_EXPOSURE_US = 100


def setup_basler(cam):
    cam.Open()
    cam.PixelFormat.Value = 'BayerRG8'
    cam.LineSelector.Value = 'Line1'
    cam.LineDebouncerTimeAbs.Value = 5.0
    cam.LineSelector.Value = 'Line2'
    cam.LineSource.Value = 'ExposureActive'
    cam.ExposureTimeAbs.Value = EXPOSURE_US
    cam.AcquisitionFrameRateEnable.Value = False
    cam.TriggerSelector.Value = 'FrameStart'
    cam.TriggerMode.Value = 'On'
    cam.TriggerSource.Value = 'Line1'
    cam.TriggerActivation.Value = 'RisingEdge'


def bayer_rg8_to_rgb(bayer):
    r = bayer[0::2, 0::2]
    g1 = bayer[0::2, 1::2].astype(np.uint16)
    g2 = bayer[1::2, 0::2].astype(np.uint16)
    b = bayer[1::2, 1::2]
    g = ((g1 + g2) // 2).astype(np.uint8)
    return np.stack([r, g, b], axis=2)


def open_basler_by_serial(serial):
    tl = pylon.TlFactory.GetInstance()
    for info in tl.EnumerateDevices():
        if info.GetSerialNumber() == serial:
            return pylon.InstantCamera(tl.CreateDevice(info))
    print(f"No Basler found with serial {serial}")
    sys.exit(1)


def open_zed(fps):
    zc = sl.Camera()
    init = sl.InitParameters()
    init.camera_fps = fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    err = zc.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ZED open failed: {err}. Try: sudo systemctl restart zed_x_daemon")
        sys.exit(1)
    zc.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)
    zc.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME, ZED_EXPOSURE_US)
    return zc


def resize_height(img, h):
    w = int(img.width * h / img.height)
    return img.resize((w, h), Image.BILINEAR)


def build_composite(z_img, b1_img, b2_img, idx, b1_blk, b2_blk):
    b1_img = b1_img.rotate(-90, expand=True)
    b2_img = b2_img.rotate(-90, expand=True)
    z = resize_height(z_img, COMPOSITE_HEIGHT)
    b1 = resize_height(b1_img, COMPOSITE_HEIGHT)
    b2 = resize_height(b2_img, COMPOSITE_HEIGHT)
    w_total = z.width + b1.width + b2.width
    comp = Image.new("RGB", (w_total, COMPOSITE_HEIGHT + BANNER_H), (20, 20, 20))
    comp.paste(z, (0, BANNER_H))
    comp.paste(b1, (z.width, BANNER_H))
    comp.paste(b2, (z.width + b1.width, BANNER_H))
    draw = ImageDraw.Draw(comp)
    draw.text((10, 8), f"ZED  frame {idx:03d}", fill=(255, 255, 255))
    draw.text((z.width + 10, 8), f"Cam1  blk={b1_blk}", fill=(255, 255, 255))
    draw.text((z.width + b1.width + 10, 8), f"Cam2  blk={b2_blk}", fill=(255, 255, 255))
    return comp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=NUM_FRAMES_DEFAULT)
    ap.add_argument("--outdir", default=f"./triple_sync_{datetime.now():%Y%m%d_%H%M%S}")
    ap.add_argument("--fps", type=int, default=15)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    bc1 = open_basler_by_serial(CAM1_SERIAL)
    bc2 = open_basler_by_serial(CAM2_SERIAL)
    setup_basler(bc1)
    setup_basler(bc2)
    print(f"Cam 1: {bc1.GetDeviceInfo().GetModelName()} "
          f"SN={bc1.GetDeviceInfo().GetSerialNumber()}")
    print(f"Cam 2: {bc2.GetDeviceInfo().GetModelName()} "
          f"SN={bc2.GetDeviceInfo().GetSerialNumber()}")

    zc = open_zed(args.fps)
    zinfo = zc.get_camera_information()
    print(f"ZED:   {zinfo.camera_model} SN={zinfo.serial_number}")

    bc1.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    bc2.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    rt = sl.RuntimeParameters()
    zmat = sl.Mat()
    print("Basler grabs started.")

    WARMUP_FRAMES = 30
    print(f"Warming up ZED for {WARMUP_FRAMES} frames to let fixed exposure settle...")
    for wu in range(WARMUP_FRAMES):
        if zc.grab(rt) != sl.ERROR_CODE.SUCCESS:
            print(f"Warmup grab {wu} failed.")
            sys.exit(1)
    print("Warmup done — entering capture loop.\n")

    print(f"Capturing {args.count} triplets at {args.fps} fps — start waving now.\n")
    z_raw, b1_raw, b2_raw, b1_blks, b2_blks, iter_ms = [], [], [], [], [], []
    for i in range(args.count):
        t0 = time.perf_counter()
        if zc.grab(rt) != sl.ERROR_CODE.SUCCESS:
            print(f"[{i:03d}] zed grab failed")
            continue
        zc.retrieve_image(zmat, sl.VIEW.LEFT)
        z_bgra = zmat.get_data()
        z_rgb = z_bgra[:, :, [2, 1, 0]].copy()

        time.sleep(BASLER_TRANSFER_WAIT_MS / 1000.0)

        r1 = r2 = None
        try:
            r1 = bc1.RetrieveResult(GRAB_TIMEOUT_MS, pylon.TimeoutHandling_Return)
            r2 = bc2.RetrieveResult(GRAB_TIMEOUT_MS, pylon.TimeoutHandling_Return)
        except pylon.TimeoutException:
            pass
        if not (r1 and r1.GrabSucceeded() and r2 and r2.GrabSucceeded()):
            print(f"[{i:03d}] basler grab failed/timeout")
            if r1:
                r1.Release()
            if r2:
                r2.Release()
            continue

        b1_arr = bayer_rg8_to_rgb(r1.GetArray())
        b2_arr = bayer_rg8_to_rgb(r2.GetArray())
        b1_blk = r1.BlockID
        b2_blk = r2.BlockID
        r1.Release()
        r2.Release()

        z_raw.append(z_rgb)
        b1_raw.append(b1_arr)
        b2_raw.append(b2_arr)
        b1_blks.append(b1_blk)
        b2_blks.append(b2_blk)
        dt = (time.perf_counter() - t0) * 1000.0
        iter_ms.append(dt)
        print(f"[{i:03d}] iter={dt:5.1f} ms  cam1_blk={b1_blk}  cam2_blk={b2_blk}")

    bc1.StopGrabbing()
    bc2.StopGrabbing()
    bc1.Close()
    bc2.Close()
    zc.close()

    print(f"\nCaptured {len(z_raw)}/{args.count} triplets.")
    if iter_ms:
        period_ms = 1000.0 / args.fps
        print(f"Iter time: mean={np.mean(iter_ms):.1f} ms  "
              f"max={np.max(iter_ms):.1f} ms  (ZED period={period_ms:.1f} ms)")

    if len(b1_blks) > 1:
        b1_deltas = np.diff(b1_blks)
        b2_deltas = np.diff(b2_blks)
        print(f"Cam 1 BlockID deltas: min={b1_deltas.min()} max={b1_deltas.max()} "
              f"(expected all 1)")
        print(f"Cam 2 BlockID deltas: min={b2_deltas.min()} max={b2_deltas.max()} "
              f"(expected all 1)")
        if np.all(b1_deltas == 1) and np.all(b2_deltas == 1):
            print("OK: Both Baslers captured sequential frames with no double/missed triggers.")
        else:
            print("WARN: Non-unit BlockID delta detected — review trigger signal/debouncer.")

    print("\nSaving to disk...")
    for i, (z, b1, b2) in enumerate(zip(z_raw, b1_raw, b2_raw)):
        z_img = Image.fromarray(z)
        b1_img = Image.fromarray(b1)
        b2_img = Image.fromarray(b2)
        comp = build_composite(z_img, b1_img, b2_img, i, b1_blks[i], b2_blks[i])
        comp.save(f"{args.outdir}/pair_{i:03d}.jpg", quality=95)

    print(f"Saved to {args.outdir}")
    print("View composites:")
    print(f"  eog {args.outdir}/pair_000.jpg")


if __name__ == "__main__":
    main()
