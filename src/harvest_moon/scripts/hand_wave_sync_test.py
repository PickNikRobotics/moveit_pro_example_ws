"""
hand_wave_sync_test.py

Qualitative motion-sync check.
Wave a hand (or any high-contrast object) across both cameras' FOV while this
captures N frame pairs. Each pair is saved three ways:

  zed_NNN.png    — raw ZED left view
  basler_NNN.png — raw Basler image
  pair_NNN.png   — side-by-side composite (ZED left, Basler right)

Flip through the `pair_NNN.png` files in an image viewer and look for the hand
appearing at different positions in the two panels. Because the ZED trigger
fires at end-of-ZED-exposure, the ZED captures the hand ~(E_zed + E_basler)/2
earlier — so for a hand moving left-to-right, the ZED image will show the hand
slightly further right than the Basler image of the "same" pair.

Use bright ambient lighting so the ZED auto-exposure stays short and doesn't
motion-blur. The Basler's 500 us exposure + strobe handles itself.

Before running:
1. Kill ROS camera processes:  pkill -f pylon_ros2_camera ; pkill -f zed_wrapper
2. From host:  python3 hand_wave_sync_test.py --count 40
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
BASLER_EXPOSURE_US = 2000.0
GRAB_TIMEOUT_MS = 2000
COMPOSITE_HEIGHT = 540
BANNER_H = 30
BASLER_TRANSFER_WAIT_MS = 25

def setup_basler(cam):
    cam.Open()
    cam.LineSelector.Value = 'Line1'
    cam.LineDebouncerTimeAbs.Value = 50.0
    cam.LineSelector.Value = 'Line2'
    cam.LineSource.Value = 'ExposureActive'
    cam.ExposureTimeAbs.Value = BASLER_EXPOSURE_US
    cam.AcquisitionFrameRateEnable.Value = False
    cam.TriggerSelector.Value = 'FrameStart'
    cam.TriggerMode.Value = 'On'
    cam.TriggerSource.Value = 'Line1'
    cam.TriggerActivation.Value = 'RisingEdge'

ZED_EXPOSURE_US = 100   # match Basler exposure for symmetric mid-exposure timing

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


def build_composite(z_img, b_img, idx):
    b_img = b_img.rotate(-90, expand=True)
    z = resize_height(z_img, COMPOSITE_HEIGHT)
    b = resize_height(b_img, COMPOSITE_HEIGHT)
    w_total = z.width + b.width
    comp = Image.new("RGB", (w_total, COMPOSITE_HEIGHT + BANNER_H), (20, 20, 20))
    comp.paste(z, (0, BANNER_H))
    comp.paste(b, (z.width, BANNER_H))
    draw = ImageDraw.Draw(comp)
    draw.text((10, 8), f"ZED  frame {idx:03d}", fill=(255, 255, 255))
    draw.text((z.width + 10, 8), f"Basler  frame {idx:03d}", fill=(255, 255, 255))
    return comp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=NUM_FRAMES_DEFAULT)
    ap.add_argument("--outdir", default=f"./hand_wave_{datetime.now():%Y%m%d_%H%M%S}")
    ap.add_argument("--fps", type=int, default=15)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    if not devices:
        print("No Basler cameras found.")
        sys.exit(1)
    bc = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    setup_basler(bc)
    print(f"Basler: {bc.GetDeviceInfo().GetModelName()} "
          f"SN={bc.GetDeviceInfo().GetSerialNumber()}")

    zc = open_zed(args.fps)
    zinfo = zc.get_camera_information()
    print(f"ZED:    {zinfo.camera_model} SN={zinfo.serial_number}")

    bc.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    rt = sl.RuntimeParameters()
    zmat = sl.Mat()
    print("Basler grab started.")

    WARMUP_FRAMES = 30
    print(f"Warming up ZED for {WARMUP_FRAMES} frames to let fixed exposure settle...")
    for wu in range(WARMUP_FRAMES):
        if zc.grab(rt) != sl.ERROR_CODE.SUCCESS:
            print(f"Warmup grab {wu} failed.")
            sys.exit(1)
        cur_exp = zc.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME)
        if wu % 5 == 0 or wu == WARMUP_FRAMES - 1:
            print(f"  warmup [{wu:03d}] current exposure = {cur_exp} µs")
    print("Warmup done — entering capture loop.\n")

    print(f"Capturing {args.count} pairs at {args.fps} fps — start waving now.\n")
    z_raw, b_raw, iter_ms, z_exposures = [], [], [], []
    for i in range(args.count):
        t0 = time.perf_counter()
        if zc.grab(rt) != sl.ERROR_CODE.SUCCESS:
            print(f"[{i:03d}] zed grab failed")
            continue
        zc.retrieve_image(zmat, sl.VIEW.LEFT)
        z_bgra = zmat.get_data()
        z_rgb = z_bgra[:, :, [2, 1, 0]].copy()
        z_exposures.append(zc.get_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME))

        time.sleep(BASLER_TRANSFER_WAIT_MS / 1000.0)

        try:
            result = bc.RetrieveResult(GRAB_TIMEOUT_MS, pylon.TimeoutHandling_Return)
        except pylon.TimeoutException:
            result = None
        if result is None or not result.GrabSucceeded():
            print(f"[{i:03d}] basler grab failed/timeout")
            if result:
                result.Release()
            continue
        b_arr = result.GetArray().copy()
        result.Release()

        z_raw.append(z_rgb)
        b_raw.append(b_arr)
        dt = (time.perf_counter() - t0) * 1000.0
        iter_ms.append(dt)
        print(f"[{i:03d}] captured  iter={dt:5.1f} ms  zed_exp={z_exposures[-1]} µs")

    bc.StopGrabbing()
    bc.Close()
    zc.close()

    print(f"\nCaptured {len(z_raw)}/{args.count} pairs.")
    if iter_ms:
        period_ms = 1000.0 / args.fps
        print(f"Iter time: mean={np.mean(iter_ms):.1f} ms  "
              f"max={np.max(iter_ms):.1f} ms  (ZED period={period_ms:.1f} ms)")
        if np.max(iter_ms) > period_ms + 5:
            print("WARNING: some iterations exceeded ZED period — pairing may drift.")

    print("\nSaving to disk...")
    for i, (z, b) in enumerate(zip(z_raw, b_raw)):
        z_img = Image.fromarray(z)
        b_img = Image.fromarray(b).convert("RGB")
        # z_img.save(f"{args.outdir}/zed_{i:03d}.jpg)
        # b_img.save(f"{args.outdir}/basler_{i:03d}.jpg")
        build_composite(z_img, b_img, i).save(f"{args.outdir}/pair_{i:03d}.jpg")

    print(f"Saved to {args.outdir}")
    print("View composites:")
    print(f"  eog {args.outdir}/pair_000.jpg      # arrow keys to advance")
    print(f"  # or: xdg-open {args.outdir}")


if __name__ == "__main__":
    main()
