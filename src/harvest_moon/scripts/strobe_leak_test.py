"""
strobe_leak_test.py

Qualitative strobe-leak sync check:
ZED fires trigger at end of ZED exposure (master mode, sync_mode=0 in daemon).
Basler exposes on rising edge; its Line 2 (ExposureActive) drives the LHI-DO.
Since the strobe fires AFTER the ZED shutter closes, with ambient light low
the ZED frame should be dark and the Basler frame should be bright.

Before running:
1. Exit the MoveIt Pro shell and kill any ROS camera processes holding the
   cameras open, e.g.:  pkill -f pylon_ros2_camera ; pkill -f zed_wrapper
2. Confirm zed_x_daemon is up (sync_mode=0). If ZED open fails, try:
     sudo systemctl restart zed_x_daemon
3. Dim ambient lights as much as practical.
4. Run from the host:
     python3 strobe_leak_test.py --count 20
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
from PIL import Image

from pypylon import pylon
import pyzed.sl as sl

NUM_FRAMES_DEFAULT = 20
EXPOSURE_US = 500.0
GRAB_TIMEOUT_MS = 2000


def setup_basler(cam):
    cam.Open()
    cam.LineSelector.Value = 'Line2'
    cam.LineSource.Value = 'ExposureActive'
    cam.ExposureTimeAbs.Value = EXPOSURE_US
    cam.AcquisitionFrameRateEnable.Value = False
    cam.TriggerSelector.Value = 'FrameStart'
    cam.TriggerMode.Value = 'On'
    cam.TriggerSource.Value = 'Line1'
    cam.TriggerActivation.Value = 'RisingEdge'


def open_zed(fps):
    zc = sl.Camera()
    init = sl.InitParameters()
    init.camera_fps = fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    err = zc.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"ZED open failed: {err}. Try: sudo systemctl restart zed_x_daemon")
        sys.exit(1)
    return zc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=NUM_FRAMES_DEFAULT)
    ap.add_argument("--outdir", default=f"./strobe_leak_{datetime.now():%Y%m%d_%H%M%S}")
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

    bc.StartGrabbing(pylon.GrabStrategy_OneByOne)

    rt = sl.RuntimeParameters()
    zmat = sl.Mat()
    zed_means, basler_means = [], []

    print(f"\nCapturing {args.count} pairs. Dim the room now.\n")
    for i in range(args.count):
        if zc.grab(rt) != sl.ERROR_CODE.SUCCESS:
            print(f"[{i:03d}] zed grab failed")
            continue
        zc.retrieve_image(zmat, sl.VIEW.LEFT)
        z_bgra = zmat.get_data()
        z_gray = z_bgra[:, :, :3].mean(axis=2).astype(np.uint8)

        try:
            result = bc.RetrieveResult(GRAB_TIMEOUT_MS, pylon.TimeoutHandling_Return)
        except pylon.TimeoutException:
            result = None
        if result is None or not result.GrabSucceeded():
            print(f"[{i:03d}] basler grab failed/timeout")
            if result:
                result.Release()
            continue
        b_arr = result.GetArray()
        result.Release()

        z_mean = float(z_gray.mean())
        b_mean = float(b_arr.mean())
        ratio = b_mean / z_mean if z_mean > 1e-6 else float('inf')
        zed_means.append(z_mean)
        basler_means.append(b_mean)

        print(f"[{i:03d}] zed={z_mean:6.2f}  basler={b_mean:6.2f}  ratio={ratio:5.2f}x")

        Image.fromarray(z_gray).save(f"{args.outdir}/zed_{i:03d}.png")
        Image.fromarray(b_arr).save(f"{args.outdir}/basler_{i:03d}.png")

    bc.StopGrabbing()
    bc.Close()
    zc.close()

    if not zed_means:
        print("\nNo successful pairs captured.")
        sys.exit(2)

    z_avg = float(np.mean(zed_means))
    b_avg = float(np.mean(basler_means))
    print("\n=== Summary ===")
    print(f"ZED    mean intensity: avg={z_avg:6.2f}  std={np.std(zed_means):5.2f}  n={len(zed_means)}")
    print(f"Basler mean intensity: avg={b_avg:6.2f}  std={np.std(basler_means):5.2f}  n={len(basler_means)}")
    print(f"Ratio  (Basler/ZED):   {b_avg/max(z_avg,1e-6):.2f}x")
    print(f"Saved to: {args.outdir}")
    print("\nInterpretation:")
    print("  ratio >> 1 (e.g. >5x): strobe illuminates only Basler, sync direction correct.")
    print("  ratio ~= 1:            ambient too bright or exposures overlap — dim more, retry.")
    print("  ratio < 1:             unexpected — verify trigger pulse is reaching the Basler.")


if __name__ == "__main__":
    main()
