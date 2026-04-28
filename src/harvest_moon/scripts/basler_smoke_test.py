"""
basler_smoke_test.py

Standalone smoke test for one Basler camera. Loads the factory Default UserSet,
disables triggering, sets a long fixed exposure, grabs a few frames in
free-run mode, and reports mean intensity + saves the images.

Used to determine whether a "black image" problem is in our config or in the
camera/sensor itself.

Usage:
  python3 basler_smoke_test.py --serial 25435376
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
from PIL import Image
from pypylon import pylon


def open_basler_by_serial(serial):
    tl = pylon.TlFactory.GetInstance()
    for info in tl.EnumerateDevices():
        if info.GetSerialNumber() == serial:
            return pylon.InstantCamera(tl.CreateDevice(info))
    print(f"No Basler found with serial {serial}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--serial', required=True)
    ap.add_argument('--exposure-us', type=float, default=20000.0,
                    help='Exposure in microseconds (default: 20000 = 20 ms)')
    ap.add_argument('--count', type=int, default=5)
    ap.add_argument('--outdir',
                    default=os.path.expanduser(
                        f"~/moveit_pro/moveit_pro_example_ws/vision_debug/smoke_{datetime.now():%Y%m%d_%H%M%S}"))
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    cam = open_basler_by_serial(args.serial)
    cam.Open()
    info = cam.GetDeviceInfo()
    print(f"Opened: {info.GetModelName()} SN={info.GetSerialNumber()}")
    print()

    print("Loading factory Default UserSet (clean slate)...")
    cam.UserSetSelector.Value = 'Default'
    cam.UserSetLoad.Execute()

    print(f"Configuring free-run mode, {args.exposure_us:.0f} µs exposure, Mono8...")
    cam.TriggerMode.Value = 'Off'
    cam.PixelFormat.Value = 'Mono8'
    cam.ExposureTimeAbs.Value = args.exposure_us
    cam.AcquisitionFrameRateEnable.Value = True
    cam.AcquisitionFrameRateAbs.Value = 5.0

    print("\nKey settings before grab:")
    print(f"  TriggerMode       = {cam.TriggerMode.Value}")
    print(f"  ExposureTimeAbs   = {cam.ExposureTimeAbs.Value}")
    print(f"  PixelFormat       = {cam.PixelFormat.Value}")
    print(f"  Width  x Height   = {cam.Width.Value} x {cam.Height.Value}")
    print(f"  OffsetX, OffsetY  = {cam.OffsetX.Value}, {cam.OffsetY.Value}")
    for name in ('GainRaw', 'BlackLevelRaw', 'GammaEnable', 'ExposureAuto', 'GainAuto'):
        try:
            print(f"  {name:18s}= {getattr(cam, name).Value}")
        except Exception:
            pass
    print()

    print(f"Grabbing {args.count} frames in free-run...")
    cam.StartGrabbingMax(args.count)
    means = []
    i = 0
    try:
        while cam.IsGrabbing():
            r = cam.RetrieveResult(2000)
            if r.GrabSucceeded():
                arr = r.GetArray()
                m = float(arr.mean())
                mx = int(arr.max())
                mn = int(arr.min())
                means.append(m)
                Image.fromarray(arr).save(f"{args.outdir}/frame_{i:03d}.png")
                print(f"  frame {i:03d}:  mean={m:6.2f}  min={mn}  max={mx}")
                i += 1
            r.Release()
    finally:
        cam.StopGrabbing()
        cam.Close()

    avg = float(np.mean(means)) if means else 0.0
    print(f"\nMean intensity over {len(means)} frames: {avg:.2f}")
    print(f"Saved to {args.outdir}")
    print()
    if avg < 5.0:
        print("RESULT: Camera returns essentially black frames in factory free-run mode.")
        print("        Hardware/sensor issue, or some persistent global setting is wrong.")
    elif avg < 30.0:
        print("RESULT: Very dim frames. Lens cap, scene blocked, or low gain.")
    else:
        print("RESULT: Real images coming out.")
        print("        The 'black frames' problem is in our triggered config, not the camera.")


if __name__ == '__main__':
    main()
