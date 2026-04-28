"""
provision_basler.py

One-time provisioning script: configures a Basler camera with the
trigger/exposure/line settings used by the harvest_moon system, saves to
UserSet1, and sets UserSet1 as the startup default. After running this, the
camera will boot with this config every power cycle.

Usage:
  pkill -f pylon_ros2_camera   # ensure no ROS process is holding the camera
  python3 provision_basler.py --serial 25233972     # Cam 1
  python3 provision_basler.py --serial 25435376     # Cam 2

Settings applied here mirror our tuned vision_debug scripts. If you change a
value (e.g., debouncer time or exposure), edit this file and re-run.
"""

import argparse
import sys
from pypylon import pylon

# Settings — keep these in sync with vision_debug test scripts.
EXPOSURE_US = 500.0
LINE_DEBOUNCER_US = 5.0
PIXEL_FORMAT = 'BayerRG8'


def open_basler_by_serial(serial):
    tl = pylon.TlFactory.GetInstance()
    for info in tl.EnumerateDevices():
        if info.GetSerialNumber() == serial:
            return pylon.InstantCamera(tl.CreateDevice(info))
    print(f"No Basler found with serial {serial}")
    sys.exit(1)


def configure(cam):
    print("Applying configuration to working memory...")

    cam.PixelFormat.Value = PIXEL_FORMAT

    cam.LineSelector.Value = 'Line1'
    cam.LineDebouncerTimeAbs.Value = LINE_DEBOUNCER_US

    cam.LineSelector.Value = 'Line2'
    cam.LineSource.Value = 'ExposureActive'

    cam.ExposureTimeAbs.Value = EXPOSURE_US
    cam.AcquisitionFrameRateEnable.Value = False

    cam.TriggerSelector.Value = 'FrameStart'
    cam.TriggerMode.Value = 'On'
    cam.TriggerSource.Value = 'Line1'
    cam.TriggerActivation.Value = 'RisingEdge'

    # Chunk timestamps DISABLED.
    # We tried enabling these to get hardware capture timestamps in the ROS
    # message header. The pylon wrapper does overwrite header.stamp with
    # the chunk timestamp, but the chunk timestamp is in the camera's own
    # clock domain (nanoseconds since camera power-on), NOT ROS time. That
    # makes it useless for sync with the ZED (which uses ROS time).
    # Result: timestamps drift apart by camera-uptime-since-boot,
    # ApproximateTimeSynchronizer can't pair frames at all.
    # Until the wrapper offers a "convert chunk timestamp to ROS time"
    # option (or we add that ourselves), keep chunk mode off and use
    # receive-time stamps with a wider slop in downstream consumers.
    cam.ChunkModeActive.Value = False

    print("  done.")


def save_to_userset(cam, user_set):
    print(f"Saving working memory to {user_set}...")
    cam.UserSetSelector.Value = user_set
    cam.UserSetSave.Execute()

    print(f"Setting {user_set} as the power-on default...")
    cam.UserSetDefaultSelector.Value = user_set
    print("  done.")


def verify(cam, user_set):
    print(f"\nVerifying {user_set} by re-loading and reading back...")
    cam.UserSetSelector.Value = user_set
    cam.UserSetLoad.Execute()

    print(f"  PixelFormat        = {cam.PixelFormat.Value}")
    print(f"  ExposureTimeAbs    = {cam.ExposureTimeAbs.Value}")
    print(f"  TriggerMode        = {cam.TriggerMode.Value}")
    print(f"  TriggerSource      = {cam.TriggerSource.Value}")
    print(f"  TriggerSelector    = {cam.TriggerSelector.Value}")
    print(f"  TriggerActivation  = {cam.TriggerActivation.Value}")

    cam.LineSelector.Value = 'Line1'
    print(f"  Line1 LineDebouncerTimeAbs = {cam.LineDebouncerTimeAbs.Value}")

    cam.LineSelector.Value = 'Line2'
    print(f"  Line2 LineSource = {cam.LineSource.Value}")

    print(f"  UserSetDefaultSelector     = {cam.UserSetDefaultSelector.Value}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--serial', required=True,
                    help='Camera serial number (e.g. 25233972 for Cam 1)')
    ap.add_argument('--user-set', default='UserSet1',
                    help='UserSet slot to write to (default: UserSet1)')
    args = ap.parse_args()

    cam = open_basler_by_serial(args.serial)
    cam.Open()
    info = cam.GetDeviceInfo()
    print(f"Opened: {info.GetModelName()} SN={info.GetSerialNumber()}")
    print()

    try:
        configure(cam)
        save_to_userset(cam, args.user_set)
        verify(cam, args.user_set)
    finally:
        cam.Close()

    print(f"\nDone. SN {args.serial} is now provisioned with {args.user_set}.")
    print("Power-cycle the camera (or PoE) to confirm the config loads on boot.")


if __name__ == '__main__':
    main()
