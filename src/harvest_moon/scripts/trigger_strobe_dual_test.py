"""
trigger_strobe_dual_test.py

Drive BOTH Baslers via the ZED master trigger, with each camera driving its
own LHI-DO light via Line 2. Confirms:
  - ZED can drive both opto inputs in parallel
  - Both cameras receive the trigger
  - Both lights strobe on each ZED pulse

Prereqs (same as the single-camera version):
  - ROS camera launch NOT running
  - ZED daemon in master mode (sync_mode=0) and streaming
  - 24 V rail on to both lights
"""

from pypylon import pylon
import sys

NUM_FRAMES_PER_CAM = 30
EXPOSURE_US = 500.0
GRAB_TIMEOUT_MS = 3000  # extra margin for the poll loop

devices = pylon.TlFactory.GetInstance().EnumerateDevices()
if len(devices) < 2:
    print(f"Expected 2 Basler cameras, found {len(devices)}.")
    sys.exit(1)

cams = pylon.InstantCameraArray(len(devices))
for i, dev in enumerate(devices):
    cams[i].Attach(pylon.TlFactory.GetInstance().CreateDevice(dev))

cams.Open()

for i, c in enumerate(cams):
    sn = c.GetDeviceInfo().GetSerialNumber()
    print(f"Configuring cam[{i}] SN={sn}")
    c.LineSelector.Value = 'Line2'
    c.LineSource.Value = 'ExposureActive'
    c.ExposureTimeAbs.Value = EXPOSURE_US
    c.AcquisitionFrameRateEnable.Value = False
    c.TriggerSelector.Value = 'AcquisitionStart'
    c.TriggerMode.Value = 'On'
    c.TriggerSource.Value = 'Line1'
    c.TriggerActivation.Value = 'RisingEdge'

TOTAL_FRAMES = NUM_FRAMES_PER_CAM * cams.GetSize()
print(f"\nGrabbing {TOTAL_FRAMES} frames ({NUM_FRAMES_PER_CAM} per camera).")
print("Both lights should strobe in sync with ZED trigger (~15 Hz).")

cams.StartGrabbing(pylon.GrabStrategy_OneByOne)

counts = [0] * cams.GetSize()
grabbed = 0
try:
    while grabbed < TOTAL_FRAMES:
        r = cams.RetrieveResult(GRAB_TIMEOUT_MS)
        idx = r.GetCameraContext()
        counts[idx] += 1
        grabbed += 1
        if r.GrabSucceeded():
            print(f"frame {grabbed:3d}: cam[{idx}] OK")
        else:
            print(f"frame {grabbed:3d}: cam[{idx}] FAIL err=0x{r.GetErrorCode():08x} '{r.GetErrorDescription()}'")
        r.Release()
except pylon.TimeoutException as e:
    print(f"\nTIMEOUT after {grabbed} frames: {e}")
finally:
    cams.StopGrabbing()
    cams.Close()
    print(f"\nDone. Per-camera frame counts: {counts}")
