"""
basler_feature_dump.py

Dumps all Basler camera features whose names relate to triggers, lines,
debouncing, filtering, or input conditioning. Used to discover the exact
node name for the input debouncer on this specific camera + firmware.
"""

from pypylon import pylon

cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
cam.Open()

print(f"Camera: {cam.GetDeviceInfo().GetModelName()} "
      f"SN={cam.GetDeviceInfo().GetSerialNumber()}")
try:
    print(f"Firmware: {cam.DeviceFirmwareVersion.Value}")
except Exception:
    pass
print()

# Some features are per-line; select Line 1 (our trigger input) so we read
# the right instance of line-specific nodes.
try:
    cam.LineSelector.Value = 'Line1'
    print("LineSelector set to Line1")
except Exception as e:
    print(f"LineSelector could not be set to Line1: {e}")
print()

keywords = ['debounce', 'filter', 'input', 'line', 'trigger']
print(f"=== Features matching: {', '.join(keywords)} ===")
for name in sorted(dir(cam)):
    if name.startswith('_'):
        continue
    if not any(k in name.lower() for k in keywords):
        continue
    try:
        attr = getattr(cam, name)
        if hasattr(attr, 'Value'):
            val = attr.Value
            print(f"  {name} = {val}")
        elif hasattr(attr, 'ToString'):
            print(f"  {name} = {attr.ToString()}")
    except Exception as e:
        print(f"  {name} (cannot read: {e})")

cam.Close()
