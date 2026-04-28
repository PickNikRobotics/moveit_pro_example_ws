"""
basler_trigger_enums.py

Enumerates the allowed values for TriggerSelector, TriggerActivation, and
TriggerSource on the connected Basler camera. Useful before trying a different
trigger mode, to confirm what this specific camera model + firmware supports.
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

def dump_enum(name):
    try:
        node = getattr(cam, name)
        current = node.Value
        symbolics = node.Symbolics  # list of allowed string values
        print(f"  {name}")
        print(f"    current: {current}")
        print(f"    allowed: {symbolics}")
    except AttributeError:
        print(f"  {name}: (node not present on this camera)")
    except Exception as e:
        print(f"  {name}: (error: {e})")
    print()

print("=== Trigger-related enum nodes ===")
for name in ['TriggerSelector', 'TriggerActivation', 'TriggerSource', 'TriggerMode']:
    dump_enum(name)

cam.Close()
