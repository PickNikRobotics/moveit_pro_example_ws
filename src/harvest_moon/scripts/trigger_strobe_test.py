"""                                                                                                
trigger_strobe_test.py                                                                             
                                                                                                    
Test the full chain: ZED Link master trigger → Basler Line 1 opto IN → camera exposes →            
Line 2 OUT (ExposureActive) → LHI-DO NPN trigger → light strobes.
                                                                                                    
Before running:                                                                                    
1. Make sure the ROS camera launch is not holding the Baslers open.                              
2. Make sure the ZED daemon is running in master mode (sync_mode=0).                             
3. Make sure 24 V power is on to the LHI-DO lights.                                              
"""                                                                                                
                                                                                                     
from pypylon import pylon
import sys
import time
                                                                                                    
NUM_FRAMES = 300                                                                                
EXPOSURE_US = 500.0       # 500 µs — within LHI-DO Deca OverDrive range
GRAB_TIMEOUT_MS = 2000    # allow plenty of margin even at slow trigger rates
                                            
devices = pylon.TlFactory.GetInstance().EnumerateDevices()
if not devices:                                                                                    
    print("No Basler cameras found.")                                                              
    sys.exit(1)                                                                                    
                                                                                                    
c = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
c.Open()
print(f"Opened: {c.GetDeviceInfo().GetModelName()} SN={c.GetDeviceInfo().GetSerialNumber()}")      
                                            
# --- Line 1 input: debounce the trigger to filter short spurious edges ---
c.LineSelector.Value = 'Line1'
c.LineDebouncerTimeAbs.Value = 3000.0   # µs; must be < real pulse width (16.67 ms)

# --- Line 2 output: route ExposureActive to LHI-DO NPN trigger ---
c.LineSelector.Value = 'Line2'
c.LineSource.Value = 'ExposureActive'
                                        
# --- Exposure settings ---                                                                        
c.ExposureTimeAbs.Value = EXPOSURE_US                                                              
c.AcquisitionFrameRateEnable.Value = False   # disable free-run; hardware trigger paces us
                                                                                                    
# --- Hardware trigger on Line 1 (from ZED) ---
# Honest caveat: ace U does NOT expose TriggerSelector='FrameStart' —                              
# only 'AcquisitionStart' and 'LineStart'. Trying AcquisitionStart first.                          
# If this only fires once and then stops, switch to 'LineStart'.                                   
c.TriggerSelector.Value = 'AcquisitionStart'                                                       
c.TriggerMode.Value = 'On'                                                                         
c.TriggerSource.Value = 'Line1'                                                                    
c.TriggerActivation.Value = 'RisingEdge'                                                           
                                                                                                    
print(f"Configured. Grabbing {NUM_FRAMES} frames (lights should strobe).")                         
                                                                                                    
c.StartGrabbingMax(NUM_FRAMES)
n = 0
prev_t = None
dts = []
try:
    while c.IsGrabbing():
        r = c.RetrieveResult(GRAB_TIMEOUT_MS)
        now = time.perf_counter()
        dt_ms = (now - prev_t) * 1000.0 if prev_t is not None else None
        prev_t = now
        n += 1
        blk = r.BlockID
        if dt_ms is None:
            print(f"frame {n:3d}: ok={r.GrabSucceeded()}  blk={blk}   (first frame)")
        else:
            print(f"frame {n:3d}: ok={r.GrabSucceeded()}  blk={blk}  dt={dt_ms:6.1f} ms")
            dts.append(dt_ms)
        r.Release()
except pylon.TimeoutException as e:
    print(f"TIMEOUT after {n} frames — no trigger arriving? {e}")
finally:
    c.Close()
    if dts:
        print(f"\nInter-frame dt stats (ms): min={min(dts):.1f}  max={max(dts):.1f}  "
              f"mean={sum(dts)/len(dts):.1f}  n={len(dts)}")
    print("done.")