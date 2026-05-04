# C. Field Troubleshooting Runbook

Symptom-driven. Each entry: what you see, the likely cause, the fix. In
roughly the order you're likely to hit them.

If something here doesn't resolve it, see the deeper diagnostics in
[Troubleshooting & Known Issues — Deep Dives](J_deep_troubleshooting.md).

---

## §1. Power and link checks

### 1.1 No PWR1 LED on the TrendNet switch

**Likely cause:** 24 V supply not on, or DIN-rail terminal not seated.

**Fix:** check 24 V output on the bench supply / DIN rail with a multimeter
(should read 24 V ± 0.5 V at the TrendNet's `V+` / `V−` terminals). If 24 V
is absent at the terminals but present at the supply, re-seat the DIN
connector.

### 1.2 RLY LED

In normal operation, the RLY LED is solid orange.

### 1.3 Switch port to Jetson is dark

**Likely cause:** Cat5e uplink cable unseated, or Jetson `eno1` interface is
down.

**Fix:**
1. Re-seat the cable on both ends.
2. From a Jetson terminal: `ip link show eno1` — should show `state UP`. If
   `DOWN`, run `sudo ip link set eno1 up`.
3. If still dark, swap the Cat5e cable for a known-good one.

### 1.4 Switch port to a Basler is dark or only green (no orange)

**Likely cause:** Cat6 cable problem, or the Basler isn't pulling PoE.

- Solid green only (no orange) = link without PoE delivery. The Basler isn't
  drawing power.
- Both LEDs off = no link.

**Fix:**
1. Re-seat both ends of the Cat6 cable.
2. Try the same camera on a known-working switch port.
3. Try a known-good cable on the suspect port.
4. If the camera still won't pull PoE on any port, the Basler may have
   blown its PoE input — escalate.

### 1.5 Teensy LED not blinking

**Likely cause:** Teensy not enumerated, or firmware not running, or USB
unplugged.

**Fix:**
1. Check the Teensy's USB cable is seated at both ends.
2. From the Jetson: `ls /dev/ttyACM*` — should show `/dev/ttyACM0`.
3. Open a serial monitor (`screen /dev/ttyACM0 115200`, exit with
   `Ctrl-A k y`) — you should see `Teensy trigger ready.` printed once.
   Type `c` and Enter to print counters.
4. If the Teensy is enumerated but the LED isn't blinking, type `r` in the
   serial monitor to start the trigger loop. The firmware boots in
   `RUNNING` mode by default, so this shouldn't normally be needed — but
   it's the recovery if someone has typed `s` (stop) previously.
5. If the device doesn't enumerate at all, see
   [G. Software Setup — Teensy provisioning](G_software_setup.md) to reflash.

---

## §2. ZED won't start

This is the most common launch failure. Symptoms include the ZED node
crashing during `moveit_pro run` startup, or one of these messages in the
launch log:

- `CAMERA NOT DETECTED`
- `CAMERA STREAM FAILED TO START`
- `ZED ARGUS TIMEOUT`

### 2.1 Cure-all: restart the ZED daemon

90% of ZED launch failures are fixed by:

```
sudo systemctl restart zed_x_daemon
```

Then re-run `moveit_pro run`. If it still fails on the next launch, restart
the daemon a second time and wait 10 seconds before relaunching.

### 2.2 ZED daemon refuses to start

If `sudo systemctl status zed_x_daemon` shows `failed`:

1. Power-cycle the 24 V rail (so the ZED Link Quad gets a fresh start).
2. After Jetson sees power, restart the daemon: `sudo systemctl restart zed_x_daemon`.
3. Confirm it stays up: `sudo systemctl status zed_x_daemon` should be
   `active (running)`.

### 2.3 ZED comes up but with depth errors

The harvest_moon config sets `depth_mode: 'NONE'` (in
`zed_x_overrides.yaml`) precisely because the moveit_pro container's ZED
SDK lacks the TensorRT models needed for depth. If you see depth-mode
errors, confirm `~/user_ws/install/harvest_moon/share/harvest_moon/config/zed_x_overrides.yaml`
still has `depth_mode: 'NONE'` — if it's been changed to anything else,
revert it.

---

## §3. Topic rates wrong

### 3.1 `ros2 topic hz` prints nothing for a Basler topic

**Likely cause:** the camera isn't getting trigger pulses, OR the wrapper
node failed to open the camera.

**Fix:**
1. Check the launch log for that camera node. Look for `"Failed to open
   device"` or similar.
2. Confirm the Teensy is firing — LED should be blinking (see §1.5).
3. Cycle the Teensy: in a serial monitor (`screen /dev/ttyACM0 115200`),
   type `s` (stop), then `r` (run).
4. Restart the cameras: `Ctrl+C` `moveit_pro run`, relaunch.

### 3.2 `ros2 topic hz` shows the right rate but published frames are stale

**Symptom:** topic rate is 5 Hz on the Baslers, but the image content is
seconds behind real time (looks fine in timestamps but content is stale).

**Likely cause:** the pylon wrapper buffered up frames at startup. This is
why we set `grab_strategy: 1` (LatestImageOnly) in
`basler_cam_*.yaml` — if that's been changed back to 0 (FIFO default), the
buffering bug returns.

**Fix:** confirm `grab_strategy: 1` in both
`~/user_ws/install/harvest_moon/share/harvest_moon/config/basler_cam_1.yaml`
and `basler_cam_2.yaml`. If it was missing or set to 0, fix it, rebuild
(`colcon build --packages-select harvest_moon`), and relaunch.

### 3.3 Basler rates are below 5 Hz, with stalls

**Likely cause:** GigE bandwidth contention or wrapper grab-thread lag.

**First check:** is `BASLER_DIV` actually 3 in the running Teensy firmware?
Open a serial monitor, type `c` for counters — the ratio of `zed:basler`
should be roughly `3:1`. If it's `1:1`, the firmware was flashed in
debug/test mode (DIV=1) and dual-cam at 15 Hz overruns the network path.
Reflash with `BASLER_DIV = 3`:

```
cd /home/picknik/teensy_trigger    # on host, outside moveit_pro shell
pio run -t upload
```

If `BASLER_DIV` is correct and rates are still flaky, see [Troubleshooting
Deep Dives — GigE bandwidth](J_deep_troubleshooting.md).

### 3.4 ZED rate is below 15 Hz

**Likely cause:** ZED grab thread starved, or `grab_frame_rate` in
`zed_x_overrides.yaml` was changed.

**Fix:**
1. Confirm `general.grab_frame_rate: 15` in `zed_x_overrides.yaml`.
2. If correct and still slow, restart the ZED daemon (§2.1) and relaunch
   `moveit_pro run`.

### 3.5 Cam 1 and Cam 2 are publishing at different rates

**Likely cause:** uneven GigE bandwidth distribution between the two
cameras, or one camera is missing triggers.

**Fix:** check Teensy counters with `c` in serial monitor — `basler` count
is the trigger count, both cameras should be receiving the same count.
- If counts match but ROS topic rates differ → wrapper-side issue.
  Restart `moveit_pro run`.
- If one camera's image is dark/black and the other isn't → see §4.

---

## §4. Image looks wrong

### 4.1 One camera shows black or near-black frames

**Likely causes (in order):**

1. **Lighting/strobe FOV mismatch.** The strobe may not be aimed at that
   camera's scene. Check that the LHI-DO is illuminating the field of view.
2. **Lens cap on.** Genuinely worth checking before going deep.
3. **Exposure too short for ambient.** Default `exposure: 500.0` (µs) is
   tuned for strobe-dominated illumination. In ambient-only conditions
   (e.g. strobe disabled or out of view), bump exposure — see
   [B. Operations & Tuning Playbook §1](B_operations.md).
4. **Camera not receiving trigger.** §3.1 checks apply.

### 4.2 Both cameras show black or near-black

**Likely cause:** strobe not firing, exposure too short, or trigger chain
broken.

**Fix:**
1. Check 24 V to LHI-DO strobes (Brown wire, expect 24 V).
2. Check Basler `Yellow` (Line 2 out) is going to Light `White` (NPN
   trigger). Visually verify wiring at the cable bundle.
3. If wiring is intact, bump Basler exposure temporarily to confirm the
   cameras are otherwise functional — see [B. Operations & Tuning §1](B_operations.md).

### 4.3 Frames are uniformly bright / saturated

**Likely cause:** exposure too long, or strobe firing twice, or ambient
overexposing the scene.

**Fix:** lower `exposure` in `basler_cam_*.yaml`, rebuild, relaunch. See
[B. Operations & Tuning §1](B_operations.md).

### 4.4 Color looks wrong (purple, green tint)

**Likely cause:** Bayer demosaic mismatch. At the topic level we publish
`bayer_rggb8` raw; demosaic happens downstream. If color is wrong, the
downstream consumer is applying the wrong pattern.

### 4.5 Frames are out of focus

Not a software issue. See [B. Operations & Tuning §3 (Focus tuning)](B_operations.md).

---

## §5. Sync / timestamp problems

### 5.1 Images from the two cameras don't visually line up

**Likely cause:** trigger arriving at the cameras with different latencies,
OR `BASLER_LEAD_US` in the Teensy firmware doesn't match the field setup.

The current production value is `BASLER_LEAD_US = 1100` (µs), tuned on the
bench. In the field, if pose differences make sync visibly bad, this can
be re-tuned. See [B. Operations & Tuning §5 (Sync tuning)](B_operations.md).

### 5.2 ApproximateTimeSync is pairing wrong-tick frames downstream

**Likely cause:** Basler stamps lag ZED stamps by ~40 ms (in DIV=3, 5 Hz
mode). Without offset compensation, an `ApproximateTimeSync` filter pairs
a Basler frame with the *next* ZED tick instead of the matching one.

**Fix:** the `harvest_moon/scripts/ros_sync_test.py` reference implementation
shifts Basler stamps by `-0.040 s` via a `SimpleFilter` relay before
synchronization. Apply equivalent offset compensation in the consumer.

### 5.3 ZED frames captured ambient-only, no strobe visible

**Symptom:** ZED frames don't show the strobe flash; they look like
ambient-light captures. Basler frames do show the strobe.

**This is by design.** ZED runs at 15 Hz, Basler+strobe at 5 Hz, so 2 of
every 3 ZED frames are between strobe pulses. The ZED is configured for
short fixed exposure (`exposure_time: 200 µs` in `zed_x_overrides.yaml`)
to make the strobe-overlapping frame visibly brighter than the ambient
ones — but most ZED frames are still ambient-only.

If ALL ZED frames are ambient-looking, including the ones that should
overlap with the strobe, then `BASLER_LEAD_US` may need re-tuning (the
strobe is firing outside the ZED exposure window). See
[B. Operations & Tuning §5 (Sync tuning)](B_operations.md).

---

## §6. Recording issues

### 6.1 `record_dataset.py` warns about missing topics

The pre-flight check found one or more topics not currently publishing.
Common causes:

- ZED hasn't finished initializing yet → wait 10 s after `moveit_pro run`
  startup, retry.
- ZED daemon crashed → §2.
- Topic name in `DEFAULT_TOPICS` doesn't match what the wrapper publishes
  (most likely the ZED ones — `image_rect_color` vs `image_raw` vs
  whatever your wrapper config emits).

To list what's actually publishing:
```
ros2 topic list | grep -E '(basler|zed)'
```

If the topic names need updating, edit `DEFAULT_TOPICS` in
`~/user_ws/src/harvest_moon/scripts/record_dataset.py`.

### 6.2 Disk fills up mid-recording

At default settings the recorder writes ~400 MB/s (≈1.4 TB/hour) to
`/mnt/ssd/datasets/` (the external 4 TB SSD). If the session disk fills:

- Stop recording (`Ctrl+C`).
- Move or delete prior bags from `/mnt/ssd/datasets/`.
- For long takes, use `--compress` (zstd), drop ZED rect_color from
  `--topics`, or run timed captures with `--duration N`.

### 6.3 Recording stopped but bag size in `session.json` is 0

The `ros2 bag record` subprocess crashed or never started. Common causes:

- ROS isn't sourced. Confirm `echo $ROS_DISTRO` is non-empty in the shell.
- Disk was already full when the recorder started.
- `bag/` directory existed already (very rare with the timestamped paths).

Re-run the recorder; if it still produces empty bags, run `ros2 bag record
-o /tmp/test /tf` manually to confirm `ros2 bag` itself works.

---

## §7. Last resorts

### 7.1 Full reset

When nothing in the targeted sections fixes the problem:

1. `Ctrl+C` `moveit_pro run`.
2. Power-cycle the 24 V rail (off, count 5, on).
3. Wait for switch + cameras to come back up (~10 s).
4. `sudo systemctl restart zed_x_daemon` on the Jetson host.
5. `moveit_pro run`.

This recovers from most "everything is in a weird state" situations
without needing a full Jetson reboot.

### 7.2 Full reboot

If §7.1 doesn't help: `sudo reboot` the Jetson, then power-cycle the 24 V
rail before bringing things back up.

### 7.3 Escalate

If after §7.1 and §7.2 the system still won't behave, document the
symptom + the steps you've tried + relevant log snippets, and escalate to
the rebuilder track. See [J. Troubleshooting Deep Dives](J_deep_troubleshooting.md)
for root-cause investigation patterns.
