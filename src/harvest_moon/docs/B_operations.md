# B. Operations & Tuning Playbook

Recipes for the most common in-field reconfigurations: exposure, focus,
frame rate, sync timing, strobe, and storage management. Each recipe lists
the files involved, the change to make, and the rebuild + relaunch step
needed for it to take effect.

For the bare-minimum "power up, run, capture" flow, see
[A. Field Quickstart](A_field_quickstart.md). For things going wrong, see
[C. Field Troubleshooting Runbook](C_field_troubleshooting.md). For the
*why* behind any of these settings, see
[H. Configuration Reference](H_config_reference.md).

---

## §0. The reconfigure-and-rebuild flow

Most settings live in YAML files inside the `harvest_moon` package. To
change one:

1. Edit the file in `~/user_ws/src/harvest_moon/config/` (the source tree —
   inside the moveit_pro shell, this maps to the host's
   `~/moveit_pro/moveit_pro_example_ws/src/harvest_moon/config/`).
2. Rebuild the package from `~/user_ws/`:
   ```
   cd ~/user_ws
   colcon build --packages-select harvest_moon
   source install/setup.bash       # only needed once per shell
   ```
3. Stop and restart `moveit_pro run`.

For the **Teensy firmware** (frame rate, sync lead) the flow is different
— edit on the host, reflash, no colcon involvement:

1. Edit `/home/picknik/teensy_trigger/src/main.cpp` on the Jetson host.
2. Reflash:
   ```
   cd /home/picknik/teensy_trigger
   pio run -t upload
   ```
3. Re-run `moveit_pro run` if it was running (the firmware runs autonomously
   once flashed, but you'll want a fresh ROS session to start with the new
   trigger pattern).

Throughout this doc, "rebuild and relaunch" refers to one of these flows.

---

## §1. Exposure tuning

Two cameras to tune separately. Both are **fixed exposure** — auto-exposure
is disabled because it interacts badly with strobe sync.

### 1.1 Basler exposure

**File:** `~/user_ws/src/harvest_moon/config/basler_cam_1.yaml` and
`basler_cam_2.yaml` — set the same value on both unless you intentionally
want them different.

```yaml
/**:
  ros__parameters:
    exposure: 500.0          # microseconds
    exposure_auto: false     # MUST stay false — auto breaks strobe sync
    gain_auto: false         # MUST stay false — auto interacts with exposure
```

**Constraints:**
- Must be ≤ trigger period (200,000 µs at 5 Hz — plenty of headroom).
- For strobe-dominated lighting, ~500 µs matches the LHI-DO strobe pulse
  length and is the production default.
- Long exposures (>2 ms) will pick up motion blur from any subject motion.
- Short exposures (<100 µs) will start losing signal even with strobe.

**When to change it:**
- Frames are too dark in ambient lighting (no strobe in scene) — bump up
  to 2000–5000 µs to capture without strobe.
- Frames are saturated — drop to 100–300 µs.

After change: rebuild `harvest_moon`, relaunch `moveit_pro run`.

### 1.2 ZED resolution vs publish rate (load-bearing trade-off)

The ZED is configured for native HD1200 (1920×1200) image publishing
via `general.pub_resolution: 'NATIVE'` in `zed_x_overrides.yaml`. This
gives the highest possible image quality, but the wrapper's depth
processing at full resolution can't sustain 15 Hz on this Jetson —
visual + depth streams drop to ~10 Hz under load. Pointcloud + odom +
pose stay near 14 Hz.

**To trade quality for rate**, three options:

1. **Drop to HD1080** (1920×1080, 4% fewer rows): set
   `general.grab_resolution: 'HD1080'` in `zed_x_overrides.yaml`. May
   restore 15 Hz, near-identical quality.
2. **Use 2× downscale**: change `pub_resolution: 'CUSTOM'` and add
   `pub_downscale_factor: 2.0`. Publishes at 960×600, comfortably 15 Hz.
3. **Lower the grab rate to match published rate**: set
   `general.grab_frame_rate: 10` and update the Teensy `BASLER_DIV`
   correspondingly. Keeps everything at the same lower rate — useful
   if downstream needs synchronized cadence across all streams.

**Production default = option 1 (NATIVE @ ~10 Hz)** because
data-collection and model-building usually prefer image quality over
rate.

### 1.3 ZED exposure

**File:** `~/user_ws/src/harvest_moon/config/zed_x_overrides.yaml`

```yaml
/**:
    ros__parameters:
        video:
            auto_exposure_gain: false      # MUST stay false
            exposure_time: 200             # microseconds
```

**Constraints:**
- Short value (<500 µs) is intentional: it lets the LHI-DO strobe pulse
  visibly dominate over ambient when a Basler trigger overlaps a ZED
  exposure.
- Going much higher (>1000 µs) washes out the strobe contribution and the
  ZED looks ambient-only on every frame.

**When to change it:**
- Bench / dark testing: 200 µs is fine.
- Outdoor bright daylight: ZED frames may still saturate even at 200 µs;
  drop to 100 µs.
- Indoor dim with strobe disabled: bump to 5000–10000 µs to see anything.

After change: rebuild `harvest_moon`, relaunch `moveit_pro run`.

---

## §2. Frame rate (`BASLER_DIV`)

The Teensy fires the ZED trigger at 15 Hz, and the Basler trigger at
`15 / BASLER_DIV` Hz. Production value is `BASLER_DIV = 3` → Basler at 5
Hz. The ZED rate is fixed at 15 Hz by `ZED_PERIOD_US = 66667` and is not
typically changed.

### 2.1 To change the Basler rate

Two files have to agree:

**Teensy firmware** — `/home/picknik/teensy_trigger/src/main.cpp` line 8:
```cpp
const int BASLER_DIV = 3;        // change this
```

**Basler ROS configs** — `basler_cam_1.yaml` and `basler_cam_2.yaml`:
```yaml
frame_rate: 5.0                  # must match 15 / BASLER_DIV
```

Common values:

| `BASLER_DIV` | Basler rate | YAML `frame_rate` |
|:------------:|:-----------:|:-----------------:|
| 1            | 15.0 Hz     | 15.0              |
| 2            | 7.5 Hz      | 7.5               |
| 3            | 5.0 Hz      | 5.0               |
| 5            | 3.0 Hz      | 3.0               |

> **`BASLER_DIV = 1` (15 Hz) is unstable on the moveit_pro container path.**
> The hardware can sustain it (verified on host), but the wrapper inside
> the container drops frames and stalls. If you need 15 Hz dual-Basler in
> ROS, see [J. Deep Troubleshooting — GigE bandwidth](J_deep_troubleshooting.md).
> For field operations, stick to `DIV = 3` or higher.

After change: reflash Teensy, rebuild `harvest_moon`, relaunch
`moveit_pro run`.

### 2.2 What `frame_rate` actually does in the YAML

The pylon ROS wrapper uses `frame_rate` as its **grab-loop rate**, not as a
free-run rate (the cameras are externally triggered). It must match the
trigger rate, otherwise the wrapper either runs ahead of the trigger
(dropping triggered frames) or behind (publishing stale ones).

### 2.3 Verifying the rate after a change

In the moveit_pro shell:
```
ros2 topic hz /basler_cam_1/pylon_ros2_camera_node/image_raw
ros2 topic hz /basler_cam_2/pylon_ros2_camera_node/image_raw
```

Both should print very close to your target frame rate with low std dev. If
not, check the Teensy with `c` in a serial monitor — `zed`/`basler` count
ratio should match `BASLER_DIV`.

---

## §3. Focus tuning (Pylon Viewer)

Focus is set by twisting the lens ring on each Basler — purely mechanical,
no software change. The fastest way to tune is with **Pylon Viewer** on
the host (outside the moveit_pro container), which gives a full-resolution
live preview.

### 3.1 Procedure

1. Stop `moveit_pro run` (Ctrl+C). The pylon ROS wrapper holds exclusive
   access to the cameras; Pylon Viewer can't open them while it's running.

2. Make sure the Teensy is still firing triggers — its LED should still be
   blinking. Pylon Viewer respects the camera's `TriggerMode = On`
   setting, so the live view will be paced by Teensy triggers.

3. On the Jetson host (NOT inside the moveit_pro shell), launch Pylon
   Viewer:
   ```
   /opt/pylon/bin/pylonviewer
   ```

4. In the Pylon Viewer device tree, double-click the camera you want to
   focus on. Click "Continuous Shot" or the play button to start live
   acquisition.

5. Twist the lens focus ring until the scene is sharp. Use the zoom
   controls to inspect detail. Aperture (the second ring) is also
   adjustable — leave it where you previously set it unless you intend to
   change depth of field.

6. Stop acquisition, close the camera in Pylon Viewer, repeat for the
   second camera.

7. Quit Pylon Viewer, then `moveit_pro run` to restart ROS with the
   freshly-focused cameras.

### 3.2 Focus check via ROS (without Pylon Viewer)

Lower-resolution but doesn't require stopping ROS. Bring up RViz from the
moveit_pro shell:
```
rviz2
```
Add an `Image` display for `/basler_cam_1/pylon_ros2_camera_node/image_raw`, twist the focus ring
on Cam 1, repeat for Cam 2. The 5 Hz ROS preview is slower to react than
Pylon Viewer's 15 Hz native preview, so this is fine for verification but
slow for active focus tuning.

---

## §4. Sync timing (`BASLER_LEAD_US`)

The Basler image-data path lags the ZED image-data path by ~1.1 ms in this
hardware setup. To align mid-exposures across the two cameras, the Teensy
fires the Basler trigger this much **before** the ZED trigger.

**File:** `/home/picknik/teensy_trigger/src/main.cpp` line 10:
```cpp
const unsigned long BASLER_LEAD_US = 1100;     // microseconds
```

The current value (1100) was tuned on the bench by sweeping until the
strobe pulse appeared in ZED frames at the expected rate. Field
adjustments are unlikely to be needed unless the hardware path changes
(different cameras, different cabling, different ZED Link board).

### 4.1 If you need to re-tune

Symptom: strobe-overlapping ZED frames are dim or strobe is invisible in
ZED at the expected cadence.

1. Lower or raise `BASLER_LEAD_US` in 200 µs steps.
2. Reflash Teensy (`pio run -t upload`).
3. Restart `moveit_pro run`.
4. Look at ZED frames — every 3rd ZED frame (at `DIV = 3`) should show a
   visible strobe contribution.
5. The optimal value maximizes the brightness of the strobe-overlapping
   frames. Sweep ±400 µs around 1100 µs to find the peak.

For systematic tuning with a script, see the standalone
`harvest_moon/scripts/triple_sync_test.py` — captures triplets and lets you
eyeball strobe visibility per frame.

---

## §5. Strobe control

The LHI-DO strobes are wired so that each Basler's Line 2 opto output
drives one strobe's NPN trigger. There is no software strobe-disable; each
strobe fires every Basler exposure as long as 24 V power is present.

### 5.1 Disabling strobes for bench testing around people

Easiest: power off the 24 V rail. This also disables the cameras (PoE) and
the switch, so it's the "system off" path, not a "lights off, cameras
keep running" path.

For "cameras keep running, lights off" the prototype rig has no clean
electrical disable. The bench workaround is to physically rotate or cover
each light. **In the field, this isn't an issue** — strobes are expected
to be on during data collection.

If a runtime strobe disable is needed in the field rebuild, the clean
way is to gate each strobe's 24 V power through a switch or relay
(left as an integrator decision).

### 5.2 Adjusting strobe intensity

The LHI-DO has an intensity input (Grey wire, 1–10 V analog). The
prototype jumpers Grey to Brown (24 V) for **maximum intensity**. To dim,
replace the jumper with a 1–10 V signal source (DAC, potentiometer
divider, or PWM-filtered output). See the LHI-DO datasheet for the
intensity-vs-voltage curve.

---

## §6. Storage management

`record_dataset.py` writes bags to `/nvme/datasets/` by default — the
NVMe scratch volume (~822 GB free on the prototype Jetson). The eMMC
root partition (15 GB free) is too small and too slow for native-HD1200
ZED streams.

> **For field deployment, the customer is most likely going to use an
> external SSD instead.** The path needs to be changed by whoever sets
> up the system on each new host: bind-mount the SSD into the moveit_pro
> container via `docker-compose.yaml` and point the recorder at the
> in-container path with `--outdir` (or by editing `DEFAULT_OUTDIR` in
> `record_dataset.py`). See [G. Software Setup §8.1](G_software_setup.md)
> for the step-by-step.

At default settings (full ZED depth + pointcloud at native HD1200 +
dual-Basler), each session writes **~470 MB/s ≈ 1.7 TB/hour**. Plan
disk and take length accordingly.

Approximate per-stream contributions, measured 2026-04-28:

| Stream                                    | Effective rate | Bandwidth   |
|-------------------------------------------|----------------|-------------|
| 2× Basler `image_raw` (5 Hz)              | 5 Hz           | ~30 MB/s    |
| 2× ZED `image_rect_color` (HD1200 RGB8)   | ~10 Hz         | ~140 MB/s   |
| ZED `depth_registered` (float32 HD1200)   | ~10 Hz         | ~90 MB/s    |
| ZED `disparity` (float32 HD1200)          | ~8 Hz          | ~75 MB/s    |
| ZED `confidence_map` (float32 HD1200)     | ~10 Hz         | ~90 MB/s    |
| ZED `point_cloud/cloud_registered`        | ~14 Hz         | ~100 MB/s   |
| ZED IMU streams + odom/pose + TF + small  | mixed          | ~5 MB/s     |
| **Total**                                 |                | **~470 MB/s** |

**Why ZED visual streams are at ~10 Hz:** the wrapper is configured for
native HD1200 (`pub_resolution: 'NATIVE'` in `zed_x_overrides.yaml`) for
maximum image quality. Depth processing at full resolution can't sustain
15 Hz on this Jetson. Pointcloud + odom + pose still hit ~14 Hz because
they're produced at a different stage of the pipeline. To trade quality
for rate, see §7 below.

### 6.0 Cache drops at shutdown (known behavior)

The recorder consistently reports a small number of in-flight messages
"lost" at shutdown — typically 1–2% of the total, concentrated on the
highest-rate topics (`/tf`, IMU streams). Example:

```
[WARN] [...] [rosbag2_cpp]: Cache buffers lost messages per topic:
    /zed_x/zed_node/imu/data: 14
    /tf: 66
    ...
Total lost: 123
```

This is **rosbag2's cache flush behavior at stop**, not a disk-speed
issue. When the recorder receives Ctrl+C or the duration timer fires,
the writer drains its in-memory cache and may lose messages still in
flight. NVMe is not the bottleneck — the same drop count happens on
both the eMMC and the NVMe. For continuous recording the cache stays in
steady state and no messages are lost mid-take.

**Mitigation if drops are unacceptable:** rosbag2 has a
`max_cache_size` parameter (default ~100 MB per topic depending on
version). It can be increased via the `--max-cache-size` flag when
invoking `ros2 bag record` directly, but `record_dataset.py` doesn't
plumb it through today — would need a small script change to expose it.

For typical model-building use, ~1% loss at the very end of a take is
not material. Document it and move on.

### 6.1 Check disk space

From the moveit_pro shell:
```
df -h /nvme/datasets/
```
Or on the host:
```
df -h /home/picknik/moveit_pro/moveit_pro_example_ws/datasets/
```

### 6.2 Clean up old sessions

```
ls /nvme/datasets/                          # see what's there
rm -rf /nvme/datasets/<session_dir>          # delete one
rm -rf /nvme/datasets/smoke_*                # delete all smoke tests
```

### 6.3 Reduce write rate for long takes

Several options, listed by impact:

1. **Drop ZED `point_cloud`** from the recorder (saves ~100 MB/s). The
   pointcloud is fully reconstructible from `depth_registered` +
   `camera_info` (with caveats — see [J. Deep Troubleshooting §2](J_deep_troubleshooting.md)
   on ZED's pointcloud filtering vs depth).
2. **Drop ZED `confidence` and `disparity`** (saves ~165 MB/s). Both
   are derivable from depth and the rectified pair.
3. **Drop ZED `rect_color`** (saves ~140 MB/s) if downstream just needs
   depth + Basler.
4. **Drop ZED resolution to 960×600** by setting
   `general.pub_resolution: 'CUSTOM'` and `pub_downscale_factor: 2.0`
   in `zed_x_overrides.yaml` (saves ~75% on every ZED image-side stream).
5. **Disable depth entirely** by setting `depth_mode: 'NONE'` in
   `zed_x_overrides.yaml` (saves ~355 MB/s). Drops the system back to
   the rect-color-only configuration. Use this for takes where only
   the Basler streams matter.
6. Use explicit `--topics` to record a custom subset, e.g.:
   ```
   python3 record_dataset.py --label slim --topics \
       /basler_cam_1/pylon_ros2_camera_node/image_raw \
       /basler_cam_1/pylon_ros2_camera_node/camera_info \
       /basler_cam_2/pylon_ros2_camera_node/image_raw \
       /basler_cam_2/pylon_ros2_camera_node/camera_info \
       /zed_x/zed_node/depth/depth_registered \
       /zed_x/zed_node/depth/camera_info \
       /tf /tf_static
   ```
7. **Compress** with zstd:
   ```
   python3 record_dataset.py --label compressed --compress
   ```
   Modest savings on raw image data (high entropy), big savings on TF
   and camera_info topics. CPU cost is moderate.
8. **Time-limit** captures and rotate manually:
   ```
   python3 record_dataset.py --label batch_001 --duration 60
   python3 record_dataset.py --label batch_002 --duration 60
   ...
   ```

### 6.4 Move bags off the Jetson

Bags live on the host filesystem under `/nvme/datasets/`. Use any normal copy tool
from a host shell:
```
rsync -avh /nvme/datasets/<session>/ user@somewhere:/path/
```

---

## §7. Disabling a camera temporarily

If one Basler is misbehaving and you want to record with just the other
one, edit `cameras.launch.xml`:

**File:** `~/user_ws/src/harvest_moon/launch/cameras.launch.xml`

Comment out the include block for the camera you don't want, e.g.:
```xml
<!--
<include file="$(find-pkg-share pylon_ros2_camera_wrapper)/launch/pylon_ros2_camera.launch.py">
  <arg name="node_name" value="pylon_ros2_camera_node" />
  <arg name="camera_id" value="basler_cam_2" />
  ...
</include>
-->
```

Rebuild `harvest_moon`, relaunch `moveit_pro run`. Note: the Teensy will
still send triggers to both cameras (they're parallel-wired), but the one
not launched in ROS just sits idle.

---

## §8. Reverting after experiments

If a tuning experiment leaves the system in a state that's worse than
where you started, the production-known-good values are:

| Setting                                             | Production value         | Where                                         |
|-----------------------------------------------------|--------------------------|-----------------------------------------------|
| Teensy `BASLER_DIV`                                 | `3`                      | `teensy_trigger/src/main.cpp:8`               |
| Teensy `BASLER_LEAD_US`                             | `1100`                   | `teensy_trigger/src/main.cpp:10`              |
| Basler `frame_rate`                                 | `5.0`                    | `basler_cam_*.yaml`                           |
| Basler `exposure`                                   | `500.0`                  | `basler_cam_*.yaml`                           |
| Basler `exposure_auto`, `gain_auto`                 | `false`                  | `basler_cam_*.yaml`                           |
| Basler `grab_strategy`                              | `1` (LatestImageOnly)    | `basler_cam_*.yaml`                           |
| Launch `mtu_size`                                   | `1500`                   | `harvest_moon/launch/cameras.launch.xml`      |
| ZED `grab_frame_rate`                               | `15`                     | `zed_x_overrides.yaml`                        |
| ZED `exposure_time`                                 | `200`                    | `zed_x_overrides.yaml`                        |
| ZED `auto_exposure_gain`                            | `false`                  | `zed_x_overrides.yaml`                        |
| ZED `depth_mode`                                    | `'NONE'`                 | `zed_x_overrides.yaml`                        |
| ZED daemon `sync_mode`                              | `2` (slave)              | `/etc/systemd/system/zed_x_daemon.service`    |
| `MOVEIT_CONFIG_PACKAGE`                             | `lab_sim`                | `~/moveit_pro/.env`                           |

Restoring any of these requires the rebuild + relaunch flow from §0.
