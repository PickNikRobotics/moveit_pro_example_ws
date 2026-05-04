# H. Configuration Reference

Every config file in the system, with each setting annotated. Use this
when you want to understand *why* a value is what it is, before changing
it. For *how* to change values safely, see
[B. Operations & Tuning Playbook](B_operations.md).

---

## §1. Teensy firmware — `~/teensy_trigger/src/main.cpp`

The Teensy is the timing master. Constants live at the top of `main.cpp`.

```cpp
const int PIN_ZED     = 2;
const int PIN_BASLER  = 12;
const int PIN_LED     = 13;
```
Output GPIO assignments. Pin 2 → ZED Link J17 Pin 16. Pin 12 → NPN base
via 560 Ω. Pin 13 → onboard LED (no external wire). Don't change unless
you also rewire — these are baked into the build guide.

```cpp
const unsigned long  ZED_PERIOD_US  = 66667;
```
ZED trigger period in microseconds. **15 Hz** (1 / 0.066667 s). Don't
change without confirming the ZED daemon is configured for the new rate
in `zed_x_overrides.yaml`'s `general.grab_frame_rate`.

```cpp
const int            BASLER_DIV     = 3;
```
Basler trigger fires every `BASLER_DIV`-th ZED tick. Production = 3 →
**5 Hz** Basler. Increase to slow Basler down further; decrease to speed
up (with bandwidth caveats). Must match `frame_rate` in
`basler_cam_*.yaml`. See [B. Operations & Tuning §2](B_operations.md).

```cpp
const unsigned long  PULSE_WIDTH_US = 16667;
```
Pulse HIGH duration on both Basler and ZED trigger pins. 16.67 ms ≈ one
ZED period. Long enough that any opto-input setup time is irrelevant;
short enough to fit inside the period. Rarely changed.

```cpp
const unsigned long  BASLER_LEAD_US = 1100;
```
Microseconds the Basler trigger fires *before* the ZED trigger. Tuned
empirically (the Basler image-data path lags the ZED's by ~1.1 ms).
Adjust to align mid-exposures across cameras when the path changes
(different cabling, different cameras, different Jetson). See
[B. Operations & Tuning §4](B_operations.md).

```cpp
enum Mode { IDLE, RUNNING };
Mode mode = RUNNING;
```
Boot mode. Production = `RUNNING` (autonomous trigger as soon as the
Teensy is powered). Set to `IDLE` if you want manual control via serial
commands at boot.

**Serial commands** (115200 baud, `/dev/ttyACM0`):

| Cmd | Action                                           |
|-----|--------------------------------------------------|
| `t` | Single-step: fire one trigger pair               |
| `r` | Run: enter `RUNNING` mode                        |
| `s` | Stop: enter `IDLE` mode                          |
| `c` | Print counters (zed, basler, uptime)             |
| `z` | Zap counters (reset to 0)                        |
| `h` | Help                                             |

---

## §2. Basler camera ROS config — `harvest_moon/config/basler_cam_*.yaml`

One file per camera. They're functionally identical except for the
`camera_frame` and `device_user_id` (= camera serial).

```yaml
camera_frame: basler_cam_1
device_user_id: "25233972"
```
TF frame name and the camera identifier the wrapper uses to find this
specific Basler. `device_user_id` matches the camera's serial number by
convention (set during provisioning). If you swap cameras, update both
fields.

```yaml
frame_rate: 5.0
```
The pylon ROS wrapper's grab-loop rate. **Must match the Teensy trigger
rate** (`15 / BASLER_DIV` Hz). The wrapper polls for frames at this
rate; if it's higher than the trigger rate, frames go missing in the
poll loop; if lower, the wrapper falls behind and publishes stale
content. The name is misleading — it's not a free-run frame rate
because the camera is externally triggered.

```yaml
image_encoding: "bayer_rggb8"
```
Raw Bayer mosaic, 8-bit, RGGB pattern. Demosaic happens downstream in
`image_proc` if needed. Producing raw mosaic at the topic level keeps
on-camera processing minimal and lets each consumer pick its own
demosaic algorithm.

```yaml
grab_strategy: 1
```
**`1 = LatestImageOnly`. Critical fix.** The pylon wrapper's default is
`0 = OneByOne` (FIFO). On startup, the wrapper accumulates a buffer of
frames and then publishes them with fresh timestamps but seconds-stale
content. Setting `grab_strategy: 1` makes the wrapper drop stale frames
and only publish the latest. Do not change to `0` or `2`.

```yaml
exposure: 500.0
```
Exposure time in microseconds. Sized to overlap the LHI-DO strobe pulse
length (~500 µs). Constraints: must be ≤ trigger period, and short
enough that ambient doesn't overwhelm the strobe. See
[B. Operations & Tuning §1.1](B_operations.md).

```yaml
exposure_auto: false
gain_auto: false
```
**Both must stay `false`.** Auto-exposure breaks strobe sync (the
wrapper hunts up to longer exposures during dim ambient, blowing the
fixed-pulse-width assumption).

> **What's NOT in this YAML:** trigger config (`TriggerMode`,
> `TriggerSource`, `LineSelector`, `LineDebouncerTimeAbs`, etc.) lives
> in the camera's UserSet1 (see [G. Software Setup §4](G_software_setup.md)).
> The YAML applies on top of UserSet1 at every launch. Trigger config
> is in the camera, not in this file.

---

## §3. ZED ROS overrides — `harvest_moon/config/zed_x_overrides.yaml`

Selective overrides applied on top of the `zed_wrapper` defaults at
launch time. Only settings we actually need to change live here.

```yaml
general:
    grab_frame_rate: 15
```
Match the Teensy's `ZED_PERIOD_US` (15 Hz). The `zed_wrapper` polls the
ZED daemon at this rate.

```yaml
video:
    auto_exposure_gain: false
    exposure_time: 200
```
**Auto exposure must stay `false`.** Same reasoning as Basler: auto
breaks the strobe-overlap design.

`exposure_time` is in microseconds. Production = 200 µs — short enough
that the strobe-overlapping ZED frame is visibly brighter than the
ambient-only frames between strobe pulses. See
[B. Operations & Tuning §1.3](B_operations.md).

```yaml
depth:
    depth_mode: 'ULTRA'
```
Production = `ULTRA` (highest-fidelity classical stereo, no TensorRT
needed). Other valid options: `'PERFORMANCE'` (fastest, lowest
accuracy), `'QUALITY'` (middle ground), `'NONE'` (depth disabled).

**Do NOT use `'NEURAL'`, `'NEURAL_LIGHT'`, or `'NEURAL_PLUS'`.** These
modes require TensorRT models the moveit_pro container's ZED SDK
install does not include — the ZED node will fail to start with
TensorRT errors.

When depth is enabled, the wrapper also publishes
`/zed_x/zed_node/point_cloud/cloud_registered`,
`/zed_x/zed_node/confidence/confidence_map`, and
`/zed_x/zed_node/disparity/disparity_image`. At native HD1200, these
plus the rect_color streams contribute ~440 MB/s of bag-write rate;
see [B. Operations & Tuning §6](B_operations.md) for managing storage,
and [§1.2](B_operations.md) for the resolution/rate trade-off.

```yaml
object_detection:
    od_enabled: false
body_tracking:
    bt_enabled: false
```
Same reason: depth-dependent modules. Keep disabled in this container.

---

## §4. Launch files

### 4.1 `harvest_moon/launch/cameras.launch.xml`

The production camera bring-up. Side-loaded from
`lab_sim/launch/agent_bridge.launch.xml`.

```xml
<arg name="node_name" value="pylon_ros2_camera_node" />
<arg name="camera_id" value="basler_cam_1" />
<arg name="config_file" value="$(find-pkg-share harvest_moon)/config/basler_cam_1.yaml" />
<arg name="mtu_size" value="9000" />
<arg name="startup_user_set" value="UserSet1" />
```

- `camera_id` becomes the camera-side topic namespace, and `node_name`
  becomes a sub-namespace, so topics are at
  `/basler_cam_1/pylon_ros2_camera_node/image_raw`,
  `/basler_cam_1/pylon_ros2_camera_node/camera_info`, etc. (The
  pylon ROS wrapper always inserts `node_name` into the topic prefix.)
  `camera_id` also disambiguates which Basler
  this node opens (matched against `device_user_id`).
- `mtu_size: 9000` enables jumbo frames between cameras, switch, and
  Jetson NIC. Not strictly needed at 5 Hz dual-Basler (the path fits
  comfortably at MTU 1500), but kept as bandwidth headroom for the
  customer's deployment. Requires `eno1` MTU 9000 on the Jetson NIC
  AND switch jumbo support — both are present on the prototype rig.
  Customer's rebuild can revert to `1500` if jumbo isn't desired.
- `startup_user_set: UserSet1` tells the wrapper to load the
  pre-provisioned UserSet1 at camera open, applying our trigger
  config before the YAML overrides kick in.

The ZED block sets:
```xml
<arg name="camera_id" value="-1" />
<arg name="ros_params_override_path"
     value="$(find-pkg-share harvest_moon)/config/zed_x_overrides.yaml" />
```
- `camera_id: -1` prevents the `camera_id` arg from leaking from the
  Basler blocks above (which would otherwise pin the ZED to the wrong
  camera index).
- `ros_params_override_path` points at our overrides YAML.

### 4.2 `lab_sim/launch/agent_bridge.launch.xml`

```xml
<launch>
  <include file="$(find-pkg-share moveit_studio_agent)/launch/studio_agent_bridge.launch.xml" />
  <include file="$(find-pkg-share harvest_moon)/launch/cameras.launch.xml" />
</launch>
```

Two includes: MoveIt Pro's standard agent bridge, plus our cameras.
This is how harvest_moon side-loads into the production runtime.
**Keep both includes** — removing the second one disables all our
camera bringup.

### 4.3 `harvest_moon/launch/debug_cameras.launch.py`

A standalone Python launch file with the same content as
`cameras.launch.xml`, used during development when bringing cameras up
*outside* MoveIt Pro:

```
ros2 launch harvest_moon debug_cameras.launch.py
```

Production uses the XML version (side-loaded into lab_sim). The Python
version exists so it's possible to launch cameras without MoveIt Pro's
full studio_agent_bridge stack — useful for ROS-side debugging without
the agent.

---

## §5. ZED daemon service — `/etc/systemd/system/zed_x_daemon.service`

Systemd unit that runs `ZEDX_Daemon`, the host-side process that talks
to the ZED Link Quad over PCIe and exposes frames to the ROS wrapper.

```
[Unit]
After=nvargus-daemon.service driver_zed_loader.service
Requires=driver_zed_loader.service

[Service]
ExecStart=/usr/sbin/ZEDX_Daemon
User=root
Restart=always

# 0 = no sync(Default), 1 = master/slave mode, 2 = slave mode only
sync_mode=2
```

Two non-obvious things:

1. The `sync_mode=2` line is at the bottom of the file, **outside any
   section**. This is unusual for a systemd unit. The line is parsed
   by `driver_zed_loader.service` (a oneshot dependency) at boot,
   which reads the value and passes it to the ZED kernel module.
   Editing this line *alone* doesn't apply — `driver_zed_loader.service`
   has to re-run, which means either rebooting or:
   ```
   sudo systemctl daemon-reload
   sudo systemctl restart driver_zed_loader.service
   sudo systemctl restart zed_x_daemon
   ```
2. Production = `sync_mode=2` (slave mode — ZED waits for triggers from
   the Teensy). The original system architecture had `sync_mode=0`
   (ZED as master); we moved off it because Basler exposures lagged the
   ZED's master pulses. See
   [D. Architecture §5.2](D_architecture.md).

The `Requires=driver_zed_loader.service` line ensures the loader runs
first and is up before the daemon starts.

---

## §6. MoveIt Pro environment — `~/moveit_pro/.env`

Read by the `moveit_pro` CLI at startup.

```
MOVEIT_CONFIG_PACKAGE=lab_sim
```
**Critical.** Determines what `moveit_pro run` launches. Set to
`lab_sim` because that package's `agent_bridge.launch.xml` is what
side-loads our cameras (see §4.2).

```
MOVEIT_HOST_USER_WORKSPACE=/home/picknik/moveit_pro/moveit_pro_example_ws
MOVEIT_HOST_USER_WORKSPACE_NAME=moveit_pro_example_ws
```
Where the workspace lives on the host, and the directory name. The
container mounts this at `~/user_ws` inside.

```
MOVEIT_LICENSE_KEY=<redact in customer copy>
MOVEIT_DOCKER_TAG=9.0.0
```
License key (PickNik issued; redact when shipping docs to a customer)
and the moveit_pro Docker image tag.

```
MOVEIT_USERNAME=picknik
MOVEIT_USER_GID=1000
MOVEIT_USER_UID=1000
```
Container user identity. The UID/GID match the host's `picknik` user so
file ownership stays consistent between host and container.

```
USE_HOST_DDS=false
```
Use the container's DDS implementation, not the host's. Avoids
ROS-discovery conflicts with anything else running on the host.

```
SHOULD_PROMPT_UPGRADE_EXAMPLE_WS=true
```
Whether the CLI prompts to upgrade the example workspace on
incompatible-version run. Cosmetic.

---

## §7. harvest_moon stub — `harvest_moon/config/config.yaml`

```yaml
objectives:
  objective_library_paths:
    example_objective:
      package_name: "harvest_moon"
      relative_path: "objectives"
```

This is a **placeholder** to satisfy MoveIt Pro's expectation that a
config package have a `config.yaml`. `harvest_moon` is not a full
MoveIt Pro robot configuration — it's a side-load package. The actual
config used at runtime is `lab_sim` (see §6).

The `objectives/` directory it references is empty.

> Don't try to run `moveit_pro run -c harvest_moon` directly — it'll
> try to load this stub as a full config and fail. Always use
> `moveit_pro run` (which reads `MOVEIT_CONFIG_PACKAGE=lab_sim` from
> `.env`).

---

## §8. Things NOT to change

A short list of "if you change this, things break in non-obvious ways":

| Setting                                            | Why not                                                    |
|----------------------------------------------------|-------------------------------------------------------------|
| `basler_cam_*.yaml` `grab_strategy: 1`             | Reverting to 0 reintroduces seconds-stale-frame buffering   |
| `basler_cam_*.yaml` `exposure_auto: false`         | Auto exposure breaks strobe sync                            |
| `zed_x_overrides.yaml` `auto_exposure_gain: false` | Same as above for ZED                                       |
| `zed_x_overrides.yaml` `depth_mode` set to `NEURAL*` | Container's ZED SDK can't run NEURAL modes (TensorRT models missing). `ULTRA`/`QUALITY`/`PERFORMANCE`/`NONE` all work.|
| Basler `ChunkModeActive` (in UserSet1)             | Wrapper uses chunk timestamp in camera-clock domain — breaks ROS time sync |
| `zed_x_daemon.service` `sync_mode=2`               | Reverting to 0 makes ZED master, breaks Teensy-as-master architecture |
| `cameras.launch.xml` `mtu_size`                    | 9000 (jumbo) is now production. Reverting to 1500 also works at 5 Hz Basler — both fit the bandwidth budget. Don't mix MTU values across launch + Jetson NIC + switch. |
| `cameras.launch.xml` ZED `camera_id: -1`           | Without this, `camera_id` from Basler args leaks to ZED     |
| `.env` `MOVEIT_CONFIG_PACKAGE=lab_sim`             | Setting to `harvest_moon` directly fails (stub config)      |
