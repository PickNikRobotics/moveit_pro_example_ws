# G. Software Setup Guide

First-time software provisioning, from a fresh Jetson Orin AGX through to
a working `moveit_pro run` with all three cameras streaming. Each step
verifies the previous one, so failures isolate cleanly.

Prerequisites:
- Hardware fully assembled per [F. Hardware Build Guide](F_hardware_build.md).
- §8.1 and §8.2 of F (continuity + power-up voltage checks) have passed.

---

## §0. Jetson base OS and JetPack

Out of scope for this guide. The Jetson Orin AGX should be running Ubuntu
22.04 with NVIDIA JetPack installed (the version that matches the ZED
SDK and MoveIt Pro 9.0.0 — see PickNik's MoveIt Pro install docs for the
specific JetPack version). Do this before continuing.

---

## §1. Flash the Teensy

The Teensy holds the trigger-timing firmware. This step is independent
from everything else — you can do it any time before the first
`moveit_pro run`.

### 1.1 Install PlatformIO on the host

PlatformIO is the build/flash tool for Teensy. On the Jetson host:

```
sudo apt update
sudo apt install python3-pip
pip3 install --user platformio
```

Add `~/.local/bin` to PATH if it isn't already. Verify:
```
pio --version
```

### 1.2 Get the firmware source

The firmware lives in `~/teensy_trigger/`. **[FIELD]** if not already
present, clone or copy it from wherever the customer's source-of-truth
lives. The directory structure:
```
~/teensy_trigger/
    platformio.ini
    src/main.cpp
```

### 1.3 Verify production firmware constants

Open `~/teensy_trigger/src/main.cpp` and confirm:

```cpp
const unsigned long  ZED_PERIOD_US  = 66667;   // 15 Hz ZED tick
const int            BASLER_DIV     = 3;       // 5 Hz Basler
const unsigned long  BASLER_LEAD_US = 1100;    // sync compensation
```

If any of these differ, see [B. Operations & Tuning §2 / §4](B_operations.md)
for what they do.

### 1.4 Flash

Connect the Teensy to the Jetson via USB. From the host:
```
cd ~/teensy_trigger
pio run -t upload
```

PlatformIO will detect the Teensy on `/dev/ttyACM0`, compile, and flash.
The Teensy reboots into the new firmware automatically and starts the
trigger loop in `RUNNING` mode.

### 1.5 Verify

The Teensy onboard LED should pulse ~17 ms every 200 ms (5 Hz). Open a
serial monitor:
```
screen /dev/ttyACM0 115200       # exit with Ctrl-A k y
```
You should see `Teensy trigger ready.` and a help line. Type `c` to
print counters; you'll see `zed`, `basler`, and `uptime_ms` increasing.
If the firmware accepts `r`, `s`, `c`, `t`, `z`, `h` over serial, it's
loaded correctly.

> If `screen` complains about permissions, add yourself to the `dialout`
> group: `sudo usermod -aG dialout $USER`, log out and back in.

After this step, [F. Hardware Build §8.3 (visual strobe test)](F_hardware_build.md)
becomes runnable. Do that before proceeding to make sure the trigger
chain works end-to-end at the hardware level.

---

## §2. Install the Basler pylon SDK

The pylon SDK provides the camera driver, `pylonviewer` for focus tuning,
and `PylonGigEConfigurator` for network tuning. The MoveIt Pro container
includes its own pylon SDK install for the ROS wrapper, but you also need
it on the host for the configurator and viewer.

### 2.1 Download and install pylon

Download the latest pylon SDK for Linux ARM64 from
https://www.baslerweb.com/en/downloads/software-downloads/. Pick the
debian package for `aarch64`.

Install:
```
sudo dpkg -i pylon_*_aarch64.deb
sudo apt -f install                 # resolve any missing deps
```

Verify:
```
ls /opt/pylon/bin/
/opt/pylon/bin/pylonviewer --help
```

### 2.2 (Optional) Add `/opt/pylon/bin` to PATH

Edit `~/.bashrc`:
```
export PATH=$PATH:/opt/pylon/bin
```
Then `source ~/.bashrc`. Lets you type `pylonviewer` instead of the full
path.

---

## §3. Configure the Jetson NIC and Basler IPs

The Baslers are GigE Vision cameras and need IPs on a dedicated subnet
shared with the Jetson NIC. Production layout:

| Device            | IP             |
|-------------------|----------------|
| Jetson `eno1`     | `192.168.5.2`  |
| Basler cam_1      | `192.168.5.3`  |
| Basler cam_2      | `192.168.5.4`  |

### 3.1 Set the Jetson NIC IP

Through NetworkManager (GUI or `nmcli`), set `eno1` to a manual IP of
`192.168.5.2/24`. No gateway. Verify:
```
ip addr show eno1
```
Should show `inet 192.168.5.2/24`.

### 3.2 Assign persistent IPs to the cameras

The pylon ROS wrapper does **not** set camera IPs at launch — it expects
each camera to already have a usable IP on the same subnet as the
Jetson NIC. The IPs are stored in each camera's non-volatile memory, so
this is a one-time step.

Easiest path is **Pylon Viewer's "Pylon IP Configurator"** dialog:

1. Launch Pylon Viewer on the host:
   ```
   /opt/pylon/bin/pylonviewer
   ```
2. From the menu: **Tools → Pylon IP Configurator** (or similar — exact
   menu wording depends on Pylon SDK version).
3. The dialog lists all cameras visible on the network. Each row shows
   serial number, MAC, current IP, and IP-config method (DHCP / LLA /
   Persistent).
4. For each camera, set:
   - **Static IP** (Persistent IP) enabled.
   - **IP address**: `192.168.5.3` for serial `25233972`, `192.168.5.4`
     for serial `25435376`.
   - **Subnet mask**: `255.255.255.0`.
   - **Gateway**: leave blank or `0.0.0.0`.
5. Apply. The camera writes the new IP to its non-volatile memory and
   returns to the configured IP after every power-cycle.

The CLI alternative is `PylonGigEConfigurator auto-ip` (interactive — it
prompts for target IP per camera). See:
```
sudo /opt/pylon/bin/PylonGigEConfigurator auto-ip --help
```

### 3.3 Verify

```
python3 -c "from pypylon import pylon; [print(f'{d.GetModelName()} | serial={d.GetSerialNumber()} | IP={d.GetIpAddress()}') for d in pylon.TlFactory.GetInstance().EnumerateDevices()]"
```

Should show both Baslers at `192.168.5.3` and `192.168.5.4`. Power-cycle
the cameras (briefly unplug PoE Cat6) and re-run the check — they
should come back at the same IPs, confirming persistence.

> **Host-side network tuning for jumbo frames** (production default):
> ```
> sudo ip link set eno1 mtu 9000
> sudo sysctl -w net.core.rmem_max=33554432 net.ipv4.conf.eno1.rp_filter=0
> ```
> These match the `mtu_size: 9000` in `cameras.launch.xml`. Jumbo frames
> aren't strictly required at 5 Hz dual-Basler (MTU 1500 also fits the
> bandwidth budget) but provide headroom and consistency with the
> deployed config. For the customer's rebuild, decide whether to keep
> jumbo (requires switch jumbo support) or revert all four settings to
> defaults — see [J. Deep Troubleshooting](J_deep_troubleshooting.md).

---

## §4. Provision Basler UserSet1

Each Basler stores its trigger / pixel-format / line config in a
"UserSet" that loads at camera boot. We provision UserSet1 once with the
production trigger config, then point the camera to load UserSet1 at
power-on. After this, the camera always boots with the right config —
no per-launch setup is needed.

### 4.1 Run the provisioning script

The MoveIt Pro container needs to be NOT running (the pylon ROS wrapper
holds the cameras exclusively). On the host, with the cameras powered
and discoverable:

```
cd ~/moveit_pro/moveit_pro_example_ws/src/harvest_moon/scripts
python3 provision_basler.py
```

The script (see `provision_basler.py` for the exact features it sets):
- Assigns `device_user_id = serial_number` for each camera (so ROS can
  find them by name).
- Configures `TriggerMode = On`, `TriggerSource = Line1`,
  `TriggerSelector = AcquisitionStart`, `TriggerActivation = RisingEdge`.
- Sets `LineDebouncerTimeAbs = 5.0` (filters spurious trigger edges; the
  default of 0 µs is unreliable).
- Sets `LineSelector = Line2`, `LineSource = ExposureActive` (so Line 2
  drives the strobe during each exposure).
- Sets `PixelFormat = BayerRG8`, `ExposureTimeAbs = 500` (µs).
- Sets `ChunkModeActive = false` (do NOT enable chunks — see
  [J. Deep Troubleshooting](J_deep_troubleshooting.md) for why).
- Saves to `UserSet1` and sets `UserSetDefaultSelector = UserSet1`.

### 4.2 Verify

Power-cycle each camera (unplug PoE Cat6 briefly, replug). When the
camera comes back, it should load UserSet1. Re-run the device enumeration
from §3.2 — `user_id` should now match the serial number for each
camera.

---

## §5. Configure the ZED daemon for slave mode

The ZED daemon (`zed_x_daemon.service`) talks to the ZED Link Quad and
provides the frames to the ROS wrapper. By default it runs in master
mode (sync_mode=0). For our system the ZED is in slave mode (sync_mode=2)
— it receives triggers from the Teensy via J17 Pin 16.

### 5.1 Edit the daemon service file

```
sudo nano /etc/systemd/system/zed_x_daemon.service
```

The relevant line is a literal `sync_mode=` directive at the bottom of
the file, *outside* the `[Service]` section. (This is unusual for a
systemd unit file — `driver_zed_loader.service` parses this file at
boot to read the value, which is why the comment `0 = no sync, 1 =
master/slave, 2 = slave only` lives next to it.)

Production state:
```
# 0 = no sync(Default), 1 = master/slave mode, 2 = slave mode only
sync_mode=2
```

Make sure this line says `sync_mode=2`. The rest of the file
(`ExecStart=/usr/sbin/ZEDX_Daemon`, `User=root`, `Requires=driver_zed_loader.service`,
etc.) should not be modified.

Save and exit.

### 5.2 Reload and apply

Just restarting `zed_x_daemon` is **not enough** — the ZED's sync mode
is applied at driver-load time by `driver_zed_loader.service`, which is
a oneshot service that runs at boot. To apply a sync_mode change without
rebooting:

```
sudo systemctl daemon-reload
sudo systemctl restart driver_zed_loader.service
sudo systemctl restart zed_x_daemon
```

### 5.3 Verify

```
sudo systemctl status zed_x_daemon
```
Should be `active (running)`. If it's `failed`, jump to
[C. Field Troubleshooting §2](C_field_troubleshooting.md).

---

## §6. Install MoveIt Pro

Out of scope for this guide. Follow PickNik's official MoveIt Pro 9.0.0
install instructions for the customer's chosen license tier and platform.
Confirm the install with:

```
moveit_pro --help
```

The CLI should respond.

---

## §7. Set up the workspace

The harvest_moon package, lab_sim package, and the diagnostic /
provisioning / recorder scripts all live in the moveit_pro example
workspace.

### 7.1 Clone or copy the workspace

```
cd ~/moveit_pro
# Clone or copy moveit_pro_example_ws/ here
```

**[FIELD]** Document the customer's source-of-truth (PickNik git, internal
git, tarball) and the exact command to retrieve it.

The workspace should contain at minimum:
```
~/moveit_pro/moveit_pro_example_ws/
    src/harvest_moon/
        config/         camera & ZED ROS configs
        launch/         cameras.launch.xml + debug_cameras.launch.py
        scripts/        provision_basler.py, record_dataset.py, sync tests
        docs/           the docs you're reading now
    src/lab_sim/        side-loads harvest_moon at moveit_pro run
    src/pylon-ros-camera/   pylon_ros2_camera_wrapper sources
    docker-compose.yaml
    colcon-defaults.yaml
```

### 7.2 Configure `.env`

`moveit_pro run` reads `~/moveit_pro/.env` at startup. The critical
setting is `MOVEIT_CONFIG_PACKAGE`, which determines what the runtime
launches.

Edit `~/moveit_pro/.env`:
```
MOVEIT_CONFIG_PACKAGE=lab_sim
MOVEIT_HOST_USER_WORKSPACE=/home/picknik/moveit_pro/moveit_pro_example_ws
MOVEIT_LICENSE_KEY=<your license key>
MOVEIT_DOCKER_TAG=9.0.0
MOVEIT_USERNAME=picknik
MOVEIT_USER_GID=1000
MOVEIT_USER_UID=1000
MOVEIT_HOST_USER_WORKSPACE_NAME=moveit_pro_example_ws
USE_HOST_DDS=false
SHOULD_PROMPT_UPGRADE_EXAMPLE_WS=true
```

Adjust paths and license key for the customer's setup.

### 7.3 Confirm the lab_sim side-load

`harvest_moon` is not a standalone MoveIt Pro config (its `config.yaml`
is a stub). It activates by being included from `lab_sim`. Verify:

```
cat ~/moveit_pro/moveit_pro_example_ws/src/lab_sim/launch/agent_bridge.launch.xml
```

The file should include both:
```xml
<include file="$(find-pkg-share moveit_studio_agent)/launch/studio_agent_bridge.launch.xml" />
<include file="$(find-pkg-share harvest_moon)/launch/cameras.launch.xml" />
```

If the second include is missing, add it manually.

---

## §8. Build the workspace

Inside the moveit_pro container shell (with `moveit_pro run` running in
another terminal):

```
moveit_pro shell
cd ~/user_ws
colcon build --packages-select harvest_moon lab_sim
source install/setup.bash
```

> If you need a build shell *without* launching the runtime,
> `docker compose run --rm dev bash` from `~/moveit_pro/moveit_pro_example_ws/`
> opens the `dev` service defined in `docker-compose.yaml`.

After the build completes, the configs are installed at
`~/user_ws/install/harvest_moon/share/harvest_moon/config/`.

### 8.1 Set up the dataset storage volume

The recorder writes bags to its `--outdir` path (default
`/mnt/ssd/datasets/`). For the container to see this path, the host
filesystem location has to be bind-mounted into all camera-relevant
services in `docker-compose.yaml`.

#### Production: external SSD at `/mnt/ssd`

The prototype uses a SanDisk Extreme Pro V2 4 TB portable SSD
connected to a USB 3.2 Gen 2 (10 Gbps) USB-C port. Sustained write
measured at 750 MB/s, ~3.4 TB usable, formatted ext4.

**One-time setup steps:**

1. Plug the SSD into a 10 Gbps USB-C port (verify with `lsusb -t` —
   look for a `Mass Storage` device on the `10000M` root hub).

2. Identify the device path with `lsblk` (typically `/dev/sda1`).

3. **Reformat as ext4** (factory default is exFAT, which is slow for
   sustained large-block writes — DESTROYS any existing data):
   ```
   sudo umount /dev/sda1 2>/dev/null
   sudo mkfs.ext4 -L FIELD_SSD /dev/sda1
   ```

4. Mount manually first to verify:
   ```
   sudo mkdir -p /mnt/ssd
   sudo mount /dev/sda1 /mnt/ssd
   sudo chown $USER:$USER /mnt/ssd
   mkdir -p /mnt/ssd/datasets
   ```

5. **Verify sustained write speed** (target ≥ 600 MB/s for 1.3×
   headroom over the ~400 MB/s recording rate):
   ```
   dd if=/dev/zero of=/mnt/ssd/test.bin bs=4M count=2048 oflag=direct conv=fdatasync
   rm /mnt/ssd/test.bin
   ```

6. **Add to `/etc/fstab`** so it auto-mounts at every boot. First get
   the UUID:
   ```
   sudo blkid /dev/sda1
   ```
   Then add a line to `/etc/fstab`:
   ```
   UUID=<uuid-here>  /mnt/ssd  ext4  defaults,noatime,nofail  0  2
   ```
   `noatime` skips access-time writes; `nofail` prevents boot failure
   if the SSD is disconnected.

7. Verify the fstab entry without rebooting:
   ```
   sudo umount /mnt/ssd
   sudo mount -a
   df -h /mnt/ssd
   ```

#### Bind-mount into the moveit_pro container

`docker-compose.yaml` (in `moveit_pro_example_ws/`) has these volume
lines under `agent_bridge`, `drivers`, and `dev`:

```yaml
- /nvme:/nvme         # internal NVMe scratch (fallback)
- /mnt/ssd:/mnt/ssd   # external production SSD
```

After editing `docker-compose.yaml`, restart `moveit_pro` so the new
mount applies (the running container needs to be re-created, not
just the agent restarted).

#### NVMe scratch fallback

The Jetson's internal NVMe (mounted at `/nvme`, 1 TB) remains
bind-mounted as a fallback for when the external SSD isn't connected
(e.g., bench tests). Use `--outdir /nvme/datasets` on the recorder to
write there instead. Keep both bind mounts in `docker-compose.yaml`
for flexibility.

> **For the customer's rebuild on a different machine**, replicate the
> SSD steps above with their drive. If their machine doesn't have a
> directory at `/nvme`, either `mkdir /nvme` on their host as a stub
> or remove the `- /nvme:/nvme` line from `docker-compose.yaml`
> (otherwise Docker will refuse to start the container).

---

## §9. First launch and verify

From the host (NOT inside the container shell):

```
moveit_pro run
```

Wait ~30 seconds for everything to come up. In a second terminal:

```
moveit_pro shell
ros2 topic hz /basler_cam_1/pylon_ros2_camera_node/image_raw   # should show ~5 Hz
ros2 topic hz /basler_cam_2/pylon_ros2_camera_node/image_raw   # should show ~5 Hz
ros2 topic hz /zed_x/zed_node/left/image_rect_color            # should show ~10 Hz
```

If everything is publishing at the expected rates, the system is set up
correctly. Move on to:
- [A. Field Quickstart](A_field_quickstart.md) for daily operation.
- [B. Operations & Tuning](B_operations.md) for adjustments.
- [C. Field Troubleshooting Runbook](C_field_troubleshooting.md) if any
  of the above doesn't behave.

---

## §10. Optional: provision data-collection helpers

The `record_dataset.py` script lives in `harvest_moon/scripts/` and
ships with the workspace. To make it directly executable:

```
chmod +x ~/moveit_pro/moveit_pro_example_ws/src/harvest_moon/scripts/record_dataset.py
```

It expects to be run from inside the `moveit_pro shell`. See
[A. Field Quickstart §5](A_field_quickstart.md) for usage.

---

## §11. Recovery / rebuild from scratch

If the install is in a known-bad state, the order to redo from scratch:

1. Power down everything.
2. Power up 24 V rail and Jetson.
3. Verify F §8.1, §8.2 still pass (no hardware changed).
4. Reflash Teensy firmware (§1).
5. Re-run F §8.3 (strobe visual test).
6. Re-run `PylonGigEConfigurator` to reset host network tuning (§3.2).
7. Re-run `provision_basler.py` to restore UserSet1 (§4).
8. Restart `zed_x_daemon` and `driver_zed_loader.service` (§5.2).
9. `colcon build` + `moveit_pro run`.

This sequence also serves as a sanity check after major changes (kernel
update, JetPack upgrade, container image swap).
