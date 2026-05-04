# A. Field Quickstart

Power up, run, capture, shut down. Nothing else. If something doesn't work,
see [C. Field Troubleshooting Runbook](C_field_troubleshooting.md).

This assumes the rig has already been built, software is provisioned, and the
Teensy is flashed with the production firmware (`BASLER_DIV = 3`, autoboots in
`RUNNING` mode). For first-time setup of any of those, see [G. Software Setup
Guide](G_software_setup.md).

---

## 1. Power up

Order matters slightly — the cameras need PoE before the Jetson can talk to
them, and the Teensy needs USB enumeration to get its serial port.

1. **24 V supply on.** This powers the TrendNet PoE switch, both Basler
   cameras (via PoE), the ZED Link Quad, and the LHI-DO strobes. Wait
   ~10 seconds for the switch to finish negotiating PoE — front-panel LEDs
   should settle to solid green on each used port.
2. **Jetson on.** Press the Jetson's power button. Wait for the Ubuntu
   desktop to load. The Teensy (USB-connected) enumerates as
   `/dev/ttyACM0` automatically and starts firing triggers as soon as it
   has power. There is no separate Teensy power switch.

> **Field power note:** the prototype Jetson is on a bench supply. The
> ruggedized field unit may have a different power switch / sequence; if so,
> the equivalent step is "Jetson on, after the 24 V rail has settled."

---

## 2. Verify the rig is alive (optional sanity check)

Before starting `moveit_pro run`, a quick visual:

- **TrendNet switch:** `PWR1` LED solid green, `RLY` LED solid orange. The
  port going to the Jetson shows solid green. Each port going to a Basler
  shows solid green and solid orange (link + PoE).
- **Cameras:** the Basler acA2040-35gc and the ZED X have no external
  status LEDs. There's nothing visual to check on the cameras themselves —
  link state is read off the switch above.
- **Teensy:** onboard LED is blinking. With `BASLER_DIV = 3`, the LED
  pulses for ~17 ms every 200 ms (fires together with each Basler trigger
  at 5 Hz).

If any of these are wrong, jump to
[C. Field Troubleshooting Runbook §1 (Power and link checks)](C_field_troubleshooting.md).

---

## 3. Launch the system

From the Jetson desktop, open a terminal and run:

```
moveit_pro run
```

This reads `MOVEIT_CONFIG_PACKAGE=lab_sim` from `~/moveit_pro/.env` and
launches `lab_sim`, which side-loads `harvest_moon/cameras.launch.xml` and
brings up:

- Basler camera 1 (serial 25233972) → topics under `/basler_cam_1/pylon_ros2_camera_node/`
- Basler camera 2 (serial 25435376) → topics under `/basler_cam_2/pylon_ros2_camera_node/`
- ZED X (serial 47845360) → topics under `/zed_x/zed_node/`

It takes 30–60 seconds to fully come up. Watch the terminal for "Camera ready"
or equivalent messages from each wrapper. **First-launch failures are most
often the ZED daemon** — see [C. Field Troubleshooting Runbook §2 (ZED won't
start)](C_field_troubleshooting.md).

---

## 4. Confirm it's working

Open a second terminal and check publish rates. **Expected steady-state:**

| Topic                                                       | Expected rate |
|-------------------------------------------------------------|----------------|
| `/basler_cam_1/pylon_ros2_camera_node/image_raw`            | 5 Hz           |
| `/basler_cam_2/pylon_ros2_camera_node/image_raw`            | 5 Hz           |
| `/zed_x/zed_node/left/image_rect_color`                     | ~10 Hz         |
| `/zed_x/zed_node/point_cloud/cloud_registered`              | ~14 Hz         |
| `/zed_x/zed_node/imu/data`                                  | ~100 Hz        |

> Why ZED visual streams are at ~10 Hz (not 15): we run the ZED at native
> HD1200 resolution for best image quality, and the wrapper's depth
> processing can't sustain 15 Hz at full resolution on this Jetson. The
> SDK-internal grab is still 15 Hz; ROS-published image and depth streams
> drop to ~10 Hz under load. Pointcloud / odom / pose still hit ~14 Hz
> because they're produced at a different stage in the pipeline.

```
moveit_pro shell                                              # enter the running container
ros2 topic hz /basler_cam_1/pylon_ros2_camera_node/image_raw  # should print ~5 Hz, low std dev
ros2 topic hz /basler_cam_2/pylon_ros2_camera_node/image_raw  # should print ~5 Hz, low std dev
ros2 topic hz /zed_x/zed_node/left/image_rect_color           # should print ~10 Hz
```

`Ctrl+C` to exit each `topic hz`. If a topic prints nothing or wildly varying
rates, see [C. Field Troubleshooting Runbook §3 (Topic rates wrong)](C_field_troubleshooting.md).

For a visual check, launch RViz from the same shell:

```
rviz2
```

Add an `Image` display for `/basler_cam_1/pylon_ros2_camera_node/image_raw`,
then add a second `Image` display for
`/basler_cam_2/pylon_ros2_camera_node/image_raw`. Both should show live feed;
in the field with the strobe driving the lighting, frames should look
uniformly lit.

---

## 5. Record a dataset

In the `moveit_pro shell` from §4:

```
python3 ~/user_ws/src/harvest_moon/scripts/record_dataset.py --label <session_label>
```

Where `<session_label>` is a short identifier with no spaces or slashes
(e.g. `field_run_001`, `barn_morning_take2`). The script:

- Pre-flight checks that all expected topics are publishing (warns if not).
- Creates `/mnt/ssd/datasets/<label>_<timestamp>/` with a `bag/` rosbag2
  directory and a `session.json` metadata file.
- Prints periodic size readouts every 5 seconds.
- Stops cleanly on `Ctrl+C`.

Useful flags:

- `--duration N` — auto-stop after N seconds (good for fixed-length captures).
- `--compress` — zstd compression (smaller bag, more CPU; useful for long
  takes if disk is tight).
- `--outdir PATH` — override the default output dir (e.g. fall back to the
  internal NVMe scratch at `/nvme/datasets/`).
- `--topics ...` — override the default topic list (see script `--help`).

`/mnt/ssd/datasets/` is the external 4 TB SanDisk Extreme Pro V2 SSD
mounted via `/etc/fstab` and bind-mounted into the moveit_pro container
via `docker-compose.yaml`. Sustained write speed measured at 750 MB/s,
~3.4 TB usable. At default recording rate (~400 MB/s) that's roughly
2.4 hours of continuous recording before fill.

> **The external SSD is the production storage.** Already set up on
> the prototype (mounted at `/mnt/ssd` via `/etc/fstab`, bind-mounted
> into the container via `docker-compose.yaml`, and the recorder's
> `DEFAULT_OUTDIR` points there). For the customer's rebuild on a
> different system, see [G. Software Setup §8.1](G_software_setup.md)
> for the same steps applied to their SSD.

> **Storage note:** at default settings (full ZED depth + pointcloud at
> native HD1200), expect ~470 MB/s write rate. A 10-minute take is
> ~280 GB; a 30-minute take is ~840 GB. If disk fills up, see
> [B. Operations & Tuning Playbook §6 (Managing storage)](B_operations.md).

---

## 6. Shut down

1. Stop recording: `Ctrl+C` in the recorder shell.
2. Stop `moveit_pro`: `Ctrl+C` in the launch shell, then wait for clean exit.
3. Power down the Jetson: `sudo poweroff` (or normal shutdown via desktop).
4. After the Jetson fully powers off, switch off 24 V supply.

The Teensy will stop firing triggers as soon as the Jetson removes USB power,
so cameras and strobes will idle. Powering off 24 V cuts strobes and PoE.

---

## What this guide does NOT cover

- Reconfiguring exposure, frame rate, or focus → see
  [B. Operations & Tuning Playbook](B_operations.md).
- Anything unexpected → see
  [C. Field Troubleshooting Runbook](C_field_troubleshooting.md).
- Recreating the system from scratch → see [D–I, the Rebuilder track](README.md).
