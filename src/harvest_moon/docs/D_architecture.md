# D. System Architecture Overview

What this system is, how the pieces fit together, and why the major design
decisions were made the way they were. Read this before tackling the
[Hardware Build Guide](F_hardware_build.md) or
[Software Setup Guide](G_software_setup.md) — both reference concepts from
here.

---

## §1. What this system does

A hardware-synchronized multi-camera vision rig for robotic data
collection. Three image sensors fire on the same time grid:

- **2× Basler acA2040-35gc** — global-shutter color GigE area cameras,
  2048×1536, externally triggered.
- **1× ZED X** — stereo camera with built-in IMU, GMSL2 streamed via the
  ZED Link Quad PCIe capture card. Externally triggered (slave mode).
- **2× LHI-DO 600 mm strobes** — high-intensity LED bars, fired in sync
  with each Basler exposure to dominate over ambient light.

A **Teensy 4.0** microcontroller is the timing master. It generates two
trigger signals: one to the ZED at 15 Hz, one to both Baslers at a
divider-controllable rate (production: 5 Hz, every 3rd ZED tick). Each
Basler's exposure-active output then drives its paired strobe.

The whole rig runs alongside **MoveIt Pro 9.0.0** on a **Jetson Orin AGX
64 GB**. Camera data is published as standard ROS 2 image topics and
recorded with `ros2 bag` via a wrapper script.

---

## §2. Top-level block diagram

```
                        ┌──────────────────────────────────────┐
                        │   Jetson Orin AGX 64 GB              │
                        │   (Ubuntu, MoveIt Pro 9.0.0 docker)  │
                        │                                      │
                        │   ┌──────────────────────────────┐   │
                        │   │  ZED Link Quad (PCIe)        │   │
                        │   └──────────────────────────────┘   │
                        │   ┌──────────────────────────────┐   │
                        │   │  Ethernet eno1 (1 GbE)       │   │
                        │   └──────────────────────────────┘   │
                        │   ┌──────────────────────────────┐   │
                        │   │  USB → Teensy 4.0            │   │
                        │   └──────────────────────────────┘   │
                        └────┬─────────────┬───────────┬───────┘
                             │ GMSL2 5 m   │ Cat5e     │ USB
                             │             │           │
                  ┌──────────▼──┐  ┌───────▼────────┐  │
                  │   ZED X     │  │  TrendNet      │  │
                  │             │  │  TI-PG80B      │  │
                  │ J17 Pin 16  │  │  PoE+ switch   │  │
                  │   trigger ◄─┼──┤  (24 V DIN)    │  │
                  └─────────────┘  └─┬───────────┬──┘  │
                         ▲           │ Cat6 PoE  │     │
                         │           │           │     │
                         │  ┌────────▼─┐    ┌────▼───┐ │
                         │  │ Basler   │    │ Basler │ │
                         │  │ cam_1    │    │ cam_2  │ │
                         │  │ (25233972)    │ (25435376) │
                         │  │  Pin 2 ◄──────┐ Pin 2 ◄────┐
                         │  │  Pin 4 ──────┐│ Pin 4 ───┐ │
                         │  └──────────┘   ││ └──────┘ │ │
                         │                 ││          │ │
                         │  Yellow (Line 2 out, NPN sink) drives:
                         │                 ▼▼          ▼ │
                         │            ┌──────────┐ ┌─────────┐
                         │            │ LHI-DO 1 │ │ LHI-DO 2│
                         │            │  600 mm  │ │  600 mm │
                         │            └──────────┘ └─────────┘
                         │                 ▲           ▲
                         │  Pink (Line 1 in, opto)     │
                         │           │     │           │
                         │           │     │           │
                  ┌──────┴───────────┴─────┴───────────┴─────┐
                  │   Teensy 4.0 trigger box                  │
                  │   Pin 2  ──► ZED J17 Pin 16               │
                  │   Pin 12 ──► NPN driver ──► both Baslers' │
                  │             Pin 2 (Pink, opto in)         │
                  │   Pin 13 ──► onboard LED (status)         │
                  └───────────────────────────────────────────┘
                          (USB-powered from Jetson)


                   ┌───────────────────────────────────────────┐
                   │  24 V / 500 W DIN-rail PSU                │
                   │  feeds: switch, ZED Link board, lights,   │
                   │         (Jetson via 12 V DC-DC)           │
                   └───────────────────────────────────────────┘
```

**Legend:**
- Solid lines = electrical or data cable.
- ▶ on a wire = signal direction at that point.
- The "common GND bus" (not shown for clarity) ties together: PSU 24 V
  negative, ZED J17 Pin 2, both Baslers' Pin 5 (Gray), and both lights'
  Blue. See [F. Hardware Build Guide §grounding](F_hardware_build.md).

---

## §3. Trigger signal flow

```
            Teensy 4.0
            ┌────────────────────────────────────────┐
            │  loop @ 15 Hz                          │
            │                                        │
            │   every Nth tick (DIV=3 in production):│
            │                                        │
            │   ─┬─► Pin 12 HIGH  ──► NPN base resistor (560 Ω)
            │    │                  ──► 2N3904 collector pulled to GND
            │    │                      sinks ~6 mA through
            │    │                      both Baslers' Pin 2 (Pink) opto-LEDs in parallel
            │    │                                      │
            │    │                                      ▼
            │    │           Basler 1: opto rises ──► TriggerSource=Line1
            │    │           Basler 2: opto rises ──► TriggerSource=Line1
            │    │                                      │ (after camera exposure latency)
            │    │                                      ▼
            │    │           Each Basler exposes for `exposure_us` (default 500 µs)
            │    │                                      │
            │    │           During exposure, Line 2 (Yellow) sinks current
            │    │              ──► drives paired LHI-DO strobe NPN trigger (White wire)
            │    │              ──► strobe fires for ~500 µs
            │    │                                      │
            │    │  + BASLER_LEAD_US (1100 µs) later:  │
            │    └─► Pin 2 HIGH ──► ZED Link J17 Pin 16 (3.3 V trigger in)
            │                       ──► ZED daemon (sync_mode=2) starts capture
            │                                          │
            │                              [the lead time aligns ZED's
            │                               mid-exposure with Basler's
            │                               mid-exposure, despite the
            │                               Basler's longer pipeline lag]
            └────────────────────────────────────────┘
```

The whole sequence takes ~17 ms (the pulse width) and repeats every
66.67 ms (15 Hz).

---

## §4. Data flow

```
   Basler cam_1 ──┐
   Basler cam_2 ──┤── Cat6 PoE ──► TrendNet TI-PG80B ──► Cat5e ──► Jetson eno1
                 (1 GbE per camera, switch shares uplink to Jetson)
                                                          │
                                                          ▼
   ZED X ──── 5 m GMSL2 ──► ZED Link Quad (PCIe) ─────► Jetson kernel
                                                          │
                                                          ▼
                              ┌───────────────────────────────────────┐
                              │  moveit_pro Docker container          │
                              │  ROS 2 (Humble or compatible)         │
                              │                                       │
                              │   pylon_ros2_camera_wrapper ×2 ──► /basler_cam_1/image_raw
                              │                              ──► /basler_cam_2/image_raw
                              │                              ──► /basler_cam_*/camera_info
                              │                                       │
                              │   zed_wrapper                ──► /zed_x/zed_node/...
                              │                                       │
                              │   record_dataset.py ──► ros2 bag record ──► ~/user_ws/datasets/
                              └───────────────────────────────────────┘
```

The Jetson NIC `eno1` is the bottleneck: a single Cat5e uplink (~940 Mbps
practical) shared by both Baslers. At production rate (5 Hz dual-Basler),
this is comfortably under the limit. At 15 Hz dual-Basler the link is
saturated and frames drop on the moveit_pro path — see
[J. Deep Troubleshooting](J_deep_troubleshooting.md).

---

## §5. Design rationale

### 5.1 Why hardware sync at all

Software-triggered cameras are subject to OS scheduling jitter and ROS
publish latencies. Even on a real-time-tuned Jetson, software trigger
times can drift by tens of milliseconds — fatal for any downstream task
that does dense feature matching, optical flow, or stereo across cameras.

Hardware sync via a microcontroller's GPIO gives sub-microsecond
edge-to-edge alignment between cameras, plus a deterministic phase
relationship to a global strobe pulse. Worth the wiring complexity.

### 5.2 Why Teensy as master, not the ZED

The ZED X can act as a sync master (`sync_mode=0`), emitting trigger
pulses on J17 Pin 16 at the end of each ZED exposure. We started there.
The problem: the ZED's master pulses fire at a hardware-fixed timing
relative to its own exposure. Because the Basler's data pipeline lags the
ZED's, Basler exposures landed *after* the ZED exposure ended, and the
strobe pulse (driven by the Basler) was invisible to the ZED.

Promoting the Teensy to master and demoting the ZED to slave
(`sync_mode=2`) lets us inject a tunable lead time (`BASLER_LEAD_US`) so
that Basler and ZED mid-exposures align. Strobe pulses now overlap a
fraction of the ZED frames, which is the desired behavior.

### 5.3 Why per-camera strobe wiring

Each Basler's Line 2 (Yellow, opto OUT) drives its own LHI-DO strobe via
a direct NPN trigger. We considered a central strobe controller (Basler
SLP) but discarded it: the SLP's Light+/Light− output is built for
passive-LED loads and can't trigger the LHI-DO's integrated driver. Wiring
each light directly to its paired camera also simplifies the topology
(no shared strobe channel, no contention if cameras run at different
rates).

### 5.4 Why fixed exposure, not auto

The strobe pulse is ~500 µs long. For the Basler to capture
strobe-dominated lighting, its own exposure must overlap that pulse
window. Auto-exposure picks an exposure based on average scene
brightness, which would hunt up to longer exposures during dim ambient —
washing out the strobe contribution and breaking sync between strobe and
image.

The same logic applies to the ZED: a short fixed exposure
(`exposure_time: 200 µs`) lets the strobe-overlapping ZED frames be
visibly brighter than the ambient-only frames between strobe pulses.

### 5.5 Why 5 Hz Basler / 15 Hz ZED in production

Two cameras at 2048×1536 × 8-bit Bayer × 15 Hz × 2 cams ≈ 870–900 Mbps
on the GigE path, including GVSP overhead. That's right at the practical
ceiling of a single Cat5e uplink and exceeds the moveit_pro container's
network handling capacity (verified — host can sustain 15 Hz, container
can't, see [J. Deep Troubleshooting](J_deep_troubleshooting.md)). Running
the Basler at 1/3 the ZED rate gives us bandwidth headroom and reliable
publishing.

### 5.6 Why the moveit_pro side-load architecture

The harvest_moon package contains the camera launch + configs but is
**not** a standalone MoveIt Pro robot configuration package. The way it
gets activated: `lab_sim` is set as `MOVEIT_CONFIG_PACKAGE` in
`~/moveit_pro/.env`, and `lab_sim/launch/agent_bridge.launch.xml`
includes `harvest_moon/launch/cameras.launch.xml`. So `moveit_pro run`
boots `lab_sim`, which side-loads our cameras.

This decouples the camera setup from the (eventually) full robot
configuration, and lets us iterate on cameras without touching robot
code. The customer can replace `lab_sim` with their own
`MOVEIT_CONFIG_PACKAGE` and add the same include.

---

## §6. Component summary

| Component                 | Role                                          | Production model                       |
|---------------------------|-----------------------------------------------|----------------------------------------|
| Jetson Orin AGX 64 GB     | Compute + ROS 2 + moveit_pro container        | NVIDIA dev kit                         |
| ZED Link Quad             | GMSL2 → PCIe capture card for ZED             | StereoLabs ZED Link Quad               |
| ZED X                     | Stereo camera + IMU                           | StereoLabs ZED X (SN 47845360)         |
| Basler camera ×2          | Color area cameras, externally triggered      | acA2040-35gc (SN 25233972, 25435376)   |
| Teensy 4.0                | Timing master, generates trigger pulses       | PJRC Teensy 4.0                        |
| NPN driver                | Buffers Teensy GPIO into Basler opto LEDs     | 2N3904 + 560 Ω base resistor           |
| TrendNet PoE switch       | PoE delivery + GigE switching                 | TrendNet TI-PG80B (24 V, DIN rail)     |
| LHI-DO strobe ×2          | High-intensity strobed LED illumination       | Smart Vision Lights LHI-DO 600 mm      |
| 24 V PSU                  | System power for switch, lights, ZED Link     | 24 V / 500 W DIN-rail supply           |
| 12 V DC-DC                | Jetson + ZED Link rail (from 24 V)            | DC-DC converter, DIN rail              |

For exact part numbers, vendors, and quantities, see
[E. Bill of Materials](E_bom.md).

---

## §7. What lives where (file map)

```
~/teensy_trigger/                          (host, outside container)
    src/main.cpp                            firmware: BASLER_DIV, BASLER_LEAD_US, etc.
    platformio.ini                          PlatformIO config

~/moveit_pro/
    .env                                    MOVEIT_CONFIG_PACKAGE=lab_sim
    moveit_pro_example_ws/
        src/lab_sim/launch/agent_bridge.launch.xml      side-loads harvest_moon
        src/harvest_moon/
            launch/cameras.launch.xml       camera bring-up
            config/basler_cam_1.yaml        per-camera ROS params
            config/basler_cam_2.yaml
            config/zed_x_overrides.yaml     ZED overrides (depth_mode, exposure)
            scripts/record_dataset.py       data recording wrapper
            docs/                           you are here
            scripts/                        diagnostic scripts + provisioning + recorder
                triple_sync_test.py         standalone 3-camera sync test (no ROS)
                ros_sync_test.py            ROS-side sync test with stamp-offset relay
                provision_basler.py         one-time camera UserSet1 setup
                record_dataset.py           field data recorder (see A.5)
        vision_debug/                       captured-data output directories (legacy location)

/etc/systemd/system/zed_x_daemon.service   ZED daemon — sync_mode=2 (slave)
```

For per-file annotations, see [H. Configuration Reference](H_config_reference.md).
