# J. Deep Troubleshooting

Long-form root-cause investigations for problems that don't yield to the
quick fixes in [C. Field Troubleshooting Runbook](C_field_troubleshooting.md).
Two topics:

1. [GigE bandwidth bottleneck — the 15 Hz dual-Basler story](#1-gige-bandwidth-bottleneck)
2. [ApproxTimeSync stamp offset compensation](#2-approxtimesync-stamp-offset)

---

## §1. GigE bandwidth bottleneck

**Why this matters:** the production system runs both Baslers at 5 Hz
(every 3rd ZED tick). The hardware *can* sustain 15 Hz dual-cam in
isolation, but the moveit_pro container path can't. If you want 15 Hz
in the field, you need to know what we tried and where the remaining
options are.

### 1.1 Bandwidth math at 15 Hz

Per camera, full resolution:
- 2048 × 1536 × 1 byte (BayerRG8) × 15 fps ≈ **378 Mbps payload**

Two cameras:
- ~756 Mbps payload + ~15–20% GigE overhead ≈ **870–900 Mbps**

GigE practical ceiling on a single link:
- 1000 Mbps theoretical, ~940 Mbps practical with standard MTU 1500
- ~970 Mbps with MTU 9000 jumbo frames

So 15 Hz dual-Basler at full resolution sits within ~5% of the link's
practical capacity. Any imperfection in the path (cable, switch buffer,
host kernel buffer, packet retransmission overhead) tips it over.

### 1.2 What we tried (chronological)

This was extensively debugged across two sessions in late April 2026.
The full debug log lives in the project memory file
`project_gige_dual_camera_bottleneck.md`. Summary:

| Intervention                                              | Result                                                |
|-----------------------------------------------------------|--------------------------------------------------------|
| Default settings (MTU 1500, no host tuning)               | Single cam ✓; dual cam at 5 Hz ✓; dual cam at 15 Hz drops most frames |
| Cat6/6a uplink instead of Cat5e                           | No improvement                                         |
| Switch port swaps                                         | No improvement; introduced MAC flapping                 |
| 24 V power cycle                                          | Reset to clean state, but 15 Hz still broken          |
| `PylonGigEConfigurator auto-all` — full host tuning       | Initially better, eventually got worse                 |
| MTU 9000 (jumbo frames) on Jetson `eno1`                  | Required for 15 Hz to even approach success           |
| Manual sysctl: `rmem_max=32MB`, `netdev_max_backlog=5000`, `rp_filter=0` | Necessary alongside jumbo frames |
| NIC RX/TX ring buffer at 4096 (already max)               | No change needed                                       |

After all of the above, **on the host outside the container**:
- 200/200 frames at 14.94 Hz, zero BlockID drops, zero packet loss.

In the **moveit_pro container**:
- ~11–14 Hz with stalls, asymmetric between Cam 1 and Cam 2.

### 1.3 The host vs container gap

The decisive test was running the same `triple_sync_test.py` standalone
on the host (using `pypylon` directly, no ROS, no container). It worked
flawlessly at 15 Hz. The same hardware path failed when accessed via
the moveit_pro container's `pylon_ros2_camera_wrapper`.

That narrows the cause to one of:

1. **Network namespace isolation.** The container may have its own
   network namespace where the host-applied sysctls don't reach.
   Test path: run with host networking, or apply equivalent
   `sysctl -w` inside the container before the wrapper starts.

2. **cgroup CPU throttling.** Docker default scheduling can starve
   the wrapper's grab thread under burst load. The pylon wrapper is
   sensitive to grab-loop scheduling latency. Test path: increase
   the container's CPU shares or set CPU pinning.

3. **Different pylon defaults in the wrapper.** The C++ wrapper may
   not auto-tune things `pypylon InstantCamera` does at open
   (heartbeat timeout, packet retransmission, inter-packet gap).
   Test path: dump the camera's full feature state from both
   pypylon and the wrapper, diff them, look for divergence.

4. **Container scheduling priority.** The wrapper's grab thread
   doesn't get RT priority. Pylon supports `chrt`-style RT scheduling
   but the wrapper doesn't apply it. Test path: launch the wrapper
   under `chrt -f 99` with `SYS_NICE` cap added to the container.

We did not pursue any of these. Production stayed at DIV=3 / 5 Hz /
1500 MTU because that path is rock-solid and 5 Hz Basler is acceptable
for the application.

### 1.4 If you need to revisit 15 Hz

In rough order of effort and likelihood:

1. **Swap the TrendNet TI-PG80B for a managed industrial switch with
   deeper buffers.** Any business-grade ~$100 unit (Netgear GS108T,
   TP-Link TL-SG2008P, similar). Switch-side packet buffering is the
   one thing that can absorb wrapper-side hiccups without dropping
   frames. This is the single highest-impact swap.

2. **Apply RT scheduling to the pylon wrapper inside the container.**
   Requires `SYS_NICE` Linux capability + `chrt -f 99` wrapping the
   node executable. Diff the wrapper's open-time pylon feature state
   against `pypylon`'s defaults to identify any tuning gap.

3. **Run cameras with host networking** (`network_mode: host` in
   `docker-compose.yaml`) for the camera node only. Bypasses
   namespace-scoped kernel buffer limits. Has security implications
   (open ports), so isolate to the camera service.

4. **AOI / ROI crop on each camera.** Cutting the frame to 1024×768
   reduces bandwidth 4×, comfortably fits any imperfect path. Set
   `Width` and `Height` GenICam features in the camera's UserSet1.

5. **Dedicated NIC per camera.** USB3-to-Ethernet adapter for one
   camera, so each has a full 1 Gbps path to itself. Eliminates
   contention entirely.

The first option is the most leverage per dollar and per hour.

---

## §2. ApproxTimeSync stamp offset

**Why this matters:** if you're consuming Basler + ZED images
downstream and need them paired by timestamp (e.g. via
`message_filters.ApproximateTimeSynchronizer`), naive synchronization
pairs the wrong frames. You need to apply a stamp offset before the
sync filter to correct for hardware-pipeline latency differences.

### 2.1 The pipeline-latency mismatch

The Basler trigger fires `BASLER_LEAD_US = 1100` µs **before** the ZED
trigger (per the Teensy firmware). After the trigger:

- ZED's pipeline (sensor → ZED Link Quad → kernel → daemon →
  `zed_wrapper` → ROS topic) takes a small, deterministic time.
- Basler's pipeline (sensor → camera processor → GigE → switch →
  Jetson NIC → kernel → pylon SDK → ROS wrapper → ROS topic) takes
  noticeably longer — primarily the GigE transmission of a 3 MB
  payload at ~370 Mbps takes ~65 ms by itself, plus wrapper
  processing.

Net effect: although the cameras *expose* nearly simultaneously, the
**ROS-published image_raw stamps** show Basler running ~40 ms behind
ZED in production (DIV=3, 5 Hz). The Teensy lead time only compensates
for the camera-internal latency differences, not the network/wrapper
latency.

### 2.2 Why it breaks ApproxTimeSync

`message_filters.ApproximateTimeSynchronizer` pairs messages whose
stamps are closest. With ZED at 15 Hz and Basler at 5 Hz, every Basler
frame *should* pair with the ZED frame from the same trigger tick.
But because Basler stamps lag ZED stamps by ~40 ms:

- The Basler frame from trigger N arrives with stamp N+40 ms.
- The ZED frame from trigger N has stamp N.
- The next ZED frame (trigger N+1, ~67 ms later) has stamp N+67 ms.
- `(N+40) - N = 40 ms` vs `(N+67) - (N+40) = 27 ms`.

The synchronizer picks the closer pair → it **pairs Basler N with ZED
N+1**, not ZED N. Subsequent triplets line up wrong by one full ZED
tick.

### 2.3 The fix: a SimpleFilter relay

The standard pattern: subscribe to each Basler topic with a callback
that *shifts* the message stamp backward by the measured offset, then
re-emits the message via `message_filters.SimpleFilter.signalMessage()`.
The synchronizer reads from the relays, not the raw topics.

Implementation sketch (see `harvest_moon/scripts/ros_sync_test.py` for
the full working version):

```python
import message_filters
from rclpy.duration import Duration
from sensor_msgs.msg import Image

c1_filter = message_filters.SimpleFilter()
c2_filter = message_filters.SimpleFilter()

def shift_stamp(msg, dt_s):
    new_stamp = (Time.from_msg(msg.header.stamp)
                 - Duration(seconds=dt_s)).to_msg()
    msg.header.stamp = new_stamp
    return msg

def make_relay(out_filter, dt_s):
    def cb(msg):
        out_filter.signalMessage(shift_stamp(msg, dt_s))
    return cb

# Production offsets (DIV=3, 5 Hz Basler):
#   Cam 1: -0.040 s   (subtract 40 ms from Basler stamp)
#   Cam 2: -0.035 s   (slightly less for Cam 2 — measure for your rig)
self.create_subscription(Image, '/basler_cam_1/image_raw',
                         make_relay(c1_filter, -0.040), 30)
self.create_subscription(Image, '/basler_cam_2/image_raw',
                         make_relay(c2_filter, -0.035), 30)

zed_filter = message_filters.Subscriber(self, Image,
                                        '/zed_x/zed_node/left/image_rect_color')

sync = message_filters.ApproximateTimeSynchronizer(
    [c1_filter, c2_filter, zed_filter], queue_size=30, slop=0.020)
sync.registerCallback(self.synced_cb)
```

After the relay, the synchronizer pairs frames from the same trigger
tick reliably.

### 2.4 Determining the right offset for your DIV

The 40 ms / 35 ms values are specific to DIV=3 / 5 Hz / MTU 1500.
Different rates shift the offset because the Basler's transmission
time changes.

To measure for a different DIV:

1. Run `harvest_moon/scripts/ros_sync_test.py` with `--cam1-offset 0
   --cam2-offset 0` (no compensation).
2. Look at the printed signed offsets. Expect a histogram clustered
   around the true value with some jitter.
3. Update the offsets in the consumer code to the median signed value.

### 2.5 What NOT to do (Basler chunk timestamps)

The Basler camera can emit per-frame timestamps as "chunks" alongside
the image data, accessible via `ChunkTimestamp` after enabling
`ChunkModeActive`. This sounds appealing — same-clock sync from the
camera itself.

**Don't enable it via the pylon ROS wrapper.** The wrapper, when chunks
are enabled, uses the raw camera-clock value (nanoseconds since camera
boot) as the message stamp instead of the Jetson's host clock. That
breaks every cross-camera sync mechanism (the Basler stamps are no
longer in the same time domain as the ZED stamps), and breaks any
downstream filter that expects ROS time.

The provisioning script (`harvest_moon/scripts/provision_basler.py`)
explicitly sets `ChunkModeActive = false` for this reason.
