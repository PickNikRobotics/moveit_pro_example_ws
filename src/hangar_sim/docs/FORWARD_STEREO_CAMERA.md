# Forward stereo input for VO/VSLAM

<!-- markdown-link-check-disable-next-line -->
`hangar_sim` provides deterministic, base-mounted stereo input topics for downstream evaluation associated with the [MoveIt Pro product VO/VSLAM benchmark work](https://github.com/PickNikRobotics/moveit_pro/issues/20105). The link targets a private repository and returns 404 to unauthenticated link checkers.

This package provides only the simulated camera source and ROS image/calibration contract. It does **not** launch or configure a VO/VSLAM implementation, evaluation harness, or benchmark Objective. Those consumers must be integrated separately.

## Published contract

| Stream | Image | Calibration | Frame |
| --- | --- | --- | --- |
| Left | `/forward_stereo/left/image_rect` | `/forward_stereo/left/camera_info` | `forward_stereo_left_optical_frame` |
| Right | `/forward_stereo/right/image_rect` | `/forward_stereo/right/camera_info` | `forward_stereo_right_optical_frame` |

Both images in a pair have the same source/capture timestamp. Each `camera_info` preserves the corresponding image timestamp and frame. The packaged launch defaults `use_sim_time` to false, so these timestamps must not be interpreted as `/clock`-based simulation time unless the complete runtime explicitly configures that clock. The source rate is the MuJoCo `render_publish_rate`, **20 Hz**.

## OAK-D Pro parity profile

This simulation profile matches a standard OAK-D Pro's nominal stereo geometry and ROS image/calibration data contract. Configure the physical camera to a center-cropped `1280x720 @ 20 Hz` profile when collecting comparison data.

- Parent/base frame: `ridgeback_base_link`.
- Left optical center: `(0.45, 0.0375, 0.55)` m in the base frame.
- Right optical center: `(0.45, -0.0375, 0.55)` m in the base frame.
- Baseline: **0.075 m**, horizontal and measured from left to right.
- Both cameras face base `+X`; their optical frames use ROS convention (`+Z` forward, `+X` right, `+Y` down).
- Sensor profile: **1280 x 720**, a center crop of the OV9282 sensor. MuJoCo requires all rendered cameras to share its 1280 x 720 offscreen buffer.
- Field of view: **80 degrees horizontal / 50.534 degrees vertical**. The horizontal field matches the nominal OAK-D Pro stereo FOV; the vertical crop preserves the same focal length.
- Output encoding: **`mono8`**, converted from MuJoCo's synchronized RGB render.
- Rate: **20 Hz**, matching the deterministic MuJoCo render rate and an explicitly configurable physical OAK capture profile.
- Ideal pinhole focal lengths: `fx = fy = 762.722299 px`.
- Principal point: `(cx, cy) = (640, 360) px`.
- The left projection matrix has `P[0,3] = 0`. The right projection matrix has `P[0,3] = -fx * baseline = -57.204172 px*m`, following the ROS stereo convention.

MuJoCo's native source topics (`/forward_stereo_left/color` and `/forward_stereo_right/color`) are paired by exact timestamp. Unmatched frames are not published through the stereo contract. Images and their corresponding `camera_info` messages share the same source timestamp.

### Simulation boundary

This is an OAK-D Pro-compatible **nominal input contract**, not a photometric sensor emulator. MuJoCo renders ideal pinhole images, so distortion coefficients are zero, `R` is identity, and the images are already rectified. It does not reproduce per-device calibration variation, exposure/gain behavior, shot noise, motion blur, lens shading, infrared dot projection, active illumination, or DepthAI's on-device stereo-depth processing. Physical-camera validation must record the OAK's own `camera_info`, exposure conditions, dropped-frame counts, and actual measured rate; do not replace the real calibration with these nominal values.

## Inspection

After starting `hangar_sim`, inspect either image with:

```bash
ros2 run rqt_image_view rqt_image_view /forward_stereo/left/image_rect
```

Inspect individual message fields with:

```bash
ros2 topic echo /forward_stereo/left/image_rect --once --field header
ros2 topic echo /forward_stereo/left/image_rect --once --field encoding
ros2 topic echo /forward_stereo/right/camera_info --once --field p
```

Sequential `ros2 topic echo` commands do not prove synchronization because they can observe different captures. The `forward_stereo_publisher_test` target verifies exact four-message timestamp equality, dimensions, encoding, frame IDs, and calibration. For full-system validation, record all four contract topics together with `/tf`, `/tf_static`, any configured `/clock`, and the selected ground-truth pose/odometry topics. A downstream evaluation should report matched pairs, effective rate, and dropped/unmatched frames from that bag; no such evaluation harness is included here.

The publisher clears its pairing queues when its ROS clock jumps backward so messages from different clock epochs cannot be combined. This is a clock-discontinuity guard; it does not imply that a MuJoCo keyframe reset emits a clock jump.
