# SO-101 arm and camera-mount meshes

Source: [`danwahl/vla-test`](https://github.com/danwahl/vla-test/tree/bbbe60b0838c8f796942e857e437b81c2fba7bb2/sim/src/sim/description/assets)

Upstream commit: `bbbe60b0838c8f796942e857e437b81c2fba7bb2`

License: Apache License 2.0; see `LICENSE` in this directory, which is the
repository-level license of the source repository.

`vla-test` in turn vendors these from the upstream SO-ARM100 / LeRobot
description, which the mesh filenames preserve (`base_so101_v2.stl`,
`sts3215_03a_v1.stl`, and so on).

All 18 STL files in this directory are byte-for-byte unmodified upstream
exports, verified by SHA-256 against the source tree at the commit above:

- `arm_base.stl`
- `base_motor_holder_so101_v1.stl`
- `base_so101_v2.stl`
- `cam_mount_bottom.stl`
- `cam_mount_middle.stl`
- `cam_mount_top.stl`
- `camera_wrist_mount.stl`
- `motor_holder_so101_base_v1.stl`
- `motor_holder_so101_wrist_v1.stl`
- `moving_jaw_so101_v1.stl`
- `rotation_pitch_so101_v1.stl`
- `sts3215_03a_no_horn_v1.stl`
- `sts3215_03a_v1.stl`
- `under_arm_so101_v1.stl`
- `upper_arm_so101_v1.stl`
- `waveshare_mounting_plate_so101_v2.stl`
- `wrist_roll_follower_so101_v1.stl`
- `wrist_roll_pitch_so101_v2.stl`

## What was not taken

Upstream also ships an `assets/coacd/` directory: 72 convex parts produced by a
CoACD decomposition of `wrist_roll_follower_so101_v1.stl` and
`moving_jaw_so101_v1.stl`, for MuJoCo's contact solver. This config has no
MuJoCo model, and MoveIt's collision checker takes the visual STLs directly, so
those parts are omitted and `description/so101.urdf.xacro` uses the visual mesh
for collision on both links.
