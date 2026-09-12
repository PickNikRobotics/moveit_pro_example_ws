# SO-101 arm and camera-mount meshes

Source: [`danwahl/vla-test`](https://github.com/danwahl/vla-test/tree/bbbe60b0838c8f796942e857e437b81c2fba7bb2/sim/src/sim/description/assets)

Upstream commit: `bbbe60b0838c8f796942e857e437b81c2fba7bb2`

License: Apache License 2.0; see `LICENSE` in this directory, which is the
repository-level license of the source repository.

`vla-test` in turn vendors these from the upstream SO-ARM100 / LeRobot
description, which the mesh filenames preserve (`base_so101_v2.stl`,
`sts3215_03a_v1.stl`, and so on).

The following 18 STL files are byte-for-byte unmodified upstream exports,
verified by SHA-256 against the source tree at the commit above (the remaining
STLs in this directory come from XLeRobot, see below):

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
those parts are omitted. Both links now carry the XLeRobot fin jaws below, each
with its own decimated `*_collision.stl`; only the fin finger reuses its visual
mesh for collision.

# XLeRobot soft fin-ray gripper meshes

Source: [`Vector-Wangel/XLeRobot`](https://github.com/Vector-Wangel/XLeRobot)

Upstream commit: `51ca0ec31bdb48713b94bacdba828bf8d889296b`

License: Apache License 2.0, declared by `LICENSE` at the root of the upstream
repository. That file is the stock Apache-2.0 text with the copyright appendix
left as unfilled boilerplate (`Copyright [yyyy] [name of copyright owner]`), and
the upstream repository ships no `NOTICE` file, so there is nothing further to
reproduce here. The `LICENSE` in this directory is a separate copy carrying the
`vla-test` copyright line and covers only the SO-101 meshes above.

Upstream file: `hardware/SO101_soft_fin.stl`. That single binary STL holds three
disconnected bodies laid out on a print plate, which split into the parts below.
The fin-ray finger also ships as `hardware/step/soft_gripper_finger.step` and as
four copies on plate 3 of `hardware/XLeRobot_0_3_0.3mf`; the copy here comes from
the STL.

| File | Upstream body | Replaces |
|---|---|---|
| `soft_fin_fixed_jaw.stl` | 52 x 48.6 x 65.2 mm body | `wrist_roll_follower_so101_v1.stl` |
| `soft_fin_moving_jaw.stl` | 36.7 x 20.3 x 48 mm body | `moving_jaw_so101_v1.stl` |
| `soft_fin_finger.stl` | 67.2 x 24.2 x 24.1 mm body | the moulded fingers on both stock parts |

`soft_fin_finger.stl` is used twice, once per jaw. The two `*_collision.stl`
files are decimated copies of the corresponding visual mesh.

## These are not byte-identical exports

Unlike the SO-101 meshes above, these were transformed on the way in:

- scaled from millimetres to metres;
- the two jaw bodies were rigidly registered onto the stock parts they replace,
  so each is stored in that stock part's mesh frame and `so101.urdf.xacro`
  reuses the visual origin the stock mesh already used. The registration is the
  rigid transform aligning the servo bracket and the pivot yoke, which the fin
  parts carry unchanged: 83% of the fin fixed-jaw surface and 64% of the fin
  moving-jaw surface land within 0.1 mm of the stock part, the remainder being
  the finger fork that replaces the moulded finger;
- `soft_fin_finger.stl` is stored in a frame built from its own two screw bores:
  origin at the bore midpoint, x along the bore pair, y along the bore axes;
- decimated by vertex clustering to keep each file under 1 MB. Peak surface
  deviation from the upstream geometry is 0.08 mm for the visual meshes and
  0.6 mm for the collision meshes.

The upstream STL is unmodified in every other respect; no geometry was added,
removed or re-cut.
