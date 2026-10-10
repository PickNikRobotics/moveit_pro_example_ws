# Model provenance and modifications

Arm URDF and the original `assets/{shared,visual}/*.STL` are derived from
[Seeed Studio's reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc/Rebot_Arm_description/RS)
at commit `58f9e433deec2ba118c5c22d60c07b9897e92acc`.
Upstream URDF: `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`.
Mesh bytes are unchanged; `assets/` preserves upstream's `visual/` and `shared/`
subdirectories. Unused MuJoCo-only collision meshes are omitted.

Hardware design copyright © 2026 Seeed Studio Co., Ltd.
The model and derived `rebot.urdf.xacro` are distributed under **CERN-OHL-W-2.0**;
the complete upstream license is in [LICENSE](LICENSE). Seeed's root README
identifies hardware designs under this license separately from its software.
The [upstream source location](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc)
also contains the editable mechanical designs. The modified model's source
location is this public package in `PickNikRobotics/moveit_pro_example_ws`.

Modified by PickNik Inc., 2026-10-09:

- Rewrote relative mesh paths as ROS package URIs and normalized XML formatting.
- Retained upstream link frames, joint origins, axes, inertias and collisions.
- Replaced arm position limits with the specified motor-frame limits and set
  conservative velocity limits.
- Added the geometry-free J7 motor coordinate, coupled both original finger
  sliders to it at 0.008 m/rad, and updated slider limits accordingly.
- Added a fixed world frame and uncalibrated, frame-only D435 attachment points.
- Added ros2_control using only `mock_components/GenericSystem`.

Modified by PickNik Inc., 2026-10-10:

- Added uncalibrated, frame-only OAK-D scene-camera attachment points at `world`.

Original URDF export notice retained in `rebot.urdf.xacro`: SolidWorks to URDF
Exporter, Stephen Brawner (brawner@gmail.com), commit `1.6.0-4-g7f85cfe`,
build `1.6.7995.38578`; http://wiki.ros.org/sw_urdf_exporter.

The independent MoveIt Pro configuration, Objectives and tests use the package's
BSD-3-Clause license. There is no Seeed driver or SDK code in this package.

## Printed-part visual replacements (2026-10-10)

The nine `assets/printed/*.dae` files are the supplied redesigned print meshes,
retained byte-for-byte with metre units and embedded purple/white materials.
The six arm trim parts replace Seeed's corresponding side fillers and covers;
D435_mount is the supplied wrist camera bracket. These hardware derivatives,
the STOPPER and RAIL-BASE meshes, and retained internal-plastic subsets are
licensed under CERN-OHL-W-2.0; the full license above applies. The modified
source location is this package in the public MoveIt Pro example workspace.
Brand names and logos retain their owners' trademark rights.

STOPPER and RAIL-BASE were tessellated from Seeed's `1-STOPPER-1.step` and
`1-RAIL-BASE-2.step` at revision
`171ed82cb9a4a037a6beb7d1e034352baa604b63`, under `hardware/reBot_B601_RS/3D_Printed_Parts/`.
The supplied export used 0.025 mm linear / 0.08 rad angular tolerance; DAE
coordinates are scaled from millimetres to metres. Original STEP sources are
available at that public upstream revision. DAE hashes and visual transforms
are recorded in `assets/printed/manifest.json`.

The bracket's mounting-interface reference is Seeed revision
`58f9e433deec2ba118c5c22d60c07b9897e92acc`,
`Rebot_Arm_description/Camera/urdf/d435i.urdf` and its bracket mesh. Only the
mounting transform was initially reused; the D435 body and nominal optical
frame are now included as described below.

PickNik Inc. modified the visual integration on 2026-10-10: replaced matching
trim visuals, retained internal black-plastic triangles as `pla2_internal.STL`
and `pla3_internal.STL`, and added a visual-only bracket. Collision meshes,
inertia and arm joint frames remain unchanged. The D435 frame-only placeholder
is replaced by the nominal camera assembly described below.

## D435 body and nominal frames (2026-10-10)

`assets/realsense/d435.dae` is an unchanged copy of Intel's D435 mesh from
[realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros/tree/9215f26e8348ad5922a608b77882a9bfe05940f0/realsense2_description),
commit `9215f26e8348ad5922a608b77882a9bfe05940f0`, path
`realsense2_description/meshes/d435.dae`, licensed **Apache-2.0**.
The upstream license is retained in `assets/realsense/LICENSE` with trailing
blank-line normalization. Upstream notices are retained in
`assets/realsense/UPSTREAM_NOTICE.txt` with UTF-8/LF and trailing whitespace
normalization; `source.json` records the mesh SHA-256 and
Git blob hash. The assembly offsets are from Seeed's camera URDF referenced
above; the body's mesh and nominal RGB optical offsets follow
`realsense2_description/urdf/_d435.urdf.xacro` at the same RealSense revision
(Copyright 2023 RealSense, Inc.). No driver code is copied.

The wrist mount link now uses the bracket's CAD frame. Its bottom-screw frame,
body frame and nominal color optical frame form one fixed chain. This is a
visual assembly, not a measured hand-eye calibration; no collision or inertia
is added for the camera. The scene camera remains frame-only.

The link3 side panels are exchanged using a half turn about their common
assembly Y axis. Mesh bytes and the lower cover pose are unchanged; only the
two panel visual origins differ. See the README's transform table.
