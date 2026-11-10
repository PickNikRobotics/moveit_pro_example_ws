# Model provenance and modifications

Arm URDF and the original `assets/{shared,visual}/*.STL` are derived from
[Seeed Studio's reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc/Rebot_Arm_description/RS)
at commit `58f9e433deec2ba118c5c22d60c07b9897e92acc`.
Upstream URDF: `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`.
Retained mesh bytes are unchanged; `assets/` preserves upstream's `visual/` and
`shared/` subdirectories. Unused collision and replaced visual meshes are omitted.

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

Modified by PickNik Inc., 2026-10-11:

- Added xacro arguments placing `base_link` and the scene-camera mount in
  `world`; both default to coinciding with `world`.
- Added a `mujoco` hardware selector that emits no `ros2_control` block and no
  colour optical frames, for `rebot_bench_sim`'s MuJoCo description.

Modified by PickNik Inc., 2026-10-12:

- Added xacro arguments for the `d435_mount_joint` origin; they default to the
  nominal mount.
- A wrapping description that defines the `world` placement properties before
  including this file keeps them over the placement arguments.

Modified by PickNik Inc., 2026-10-14:

- Set conservative home margins of −0.02 rad for J2 and −0.01 rad for J3,
  retaining J7's zero lower bound. These are approved operating limits, not
  measured negative hard stops.

Modified by PickNik Inc., 2026-11-09:

- Moved the used description assets into this standalone simulation package.
- Removed hardware plugins, motor parameters and hardware-mode selection from
  `rebot.urdf.xacro`; it now emits geometry and frames only.
- Retained optional nominal optical frames for geometry consumers. The MuJoCo
  wrapper omits them because the simulator publishes its optical-frame sites.

Original URDF export notice retained in `rebot.urdf.xacro`: SolidWorks to URDF
Exporter, Stephen Brawner (brawner@gmail.com), commit `1.6.0-4-g7f85cfe`,
build `1.6.7995.38578`; http://wiki.ros.org/sw_urdf_exporter.

The independent MoveIt Pro configuration, Objectives and tests use the package's
BSD-3-Clause license. There is no Seeed driver or SDK code in this package.

## Printed-part visual replacements (2026-10-10)

The eight retained `assets/printed/*.dae` files are the supplied redesigned
print meshes, byte-for-byte with metre units and embedded purple/white materials.
The six arm trim parts replace Seeed's corresponding side fillers and covers.
These hardware derivatives,
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

PickNik Inc. modified the visual integration on 2026-10-10: replaced matching
trim visuals, retained internal black-plastic triangles as `pla2_internal.STL`
and `pla3_internal.STL`, and added a visual-only D435 bracket. Collision meshes,
inertia and arm joint frames remain unchanged. The bracket and Intel D435
visuals and fixed assembly frames were removed on 2026-11-10; no RealSense
mesh or driver code remains in the package.

## Gemini 305 nominal optical frame (2026-11-10)

Replaced the D435 assembly with the frame-only stock Orbbec Gemini 305
attachment from `moveit_pro_rebot_ws`'s
[`rebot_description/urdf/gemini305_wrist_camera.xacro`](https://github.com/PickNikRobotics/moveit_pro_rebot_ws/blob/main/src/rebot_description/urdf/gemini305_wrist_camera.xacro)
and hardware description defaults (Copyright 2026 PickNik Inc., BSD-3-Clause).
The nominal `gripper_end` to colour optical transform composes reBot mount CAD
with the camera's optical convention. No camera mesh, collision, inertia or
serial-specific calibration is copied. MuJoCo renders RGB at 1280x800 with
Orbbec's nominal public 68-degree vertical FOV; physical factory intrinsics
are device-specific. The scene camera's mounting transform is unchanged.

The link3 side-panel visual origins and lower-cover pose are recorded in
`assets/printed/manifest.json` and `rebot.urdf.xacro`. Mesh bytes are unchanged.
