# Model provenance and modifications

Arm URDF and all `assets/{shared,visual}/*.STL` are derived from
[Seeed Studio's reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc/Rebot_Arm_description/RS)
at commit `58f9e433deec2ba118c5c22d60c07b9897e92acc`.
Upstream URDF: `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf`.
Mesh bytes are unchanged; `assets/` preserves upstream's `visual/` and `shared/`
subdirectories. Unused MuJoCo-only collision meshes are omitted.

Hardware design copyright © 2026 Seeed Studio Co., Ltd.
The model and derived `rebot.urdf` are distributed under **CERN-OHL-W-2.0**;
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

Original URDF export notice retained in `rebot.urdf`: SolidWorks to URDF
Exporter, Stephen Brawner (brawner@gmail.com), commit `1.6.0-4-g7f85cfe`,
build `1.6.7995.38578`; http://wiki.ros.org/sw_urdf_exporter.

The independent MoveIt Pro configuration, Objectives and tests use the package's
BSD-3-Clause license. There is no Seeed driver or SDK code in this package.
