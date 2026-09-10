# NOTICE

## Robot description meshes

The SO-101 arm and camera-mount STL files under `description/assets/` are
vendored from a third party under the Apache License 2.0. Their provenance,
upstream commit and the list of files are recorded in
[`description/assets/NOTICE.md`](description/assets/NOTICE.md), and the license
text is `description/assets/LICENSE`.

## MoveIt-layer configuration

The SRDF, joint limits, IK, jog and `ros2_control` YAML in `config/` are derived
from the SO-101 configuration in the `moveit_pro_example_ws` fork PR at
[`noah-wardlow/moveit_pro_example_ws`](https://github.com/noah-wardlow/moveit_pro_example_ws)
(branch `feat/so101-vla-workflows`), which declares Apache License 2.0. Joint
and link names were changed from that PR's Onshape-derived names for a
parallel-gripper build to the stock LeRobot names used by this arm, and the
controllers it configured for VLA policy execution were removed.

Everything else in this package is licensed under `LICENSE` (BSD-3-Clause).
