# NOTICE

## Robot description meshes

The STL files under `description/assets/` are vendored from two unrelated third
parties, both under the Apache License 2.0: the SO-101 arm and camera-mount
meshes from [`danwahl/vla-test`](https://github.com/danwahl/vla-test), and the
soft fin-ray gripper meshes from
[`Vector-Wangel/XLeRobot`](https://github.com/Vector-Wangel/XLeRobot). Their
provenance, upstream commits and the list of files are recorded in
[`description/assets/NOTICE.md`](description/assets/NOTICE.md);
`description/assets/LICENSE` is the license text as carried by `vla-test`.

## MoveIt-layer configuration

The SRDF, joint limits, IK, jog and `ros2_control` YAML in `config/` are derived
from the SO-101 configuration in the `moveit_pro_example_ws` fork PR at
[`noah-wardlow/moveit_pro_example_ws`](https://github.com/noah-wardlow/moveit_pro_example_ws)
(branch `feat/so101-vla-workflows`), which declares Apache License 2.0. Joint
and link names were changed from that PR's Onshape-derived names for a
parallel-gripper build to the stock LeRobot names used by this arm, and the
controllers it configured for VLA policy execution were removed.

Everything else in this package is licensed under `LICENSE` (BSD-3-Clause).
