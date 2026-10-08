# dual_arm_sim

A MoveIt Pro base configuration for the Franka arm.
For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/).

It is based on `fr3_duo_mock` from `src/external_dependencies/moveit_pro_franka_ws`, which supplies the SRDF, kinematics, joint limits, Behavior loaders and gripper Objectives.
This package adds the MuJoCo scene, its own controllers and jog settings, waypoints and Objectives.
