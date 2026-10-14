# reBot bench behaviors

The base configuration loads `RebotBenchBehaviorsLoader`. The same behaviors and
Objectives are inherited by the MuJoCo and hardware configurations.

`FitCubeToMask` fits an upright 50 mm cube resting on the world mat plane (`z=0`)
to SAM3 masks and RGB CameraInfo. It transforms rays using the camera optical
frame at the image timestamp. No depth image, colour threshold, or simulator
state enters the estimate. It fits the projected full cube silhouette, including
visible sides, rather than treating the mask centroid as the top-face centre.
The result is the top centre at `z=0.05` with a square-symmetric yaw chosen near
the robot base heading, suitable for the `Ry(pi/2)` top-down tool convention.

Masks that touch the image boundary, contain disconnected silhouettes, have an
inconsistent projected area, or exceed 2 pixels contour RMS are rejected.
Multiple masks for the same cube are ranked by fit error; distinct fitted cubes
more than 10 mm apart are ambiguous and fail. These checks detect many incomplete
masks but do not certify camera calibration or millimetre accuracy on hardware.

`HoldGripperInTrajectory` appends the preceding `joint7` command to every arm
trajectory point. This prevents a partial arm goal from replacing a loaded
jaw command with its measured value. After a gripper goal that did not finish,
pass the measured `gripper_state` instead, so the controller accepts a goal
that starts where the jaw stalled. It preserves velocity, acceleration, and
effort arrays when present and rejects malformed trajectories. It does not
compensate arm deflection or change the arm trajectory's initial measured state.

`AnchorTrajectoryToCommand` shifts a trajectory planned from the measured state
so that it starts at the controller's previous command, for a joint that settles
short of its command under load, and holds every commanded joint the
trajectory leaves out. It uses measurement instead for a missing command or one
more than 0.10 rad from measured state, logging each replacement, and outputs
the exact endpoint for the caller to commit after execution succeeds. Joints
listed in `measured_joints`, such as a jaw planned from its measured position,
keep their measured start whatever the command. The stack
Objectives anchor every leg this way, so the command stays continuous from one
leg to the next.

`RecordRebotTrajectory` appends each trajectory, with its dispatch time, to an
audit file (`/tmp/rebot-trajectory-audit.yaml` by default) before execution,
and fails the leg if it cannot write it. With the controller's state, it lets a
canceled or interrupted leg's held command be reconstructed afterwards.

`LoadMotionScales` reads one leg kind's `velocity_scale_factor` and
`acceleration_scale_factor` from a YAML file in a ROS package's share directory
(`rebot_bench_base_config/objectives/rebot_stack_motion.yaml` for the stack
Objectives). It fails on a missing file, entry or factor, or a factor outside
(0, 1], so a bad settings file stops a run before any motion.

`IsJointAtLeast` succeeds only when a joint's measured position, from a
`sensor_msgs/JointState` such as `GetJointState`'s output, is at or above a
minimum. The stack Objectives use it after opening the jaw, because a canceled
gripper move can still report success.

`ReportUnreachableCube` always fails, with a message naming the cube and where
it sits, such as `blue cube at x=0.277 y=0.253 is outside the arm's reach`. The
stack Objectives run it when a reach check fails, before any motion.

Build and run the behavior tests in a MoveIt Pro development environment:

```sh
colcon build --packages-select rebot_bench_behaviors
colcon test --packages-select rebot_bench_behaviors
colcon test-result --verbose
```
