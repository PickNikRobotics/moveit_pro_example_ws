# reBot bench behaviors

The base configuration loads `RebotBenchBehaviorsLoader`. The same behaviors and
Objectives are inherited by the MuJoCo and hardware configurations.

`FitCubeToMask` fits an upright 50 mm cube resting on a horizontal surface at
`support_height` (default 0, the world mat plane; 0.05 for a cube stacked on
another) to SAM3 masks and RGB CameraInfo. It transforms rays using the camera optical
frame at the image timestamp. No depth image, colour threshold, or simulator
state enters the estimate. It fits the projected full cube silhouette, including
visible sides, rather than treating the mask centroid as the top-face centre.
The result is the top centre, 0.05 m above the support, with a square-symmetric
yaw chosen near the robot base heading, suitable for the `Ry(pi/2)` top-down tool
convention. A cube on a different support than the one given often fails the
2-pixel fit, but not always: at the bench scene camera's resolution a lone cube
on the mat also fits a cube on a 50 mm support, about 4 cm nearer the camera, at
under 1.5 px. The fit alone does not prove a stack; `IsCubeStacked`, with
depth, does that part. With `top_face_fallback`, when no mask fits a whole cube, the masks
are tried again as the cube's top face alone: against the blue mat, SAM3 can segment
a stacked blue cube's top face without its darker sides. While a whole cube fits,
top faces are not tried, since a side face or a printed mark can fit one too. Against a
like-coloured mat SAM3 can also split one cube into face masks, so each connected union
of the masks is fitted as well as each mask. A lone camera-facing side face fits a top
face about a cube width nearer the camera, less well than a real top face, so a top face
alone must fit within `top_face_max_rms_px` (default 1.0 px; a whole cube within 2 px).
**Locate reBot Cube** passes `top_face_max_rms_px` = 1.0 and retries a refused fit
on up to three fresh frames. The top face fixes the grasp. `top_face_fallback`
is off by default, so the stack keeps rejecting masks of partly hidden cubes.

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

`ClampRobotStateToBounds` clamps each joint of a robot state into the robot
model's position bounds, and fails for a joint the model does not know. A joint
held in impedance mode can sag past a bound: `PlanToJointGoal` then plans from
past it, and refuses the start once the sag reaches self-collision (joint3 at
about −0.08 rad). `PlanToJointGoalFromBoundedStart` is `PlanToJointGoal` with
the planning scene's current state clamped the same way, so it plans from the
bound with every port, collision check and detour unchanged. **reBot Move to Arm
Waypoint** plans with it, and **Return reBot Leader Home** plans both of its legs
with it and anchors them to the clamped measured arm.

`RecordRebotTrajectory` appends each trajectory, with its dispatch time, to an
audit file (`/tmp/rebot-trajectory-audit.yaml` by default) before execution,
and fails the leg if it cannot write it. With the controller's state, it lets a
canceled or interrupted leg's held command be reconstructed afterwards.

`LoadMotionScales` reads one leg kind's `velocity_scale_factor` and
`acceleration_scale_factor` from a YAML file in a ROS package's share directory
(`rebot_bench_base_config/objectives/rebot_stack_motion.yaml` for the stack
Objectives). It fails on a missing file, entry or factor, or a factor outside
(0, 1], so a bad settings file stops a run before any motion.

`DrawPlaceSpot` draws a uniformly random point inside an x/y rectangle, at least
a clearance from another pose, at `z=0` in that pose's frame and with its
orientation. Each tick draws anew; the unstack Objective retries it together with
IK checks of the resulting place and pick poses to reject unreachable spots.

`IsJointAtLeast` succeeds only when a joint's measured position, from a
`sensor_msgs/JointState` such as `GetJointState`'s output, is at or above a
minimum. The stack Objectives use it after opening the jaw, because a canceled
gripper move can still report success.

`IsPlannedJointAtMost` refuses a planned joint position above `maximum` in a
`moveit_msgs/RobotState`, with the configured reason. The pre-motion reach
checks use it for joint2's 2.2 rad bound; change that bound in all reach helpers
together. It is a geometric planning check, not a motor-temperature sensor.

`IsCubeStacked` succeeds only when a located cube's top centre lies within
`max_offset` horizontally of the located top of the cube it should rest on, and
the height of `cube_points`, the depth points inside the cube's mask, at
`height_quantile` (0.75 by default, above the median so mask-edge pixels on the
mat behind the cube do not pull a stacked cube down) is at least `min_height`. All three inputs must be in one frame, with z=0 at the
mat. The episode cycle runs it after each stack, against red as located before
the stack, so a blue that slid off onto the mat is not saved as a stack. Colour
alone cannot tell a blue resting on red from a blue on the mat just behind it,
as the camera sees it; the depth height can. Its test runs `FitCubeToMask` with
the bench scene camera's pose and intrinsics, with depth ray-cast from the mask.
`FitCubeToMask` outputs the mask it fitted as `cube_masks` for that check.

`ReportUnreachableCube` always fails, with a message naming the cube and where
it sits, such as `blue cube at x=0.277 y=0.253 is outside the arm's reach`. The
stack Objectives run it when a reach check fails, before any motion.

Build and run the behavior tests in a MoveIt Pro development environment:

```sh
colcon build --packages-select rebot_bench_behaviors
colcon test --packages-select rebot_bench_behaviors
colcon test-result --verbose
```
