# SO-101 Objectives

The SO-101 has five arm degrees of freedom, so arbitrary six-dimensional
Cartesian poses are not generally reachable. Use position-only or
orientation-sampled workflows when adding Cartesian Objectives.

`Move Live SO101 to Waypoint` and the standard Teleoperate workflow execute
through `/joint_trajectory_controller/follow_joint_trajectory`. The Pi owns
that controller and the physical command interfaces.

`Execute SO101 VLA Policy` switches to
`joint_trajectory_admittance_controller`, executes camera-conditioned chunks,
then restores the standard trajectory controller. MoveIt Pro 10.0 requires
JTAC's `FollowJointTrajectoryWithAdmittance` contract for policy chunk
stitching; a standard JTC cannot replace it. If an Objective is cancelled
during the handoff, run `Restore SO101 Planning Controller` before another
motion workflow.

Bring up a policy in this order:

1. Verify the checkpoint's joint order, units, frame rate, and two-camera
   observation contract.
2. Confirm the inference server health reports the expected model and
   accelerator.
3. Run the policy in simulation.
4. Confirm the Pi lists JTAC and the launcher reports VLA readiness.
5. Run on hardware only with explicit motion authorization and physical power
   isolation available.
