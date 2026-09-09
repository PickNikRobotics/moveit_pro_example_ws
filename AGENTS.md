# AI Code Assistant Instructions for MoveIt Pro Example Workspace

## MuJoCo Scene Files

### Keyframe qpos must match model DOF count

When editing `scene.xml` files (adding/removing bodies with joints), the `<keyframe>` section's `qpos` attribute must have exactly the number of values matching the model's total degrees of freedom. A mismatch causes `ros2_control_node` to crash with:

```
Error: keyframe 0: invalid qpos size, expected length <N>
```

Each joint type contributes to qpos:
- **freejoint**: 7 values (x, y, z, qw, qx, qy, qz)
- **hinge/slide**: 1 value each
- **ball**: 4 values (quaternion)

After adding or removing bodies with joints, **remove the keyframe** and let MuJoCo use body `pos=` attributes for initial positions.

### Velocity actuators: `armature/kv` time-constant must stay below the timestep

A `<velocity kv="...">` actuator on a joint with `armature="..."` behaves like a first-order servo with time-constant `τ = armature / kv`. If `τ` is larger than the scene `timestep`, the servo cannot inject enough velocity correction per step to overcome external load, and the joint effectively **stops responding to commands** — it stays pinned near zero even at full command. The per-step velocity correction scales as `kv · timestep / armature`, so halving the timestep halves the authority.

This bit `hangar_sim`'s mecanum base: the wheels had `armature="1.0"`, `kv="50"` → `τ = 0.02 s`. It worked only because the timestep was `0.025 s` (above τ). Standardizing the timestep to `0.003 s` dropped it well below τ, the wheel servos lost authority, the wheels pinned at ~0 rad/s, and the base would not drive (the whole-body `ExecuteTrajectory` then hung forever waiting for the base to reach goal). Fix was `kv: 50 → 500` (τ → 0.002 s, below the new timestep), verified in standalone MuJoCo to be stable across `timestep` 0.025→0.002. Lowering `armature` instead also raises authority but went unstable at small timesteps — prefer raising `kv`. (hangar's scene ultimately runs `timestep="0.008"`, coarser than the 0.003 s the other configs use: at 0.003 the CI runner overran ~47% of sim steps and starved controller mode-switching. `kv=500` keeps the wheels valid there too — τ=0.002 s < 0.008 s.)

The two coupled numbers live in different files: the actuator `kv` is in the `<velocity>` blocks of `hangar_sim/description/ur5e_ridgeback.xml` (~line 1709), and the joint `armature` is in the per-wheel includes (`hangar_sim/description/{front,rear}_{left,right}_wheel_link.xml`, the wheel `<joint>`).

Rule of thumb when changing a sim `timestep`: for every velocity actuator, check `armature/kv < timestep`. The symptom of violation is a joint that ignores commands (pinned), not one that oscillates.

### MuJoCo documentation

Refer to [docs.picknik.ai](https://docs.picknik.ai) for MuJoCo configuration guides:

- [Physics Simulator Setup](https://docs.picknik.ai/how_to/configuration_tutorials/migrate_to_mujoco_config/) — creating scene.xml from URDF, camera/sensor setup, mesh conversion, MuJoCo Interactive Viewer
- [config.yaml Reference](https://docs.picknik.ai/how_to/configuration_tutorials/config_yaml_reference/) — `hardware` section for `picknik_mujoco_ros/MujocoSystem` plugin configuration
- [Simulator Keyframes Setup](https://docs.picknik.ai/how_to/configuration_tutorials/create_robot_sim_config/configure_keyframes/) — defining keyframes in scene.xml, `ResetMujocoKeyframe` Behavior
- [Optimize Model Meshes](https://docs.picknik.ai/how_to/configuration_tutorials/optimizing_robot_model_meshes/) — MuJoCo enforces 1-200,000 faces per STL
- [Simulation Troubleshooting](https://docs.picknik.ai/troubleshooting/Simulation%20Troubleshooting/) — physics parameters, grip stability, mass/inertia errors, rendering issues

## Objective XML Files

### MetadataFields required for CI

Every objective XML file must include a `MetadataFields` block inside the `TreeNodesModel` section. The `validate_objectives` CI check will fail without it.

```xml
<TreeNodesModel>
  <SubTree ID="My Objective Name">
    <MetadataFields>
      <Metadata runnable="true" />
      <Metadata subcategory="Category Name" />
    </MetadataFields>
  </SubTree>
</TreeNodesModel>
```

- `runnable` — set to `"true"` for top-level objectives the user can run, `"false"` for subtrees only called by other objectives
- `subcategory` — groups the objective in the UI (e.g., `"AprilTag"`, `"Grasping"`, `"MuJoCo Simulation"`)

### A gripper config needs `close_gripper.xml` / `open_gripper.xml`, or teleop gripper silently fails

Teleoperation drives the gripper by looking up Objectives named exactly `"Close Gripper"` / `"Open Gripper"` (the `Request Teleoperation` SubTree in moveit_pro core). If a config package doesn't provide those overrides in its `objectives/` directory, the lookup falls back to moveit_pro's core placeholder, which logs `[ERROR] LogMessage Error: This robot configuration does not have a \`Close Gripper\` Objective configured to override this default.` on every BT tick for as long as the control is held, and the gripper never moves — even if some other Objective in the same config already drives the gripper directly via `MoveGripperAction` (that path bypasses the named-Objective lookup entirely). Any new config with a gripper needs both files; see `moveit_pro_kinova_configs/kinova_gen3_base_config/objectives/{close,open}_gripper.xml` for the reference pattern.

## Running MoveIt Pro from a git worktree

The user image tag is `moveit-pro-<svc>:<version>-<distro>-${MOVEIT_HOST_USER_WORKSPACE_NAME}`,
and that variable defaults to the workspace directory's basename. Every worktree
of this repo shares that basename, so a plain `moveit_pro build` from a worktree
overwrites the images built from the primary checkout. Set
`MOVEIT_HOST_USER_WORKSPACE_NAME` to something unique for the worktree, and pass
`-w "$PWD"` to `build` and `run`, which also keeps the CLI from repointing the
user's global config at the worktree.

Inside the containers, `ros2 node list` and friends return nothing until you run
`ros2 daemon stop` once: the daemon that survives from an earlier deployment
holds a participant that finds nothing on the current graph.

## One trajectory controller, several planning groups

When a config puts every joint on a single `joint_trajectory_controller` (the
right call when something other than MoveIt also drives the arm, since a second
controller claiming a joint's command interface locks the first one out), every
goal a planning group sends names a subset of the controller's joints and
`allow_partial_joints_goal` must be true, or the controller rejects all of them
with "Joints on incoming trajectory don't match the controller joints."

That controller has one owner at a time. A node publishing on its topic
interface restarts the trajectory on every message, so an action goal from a
plan is accepted and then never converges — or aborts on a path tolerance the
still-moving robot violated. Such a publisher has to yield: gate it on a
heartbeat the driving Objective ticks, and on the controller's
`follow_joint_trajectory/_action/status`. `so101_sim` does both.

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
