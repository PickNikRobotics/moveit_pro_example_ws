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
