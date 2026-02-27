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

After adding or removing bodies with joints, either:
1. **Remove the keyframe** and let MuJoCo use body `pos=` attributes for initial positions
2. **Regenerate the keyframe** from the MuJoCo Interactive Viewer (Ctrl+Shift+K) once the scene is stable

### MuJoCo documentation

Refer to [docs.picknik.ai](https://docs.picknik.ai) for MuJoCo configuration guides:

- [Physics Simulator Setup](https://docs.picknik.ai/how_to/configuration_tutorials/migrate_to_mujoco_config/) — creating scene.xml from URDF, camera/sensor setup, mesh conversion, MuJoCo Interactive Viewer
- [config.yaml Reference](https://docs.picknik.ai/how_to/configuration_tutorials/config_yaml_reference/) — `hardware` section for `picknik_mujoco_ros/MujocoSystem` plugin configuration
- [Simulator Keyframes Setup](https://docs.picknik.ai/how_to/configuration_tutorials/configure_keyframes/) — defining keyframes in scene.xml, `ResetMujocoKeyframe` Behavior
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
