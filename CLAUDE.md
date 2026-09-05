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

### `<include>` inside a `<body>` discards the included file's own wrapping element

When an included file's root element is itself a `<body>` (e.g. one file per wheel, mirroring `hangar_sim/description/*_wheel_link.xml`), MuJoCo's compiler splices in only that root element's **children** — the wrapping `<body>`'s own `name`/`pos` are silently discarded, not nested. Two wheel bodies written this way collapse into one, and their `<inertial>` tags collide: `Schema violation: unique element 'inertial' found N times`.

The real body (with its real `name`/`pos`) must be declared inline at the include site; the included file's own root element is just a throwaway wrapper to satisfy "one root element" for valid standalone XML:

```xml
<!-- in the parent file -->
<body name="front_left_wheel_link" pos="0.256 0.2829 0.02913">
  <include file="front_left_wheel_link.xml" />
</body>
```

```xml
<!-- front_left_wheel_link.xml -->
<body>
  <inertial .../>
  <joint name="front_left_wheel_joint" .../>
  <geom .../>
</body>
```

Verified against MuJoCo 3.6.0 (`picknikciuser/moveit-pro:main-jazzy-amd64-cuda13.2-cudnn9`'s bundled Python bindings) with a minimal `mj_name2id`/body-count check; see `src/lunar_sim/description/husky_a300.xml` for a real usage.

### Velocity actuators: `armature/kv` time-constant must stay below the timestep

A `<velocity kv="...">` actuator on a joint with `armature="..."` behaves like a first-order servo with time-constant `τ = armature / kv`. If `τ` is larger than the scene `timestep`, the servo cannot inject enough velocity correction per step to overcome external load, and the joint effectively **stops responding to commands** — it stays pinned near zero even at full command. The per-step velocity correction scales as `kv · timestep / armature`, so halving the timestep halves the authority.

This bit `hangar_sim`'s mecanum base: the wheels had `armature="1.0"`, `kv="50"` → `τ = 0.02 s`. It worked only because the timestep was `0.025 s` (above τ). Standardizing the timestep to `0.003 s` dropped it well below τ, the wheel servos lost authority, the wheels pinned at ~0 rad/s, and the base would not drive (the whole-body `ExecuteTrajectory` then hung forever waiting for the base to reach goal). Fix was `kv: 50 → 500` (τ → 0.002 s, below the new timestep), verified in standalone MuJoCo to be stable across `timestep` 0.025→0.002. Lowering `armature` instead also raises authority but went unstable at small timesteps — prefer raising `kv`. (hangar's scene ultimately runs `timestep="0.008"`, coarser than the 0.003 s the other configs use: at 0.003 the CI runner overran ~47% of sim steps and starved controller mode-switching. `kv=500` keeps the wheels valid there too — τ=0.002 s < 0.008 s.)

The two coupled numbers live in different files: the actuator `kv` is in the `<velocity>` blocks of `hangar_sim/description/ur5e_ridgeback.xml` (~line 1709), and the joint `armature` is in the per-wheel includes (`hangar_sim/description/{front,rear}_{left,right}_wheel_link.xml`, the wheel `<joint>`).

Rule of thumb when changing a sim `timestep`: for every velocity actuator, check `armature/kv < timestep`. The symptom of violation is a joint that ignores commands (pinned), not one that oscillates.

### Vendored A300 visual meshes: converting DAE to OBJ for MuJoCo

MuJoCo has no COLLADA (`.dae`) loader, but `clearpath_platform_description`'s A300 visual
meshes (`chassis.dae`, `livery.dae`, `status_lights.dae`, `attachments/bumper.dae`) are only
shipped as DAE - unlike the wheel/collision meshes, which are already STL. The vendored files
are kept byte-identical to upstream (never edit them); convert to OBJ and commit the result into
the consuming package's own `description/assets/` instead (same reasoning as the STL-copy note
above: cross-package MuJoCo mesh paths don't survive a colcon install split).

`trimesh` is already present in the `picknikciuser/moveit-pro` runtime image, but its DAE loader
needs `pycollada`, which is not. Installing it into a `--rm` container only touches that
container's throwaway layer, not the image itself:

```
docker run --rm --entrypoint bash picknikciuser/moveit-pro:<tag> -c \
  "pip install --break-system-packages --no-cache-dir pycollada && python3 -c '
import trimesh
mesh = trimesh.load(\"chassis.dae\", force=\"scene\").dump(concatenate=True)
mesh.export(\"chassis.obj\", include_texture=False)
'"
```

`include_texture=False` skips emitting a companion `.mtl` - apply color via a plain MuJoCo
`<material>` on the geom instead of trying to carry over per-face COLLADA materials. The
converted mesh keeps the DAE's own local-frame vertices, so it drops onto its parent body with
whatever `pos`/`quat` the URDF's visual `<origin>` implies - work that out from the xacro rather
than eyeballing it against a render; see `src/lunar_sim/description/husky_a300.xml`'s bumper
comment for a worked example (a mount-frame offset and a visual-origin counter-offset that cancel
to a plain identity transform).

### `mode="targetbody"` only rotates a camera, it doesn't move it

A `<camera mode="targetbody" target="...">` continuously re-aims to face the target body, but its
`pos` is still a fixed point in world space - unlike a camera attached to a moving body, it does
not follow the target around. For a scene camera meant to frame a mobile base throughout an
objective (not just at its starting pose), remember the viewing angle toward the rest of the
scene - the ground plane, in particular - changes as the base drives away from that fixed point,
which can reveal rendering artifacts (e.g. directional-light shadow-map aliasing past the shadow
frustum's edge, see `src/lunar_sim/description/husky_scene.xml`) that were not visible from the
starting pose. Verify renders at more than one point along the objective, not just at rest.

### `texrepeat` with `texuniform="true"` is repeats-per-metre, not repeats-over-the-whole-geom

With `texuniform="true"` on a `<material>`, `texrepeat="R R"` means the texture repeats R times
per metre of world space, not R times across the whole geom - the opposite of the more intuitive
"total tiles across this surface" reading. A 20x20 m ground plane with `texrepeat="8 8"` therefore
tiles every `1/8 = 0.125 m`, not every `20/8 = 2.5 m`; the smaller tile shows as an obvious
repeating grid at any zoom that puts more than a couple tiles in frame. To get a target tile size
of `S` metres, use `texrepeat="${1/S} ${1/S}"` (e.g. `0.3 0.3` for a ~3.3 m tile), independent of
the geom's own size.

### `<texture file="...">` only loads PNG, not JPEG

MuJoCo's built-in texture loader for `<texture type="2d" file="...">` accepts PNG (or its own
custom binary format) - a `.jpg`/`.jpeg` file fails the model load with `Non-PNG texture, assuming
custom binary file format, unexpected file size`, not a clearer "unsupported format" error.
Photoreal ground/wall textures are often distributed as JPEG (e.g. ambientCG, Poly Haven, or a NASA
mission-photo scan); convert to PNG before wiring into a scene - see
`src/lunar_sim/description/assets/lunar_regolith_as15-86-11671_2k.png` and the texture's
provenance note in `src/lunar_sim/README.md`.

Separately: this repo's root `.gitattributes` LFS-tracks every `*.jpg`/`*.png`/`*.jpeg` with no
per-file exceptions (confirmed against every existing image asset in the repo, down to 19 KB
thumbnails) - a new texture/image asset should go through the normal `git add` + LFS flow like any
other, not a one-off `.gitattributes` carve-out.

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
