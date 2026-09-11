# AI Code Assistant Instructions for MoveIt Pro Example Workspace

> This file is the canonical source of repository-wide instructions for every AI assistant. The `CLAUDE.md` beside it, plus `.cursorrules` and `.github/copilot-instructions.md`, are symlinks to it (the same convention as the moveit_pro repository). Edit `AGENTS.md`; never replace a symlink with a copy.

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

After adding or removing bodies with joints, either widen the keyframe's `qpos` to match or
**remove the keyframe** and let MuJoCo use body `pos=` attributes for initial positions. A
keyframe that has to stay (one a `ResetMujocoKeyframe` Objective or a `mujoco_keyframe`
hardware param targets) is worth guarding with a parse-the-MJCF length check - see
`test_keyframe_qpos_matches_model_dof_count` in
`src/lunar_sim/test/test_husky_mujoco_geometry.py`.

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
the consuming package's own `description/assets/` instead (cross-package MuJoCo mesh paths
don't survive a colcon install split, so the vendored STL meshes get copied in the same way).

`trimesh` is already present in the `picknikciuser/moveit-pro` runtime image, but its DAE loader
needs `pycollada`, which is not. Installing it into a `--rm` container only touches that
container's throwaway layer, not the image itself:

```
docker run --rm --mount type=bind,src=$PWD,dst=/work --entrypoint bash picknikciuser/moveit-pro:<tag> -c \
  "pip install --break-system-packages --no-cache-dir pycollada && python3 -c '
import trimesh
mesh = trimesh.load(\"/work/chassis.dae\", force=\"scene\").dump(concatenate=True)
mesh.export(\"/work/chassis.obj\", include_texture=False)
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
`pos` is still a fixed point in its parent body's frame - world-fixed only when the camera is a
direct child of `<worldbody>` (as in this repo's scene cameras); one nested under a moving body
would still be dragged along by that body, just no longer re-oriented independently. Either way,
for a scene camera meant to frame a mobile base throughout an
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

### `<texture file="...">` doesn't load JPEG

MuJoCo 3.6's built-in texture loader for `<texture type="2d" file="...">` accepts PNG, KTX, or its
own custom binary format - a `.jpg`/`.jpeg` file fails the model load with `Non-PNG texture,
assuming custom binary file format, unexpected file size`, not a clearer "unsupported format"
error.
Photoreal ground/wall textures are often distributed as JPEG (e.g. ambientCG, Poly Haven, or a NASA
mission-photo scan); convert to PNG before wiring into a scene - see
`src/lunar_sim/description/assets/lunar_regolith_untiled.png` and the texture's
provenance note in `src/lunar_sim/README.md`.

Separately: this repo's root `.gitattributes` LFS-tracks every `*.jpg`/`*.png`/`*.jpeg` with no
per-file exceptions (confirmed against every existing image asset in the repo, down to 19 KB
thumbnails) - a new texture/image asset should go through the normal `git add` + LFS flow like any
other, not a one-off `.gitattributes` carve-out. `*.stl`/`*.STL` are LFS-tracked the same way.

### `<compiler meshdir="...">` also resolves `<hfield file="...">`, unlike `<texture file="...">`

A `<compiler meshdir="assets">` attribute applies to both `<mesh>` **and** `<hfield>` file paths,
but not to `<texture>` (textures use the separate, here-unset `texturedir`, which defaults to the
model file's own directory). So with `meshdir="assets"`, a mesh reference like
`file="chassis.obj"` is correct un-prefixed, and an hfield PNG must be written the same
un-prefixed way (`file="lunar_hfield.png"`, not `file="assets/lunar_hfield.png"`) even though a
`<texture>` in the very same file needs the full `file="assets/regolith.png"` path. Getting this
wrong fails the model load with `Error opening file 'assets/assets/lunar_hfield.png'` (meshdir
prepended twice), not a clearer "not found" error. See
`src/lunar_sim/description/husky_scene.xml`'s `<hfield>` element.

### An `<hfield file="...">` PNG is loaded bottom-up: image row 0 lands at maximum y

MuJoCo flips a heightfield PNG's rows on load, so the image's *first* row becomes the *last*
`hfield_data` row - i.e. image row 0 is world `+size_y`, not `-size_y` (image column 0 is world
`-size_x`, unflipped). Verified against MuJoCo 3.6 by loading a ramp PNG and reading
`model.hfield_data`, plus downward `mj_ray` probes on a quadrant-coded hfield. A generator that
writes rows in increasing-y order silently produces terrain mirrored about y=0, which nothing
catches: the model loads, the render looks plausible, but anything else placed from the same
in-memory height array (rocks dropped onto the surface, a spawn pose) sits at the wrong ground
height. Emit the rows reversed - see `generate_terrain.py`'s `height_to_png`.

### `<geom>` has no per-instance mesh `scale` - put it on the `<mesh>` asset instead

MuJoCo's schema rejects `scale` on `<geom type="mesh">` (`Schema violation: unrecognized
attribute: 'scale'`); only the `<mesh>` asset itself takes `scale="x y z"`. To scatter many
instances of the same base mesh at different sizes (e.g. a small procedural rock library) without
duplicating geometry per instance, quantize the desired sizes into a handful of buckets and emit
one `<mesh>` asset per (variant, bucket) combo, each referencing the same STL file with a
different `scale`, then point each `<geom>`'s `mesh=` at the right bucketed asset name. See
`generate_terrain.py`'s `ROCK_SIZE_BUCKETS` / `rocks_assets_generated.xml`.

### Every fixed-mode MJCF camera needs a `<camera_name>_optical_frame` site once `render_publish_rate > 0`

`picknik_mujoco_ros/MujocoSystem` (`cameras.cpp`'s `extract_cameras()`) enumerates **every**
fixed-mode camera in the compiled MJCF model - not just ones referenced from the URDF - and
throws `The MJCF model does not define a site for the camera frame: <name>_optical_frame` at
hardware init if any of them lacks a matching `<site>`. A camera with `mode="targetbody"` (or
any non-fixed mode) is exempt. This bites a debug-only scene camera added for offline rendering
(e.g. `validate_and_render.py`) the moment `render_publish_rate` is turned on for the first time
- it will not surface from reading the xacro/URDF, only from actually running the stack. Fix: add
a `<site name="<camera>_optical_frame" pos="<camera pos>" quat="...">` at the camera's pose, with
the quat rotated 180 deg about the camera's own local X axis (MuJoCo camera convention -> ROS
optical-frame convention: flip Y and Z, keep X) - see `scene_camera_optical_frame` in
`src/lunar_sim/description/husky_scene.xml` or `src/factory_sim/description/scene.xml` for the
established pattern.

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
`MOVEIT_HOST_USER_WORKSPACE_NAME` to something unique for the worktree.

That value becomes a Docker image tag, so it has to be tag-safe: match
`[\w][\w.-]{0,127}` — letters, digits, underscore, then more of those plus `.`
and `-`. In particular **no slashes**, so a branch name like
`fm/captain-so101` cannot be used verbatim; flatten it (`fm-captain-so101`).
An unsafe value fails at `docker build` with an invalid-reference error rather
than at the CLI, which is a confusing place to learn it.

`-w "$PWD"` is not enough on its own to make the build read *this* worktree. The
docker build context comes from `MOVEIT_HOST_USER_WORKSPACE` in
`~/.config/moveit_pro/moveit_pro_config.9.yaml`, which still points wherever the
last `run` left it — quite possibly another lane's worktree, which then builds
silently and wrongly. Exporting the variable does not fix it either: the CLI
aborts on a config/environment mismatch. Give the CLI its own config instead:

```bash
export MOVEIT_HOST_USER_WORKSPACE_NAME=<unique>
export MOVEIT_PRO_CONFIG_DIR="/tmp/mpcfg-$MOVEIT_HOST_USER_WORKSPACE_NAME"
cp -rT ~/.config/moveit_pro "$MOVEIT_PRO_CONFIG_DIR"  # license key and port book
workspace_path="$(printf '%s' "$PWD" | sed 's/[\\&|]/\\&/g')"
sed -i "s|^MOVEIT_HOST_USER_WORKSPACE: .*|MOVEIT_HOST_USER_WORKSPACE: $workspace_path|" \
  "$MOVEIT_PRO_CONFIG_DIR/moveit_pro_config.9.yaml"
```

The config dir has to be per-lane too. A path every worktree shares means the
next lane's `cp -rT` restores the global copy under you, and your build silently
follows its `sed` to that lane's workspace.

Copying rather than starting empty carries over the port reservations that
existed at copy time, so `moveit_pro run --instance lane-a` starts from a book
that already knows about everyone else's deployments instead of handing out
port 3000 again.

Give each concurrent worktree its own instance name — `--instance lane-a` in
one, `--instance lane-b` in the next — and never reuse a name another lane is
running, since the name is the registry key. Be aware of the limit of the copy:
the CLI allocates ports from `instance_ports.yaml` inside
`$MOVEIT_PRO_CONFIG_DIR`, which is now a per-lane copy, so two lanes that each
start a *new* instance after copying can still pick the same block — the copy
prevents collisions with what already existed, not with a sibling lane racing
you. Symlinking the file back to the shared one does not fix it: the CLI writes
the registry with `os.replace`, which replaces the symlink with a regular file
on the first reservation. Stagger `moveit_pro run` between lanes, or check
`~/.config/moveit_pro/instance_ports.yaml` before starting, and confirm the
ports the banner prints.

The banner's "Your example workspace is at version <branch>" line names the
branch it actually read — check it before trusting a build.

Inside the containers, `ros2 node list` and friends return nothing until you run
`ros2 daemon stop` once: the daemon that survives from an earlier deployment
holds a participant that finds nothing on the current graph. A `docker exec`
also needs `CYCLONEDDS_URI=file:///home/<user>/.ros/cyclonedds.xml` passed
explicitly (`docker exec -e CYCLONEDDS_URI=...`) — the container's entrypoint
generates and exports that file only for its own PID 1 process tree, and
`docker exec` does not inherit env exported after container start, so a bare
exec's `ros2` CLI joins the wrong (default) CycloneDDS config and never
discovers the app's participants over the loopback-only, no-multicast peer
list that config sets up.

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
