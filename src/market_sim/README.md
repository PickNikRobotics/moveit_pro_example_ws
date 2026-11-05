# market_sim

A MoveIt Pro configuration for a Franka Mobile FR3 Duo in a simulated supermarket, under MuJoCo
physics, with Beluga AMCL lidar localization and omnidirectional Nav2 navigation.

## The robot

`market_sim` is a standalone configuration. Its robot description, planning groups, controllers
and base-velocity bridge are copied from the Franka Mobile FR3 Duo configuration in
`moveit_pro_franka_ws` (`mobile_fr3_duo_mock` and `mobile_fr3_duo_sim`, at commit `940aa42`). The
copy is deliberate: each sim in this workspace is an independent, standalone package, so robot
files are duplicated across sims instead of shared. `market_sim` does not track that source, so an
upstream fix has to be copied here by hand. `NOTICE` lists the third-party sources of the package's
files. The robot macros come from the vendored `franka_description`: two seven-joint arms, both
grippers, the vertical spine and the synchro-drive base. The synchro drive is modeled as three
omnidirectional planar actuators (`planar_x`, `planar_y`, `planar_theta`), not wheel-level steering
or skid dynamics.

`mjcf/mobile_fr3_duo.xml` and `mjcf/sensors.xml` are also copied from `mobile_fr3_duo_sim`. The
robot meshes come from their own sources (`NOTICE`): the arm, hand, spine, head, mount and base
collision meshes load from the vendored `franka_description` through `package://` paths, the finger
meshes are MuJoCo Menagerie's, and the base's five visual meshes, one per colour, are converted from
`franka_description`'s `tmrv0_2.dae`. The planar base joints' travel range covers the whole store, in
both `mjcf/mobile_fr3_duo.xml` and `description/mobile_fr3_duo.urdf.xacro`; a test keeps the two
equal.

Frames: `world -> map` (static), `map -> odom` (Beluga AMCL), then the URDF planar joints to
`base_link` (robot_state_publisher). A private `/base_state_broadcaster/joint_states` stream supplies
actuator positions. `odometry_joint_state_publisher.py` differences their stamped positions into
body-frame `/odom` velocity, so a slow simulation does not inflate distance by integrating simulated
velocity against wall time. Fuse combines this velocity with the body-aligned simulated IMU
orientation and yaw rate and publishes `/odom_filtered`, with TF disabled. The bridge publishes only
the fused planar joint state on `/joint_states`; raw planar joints cannot race it. Nav2 reads
`/odom_filtered`, and AMCL uses the fused TF chain.

The IMU uses MuJoCo `framequat`, `gyro`, `accelerometer` and `IMUSensorBroadcaster`. The ideal
simulated orientation constrains heading but does not resolve repetitive aisle geometry. Ordinary
fusion consumes neither ground-truth odometry nor AMCL poses. `config/fuse/fuse.yaml` selects the
sensor models; simulation reset seeding is the explicit exception described below. This planar
actuator model does not establish hardware wheel-odometry accuracy.

The Duo carries front and rear SICK nanoScan3 lidar models at `(0.3275, 0.2175, 0.19065)` and
`(-0.3275, -0.2175, 0.19065)` metres in `base_link`, each with a 275-degree sweep and 0.10–40 m
range. RGB cameras include `camera_front`, `camera_rear`, `camera_left` and `camera_right`. Scans
pass through a self-return filter sized from the stowed footprint, then enter synchronized
localization and live costmaps. `base_twist_to_planar.py` converts body-frame commands to actuator
axes using private encoder heading; missing, invalid or stale commands/feedback produce zero. The
linear-speed bound is 0.35 m/s, including diagonal motion, and yaw-rate bound is 0.5 rad/s.

Teleoperation jogs each arm, and the `mobile_base` group jogs the base (joint jog through
`base_jvc`, pose jog through `base_vfc`). Nav2 drives the base through `base_jgvc`.
The `spine` group joint-jogs the spine through `spine_jvc`.

## Localization and navigation Objectives

- **Localize Robot**: place an interactive marker where the robot really is; the Objective seeds
  Beluga AMCL there, asks it for in-place updates, and keeps the result only if both lidar scans fit
  the map (ScanMatchResidual), otherwise it restores the previous estimate. **Refine Localization
  In Place** does the same from the current estimate, without the marker. **Localize Robot If
  Needed** and **Refine Localization In Place Subtree** are their building blocks. They follow
  `hangar_sim`'s Objectives of the same names.
- **Two maps.** Both lidars scan a plane just above the shelves' bottom deck, so they see past
  the deck to the uprights and back panel. AMCL and Refine Localization In Place use
  `maps/market_localization.png`, which holds only what crosses that plane (the uprights, the back
  panels, the registers, the D4 dock and the walls); against the full footprint, AMCL could slide
  the base by the deck's depth. Nav2's costmaps use `maps/market.png`, every gondola's full
  footprint, so the base never plans onto the deck. A test casts the lidar plane in the scene and
  checks it against the localization map.
- AMCL consumes `/scan_mapping`, the merger's timestamp-matched 360-degree stream from both lidars,
  so each observation represents one acquisition instant and unobserved bins remain invalid ranges.
  Neither map uses live ground truth during navigation.
- **Navigate with NavFn** and **Navigate with SMAC**: click a goal on the map. They differ only in
  the planner. NavFn plans for the robot's centre point; the Smac State Lattice planner checks the
  whole footprint along the plan. The Navigate to Aisle Objectives use NavFn.
- Every Objective that drives the base to one goal, including **Navigate to Clicked Point** and **Move Base to Ready**, calls **Drive Base using Nav2**: it runs **Stow Arms for Navigation** (both arms and the spine go to the `Stow` waypoint), gives Nav2 the base controller, and drives with Nav2's default navigate-to-pose tree, which replans and recovers. Its `planner_id` port picks the planner: Nav2's installed tree serves NavFn, and a copy with SmacLattice as its planner serves SMAC. The launch file that starts `bt_navigator` writes that copy at startup, and `test/test_nav_tree_files.py` keeps the Objective and the launch file on the same two paths. Both costmaps use the Duo's asymmetric stowed polygon. Tests verify containment of compiled collision geometry, rack clearance at aisle goals and turns, and sufficient inflation radius. Navigation with extended arms is not covered by this footprint.
- Omni MPPI permits lateral translation and rotation without a differential-drive rotation shim. Beluga AMCL uses `OmniMotionModel`; the State Lattice planner uses Nav2's matching omnidirectional primitives.
- The B2 demonstrations use `params/navigate_through_poses.xml`, which checks passed waypoints every navigation tree tick independently of the global planner throttle. Checking only when replanning can miss a crossed waypoint and create a path that doubles back. Both demonstrations select the omnidirectional State Lattice planner for their single-pose and through-poses legs.

These Objectives require the installed Behaviors `TransformPoseFrame`, `ReinterpretPoseFrame`,
`CallEmptyService`, `CallTriggerService`, `GetOccupancyGrid`, `GetLaserScan` and
`ScanMatchResidual`. They are not supported on MoveIt Pro 10.1.0. Validate the active Behavior
library before using the configuration on another release.

**Known limit: repetitive aisle geometry can produce an incorrect localization mode.** Nav2 judges
goal completion from the estimated pose, so navigation success alone does not verify the physical
shelf section or clearance. Use distinctive endcap geometry when initializing localization, and
verify the scan against the map after a long aisle drive.

**Demo D4 B2 South and Return** explicitly seeds AMCL from `base_link_ground_truth` before each
undock, with 0.1 m XY and 0.1 rad yaw standard deviations. It then refines the current estimate in
place and uses lidar localization throughout the drive and return. Its B2 centreline is 0.9 m
south of the shelf face. Start at the D4 dock using **Navigate to Demo Dock D4**. This simulation
demonstration does not test unseeded long-term localization; a successful Objective is not an
accuracy measurement. Report pose error and clearance from synchronized state measurements for the
current robot, rather than inheriting measurements from another base.

The D4 demonstration uses a lidar-visible simulated box dock, backs straight out for eight seconds
at 0.25 m/s, stops, and runs 150 in-place localization updates before navigating. The dock face
leaves 0.209 m beyond the nominal stowed footprint at `(25, 0)`; this is a simulated standoff, not a
contact docking controller. `Undock From D4` uses the installed `PublishVelocityForceCommand` Behavior to publish velocity-only commands through the existing command bridge, not directly to a controller. The eight-second reverse and half-second zero command are bounded; cleanup also sends zero on failure or cancellation. The bridge retains its localization/encoder watchdog and rejects stale input after 0.5 s.

In-place refinement samples a 10 cm / 0.1 rad standard deviation around the current estimate. A
broader global localization seed can lose the correct mode in the repetitive store geometry; the
general localization subtree retains its broader defaults. Each lidar must still pass the 90%
scan-fit check.

## The store

`scripts/generate_market.py` builds the store from the text floor plan at the top of the script.

| Plan symbol | In the model |
|---|---|
| One text column | One shelf unit's width |
| `-` | A gondola: two shelf units back to back, facing north and south |
| `}{` | A gondola facing west and east (the E aisles) |
| `][` | One steel block per checkout lane (R1 to R11) |
| `\|`, `_` | Walls; the gaps in the south wall are the doorways |

The gondolas are 2.0 m deep, with 4.0 m clear aisles between the horizontal rows; the shelf asset
stretches in depth to fill the gondola. The robot starts on the open floor south of row 1, at the
origin of the world and map frames.

The store uses Poly Haven anti-skid floor tiles, generated plaster textures, a ceiling with light
panels and two fixed stocking carts from PickNik simulation assets. Asset attribution and retained
licenses are in `mjcf/assets/store_visuals/NOTICE.md`. Cart templates live in
`market_assets/visuals/`; `generate_market.py` writes their physical world geometry while omitting
them from both maps. Both lidars and the live costmaps still detect the carts as unmapped
obstacles. **Add Racks to Planning Scene** loads `description/market_racks.urdf`, one collision box
per rack, wall, register, the D4 dock and each cart, plus a floor slab.

The shared MuJoCo render buffer and color cameras use 1280×721 pixels. An odd render height places
the horizontal lidar ring on the optical-axis pixel; an even height samples an adjacent elevation,
which can hit a shelf deck while publishing the return as horizontal.

The cameras and lidars render at most 2000 objects. `test/test_market_scene.py` keeps the compiled
store and robot below that limit with a margin.

## The keyframe and resets

The store is empty: every gondola is a static pair of shelf units that collides as one box, and
the scene has one keyframe, `default`, with the robot at the start pose.

`Reset MuJoCo Sim` returns to `default`. It deactivates command controllers, resets, and calls
`/market_sim/synchronize_localization` before restoring controllers. `reset_relocalizer.py` uses the
explicit reset truth pose to reset Fuse, waits for fresh fused odometry and planar TF, then seeds
AMCL and waits for a matching pose and TF. The encoder bridge discards the teleport interval. A
failed or timed-out reset leaves command controllers inactive and the Nav2 command bridge at zero.
Check the `reset_relocalizer`, Fuse and AMCL logs, restore missing inputs, then retry **Reset MuJoCo
Sim**. Teleports from the MuJoCo viewer also trigger synchronization; they do not restore
controllers themselves.

The command bridge requires a fresh `/market_sim/localization_ready` heartbeat and stops when it
expires. Outside a reset, missing, stale or invalid ground truth also holds commands at zero, and
readiness returns once fresh ground truth has been continuous for 0.5 s; this recovery neither
reseeds the estimators nor restarts an aborted navigation goal. Outside reset/dock seeding,
ground-truth odometry is reserved for measurement and detecting simulation discontinuities, not
correcting the navigation estimate. In mapping mode reset synchronizes Fuse and planar TF but
leaves SLAM's map state under slam_toolbox control; restart mapping after a reset that invalidates
its trajectory.

The scene keeps `lab_sim`'s grasp settings (`impratio` and `noslip_iterations`), multi-point
collision detection and MuJoCo sleeping for objects at rest.

## Regenerating

```bash
python3 src/market_sim/scripts/generate_market.py
```

It needs numpy. It writes `mjcf/market/`, `mjcf/assets/market/`, both maps in `maps/`,
`description/market_racks.urdf` and the `Navigate to Aisle` objectives. Never edit those files by
hand: a test fails when they differ from what the script writes. Regenerate after changing the
scene geometry or the lidar mounts.

## Asset source

`market_assets/` holds the shelf models, made in Blender by PickNik and licensed BSD-3-Clause
(`market_assets/LICENSE`). The shelf meshes and their XML were made in Blender once; no build
script is included, and nothing in the build needs Blender. `market_assets/` is input for
`scripts/generate_market.py`, and is installed because the scene and the planning-scene URDF also
reference its cart materials, cart body and shelf meshes at runtime.

## Camera renderer

The default color renderer is MuJoCo OpenGL. `scene_camera` and `camera_front` use the shared
`picknik_accessories` renderer selection. To use Nyx, build the workspace image with
`INSTALL_NYX=true moveit_pro build` (the default `false` skips the Torch and Nyx downloads), export
`mjcf/scene.xml` into `$USER_WS/.nyx_scenes/market_sim_duo` with
`ros2 run picknik_nyx_renderer nyx_export_scene`, then set `MOVEIT_PRO_CAMERA_RENDERER=nyx` before
launching. Export again after changing the MJCF or its assets. Both nodes use wall time, matching
`ros_global_params.use_sim_time: false`.

Nyx publishes the selected cameras' `color` and `camera_info`; MuJoCo continues to publish other
cameras, depth, point clouds and lidar. `MOVEIT_PRO_CAMERA_RENDERER=both` puts the selected color
streams under `opengl/` and `nyx/` for comparison; consumers of their usual `color` topic must use
`opengl` or `nyx` mode.

## Mapping the market

Set `MARKET_SIM_SLAM=true` before starting the Runtime, or pass `slam:=true` directly to the
additional driver launch file. The workspace Compose file forwards this flag to the drivers.
Mapping replaces both generated-map servers and AMCL with slam_toolbox and a map saver; only
slam_toolbox publishes `map -> odom`. Fuse remains the planar-encoder/IMU odometry source and
receives no live truth corrections. The two lidars are combined on `/scan_mapping` only when their
acquisition timestamps match; empty bins are invalid rather than free-space rays. Scan matching and
loop closure are enabled. The local search and odometry penalties constrain matching in repetitive
shelving; the separate loop search can correct accumulated position drift.

Run `Build Market Map` from the Desktop App to survey D4, the full B2 run and the aisles
immediately north and south, with two closed survey loops at the normal configured speed. It
requires both `MARKET_SIM_SLAM=true` and the survey-only `MARKET_SIM_GROUND_TRUTH_NAV=true`
described below: its goals are in the `world` frame, and Nav2 plans with the SmacLattice tree on
the generated `/navigation_map`, not on the growing SLAM map. slam_toolbox builds the map being
saved from its own scans and fused odometry. Save the occupancy grid using `nav2_map_server
map_saver_cli` and the pose graph using `/slam_toolbox/serialize_map`; verify both service responses
and files before using them. Saving does not select the map for AMCL.

The survey Objectives also need `MARKET_SIM_GROUND_TRUTH_NAV=true`, so their navigation is
independent of the map being built. Nav2 then uses the Simulator's `world -> base_link_ground_truth`
transform and `/ground_truth/odom`, a separate generated `/navigation_map`, and
`/navigation_scan_front` and `/navigation_scan_rear` anchored to that frame with the same rigid
sensor extrinsics. SLAM keeps its original scans and fused planar-encoder/IMU odometry. This is a
mapping-only mode, not evidence of lidar-localized navigation; the launch file refuses it without
mapping mode, and the flag should stay false for localization and navigation validation. The flag
affects the next launch.

`Finish Market Map Survey` makes one complete loop for continuing a saved SLAM session. Restore the
serialized graph at its first node before running it after a cold start. It uses the same
ground-truth navigation mode and leaves SLAM on fused planar-encoder/IMU odometry.
