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
pass through a self-return filter sized from the stowed footprint, then enter the live costmaps.
Localization uses the [Mid-360 slice](#shipped-mid-360-localization-map), not the low SICK scan
plane. `base_twist_to_planar.py` converts body-frame commands to actuator axes using private
encoder heading; missing, invalid or stale commands/feedback produce zero. The linear-speed
bound is 0.35 m/s, including diagonal motion, and yaw-rate bound is 0.5 rad/s.

## Livox Mid-360

A Mid-360 model is mounted **inverted above the fixed spine cap (C170)**, with scan
origin `(0.210, 0, 1.700)` m in `base_link`. X is the selected cap-center position
rounded forward by 0.250 mm, not the fixed spine link's origin. It does not move with the spine carriage.
The fixed joint rolls the native sensor frame 180° about X: its X remains forward,
Y points right and Z points down in `base_link`. The local accessory is declared in
`description/livox_mid360.urdf.xacro` and `mjcf/livox_mid360.xml`; the vendored
`franka_description/robots/mobile_fr3_duo_v0_2` macros remain unchanged. Its 65 × 65 × 60 mm,
265 g envelope adds no joints or keyframe values and fits the stowed navigation footprint.
The **A2 printable reference mount** has two PLA blades bolted directly onto the tower
top, no base plate, a filleted crown and a flush 4 mm 6061 heat plate. Its assets,
unit/frame convention, conversion checks and estimated 0.371 kg mount inertia are
recorded in [the asset README](description/meshes/livox_mid360/README.md).
Detailed visuals and separate collision hulls preserve the sensor's central clearance;
no whole-mount convex hull fills it. The print masses are slicer estimates, not weighed
hardware. Neither the mount nor the illustrative housing is a fabrication, strength or
thermal certification.

| Contract | Simulated value |
|---|---|
| Cloud | `/livox/lidar`, `sensor_msgs/msg/PointCloud2`, XYZ float32 |
| Message frame | `livox_frame` (robot_state_publisher); optical site TF from MuJoCo only |
| Horizontal / vertical FOV | 360°; native −7° to +52°, mounted base/world −52° to +7° |
| Rate | Nominal 10 Hz; original wall-clock acquisition stamp, best-effort sensor QoS |
| Sampling | 333 azimuths × 60 elevations, 19,980 rays per message; row 7 is horizontal |
| Range | 0.1–40 m; conservative 10%-reflectivity range, not the 70 m bright-target maximum |
| Range noise | Gaussian σ = 7 mm, zero mean, applied along each ray by the native plugin |
| Missing returns | 35% independent dropouts plus actual scene occlusion and range limits |

This reuses `picknik_mujoco_ros`'s image-based lidar, like the two existing SICK scanners.
The scan axis stays vertical and renders native −70° to +70°;
`scripts/livox_mid360_cloud.py` retains native −7° to +52°, changes optical XYZ to the
co-located `livox_frame`, and publishes on arrival in its subscription callback when
acquisitions are at least 95 ms apart (100 ms target with 5 ms jitter tolerance). Overscan avoids clipping high-elevation rays at tile
edges on MoveIt Pro 10.2. Simply pitching a 360° camera would tilt its entire scan cone,
not produce the Mid-360's uneven vertical FOV. The intermediate `/livox/mujoco/points`
cloud is not the sensor output. Shared MuJoCo render/cloud rates stay at 20 Hz, leaving
the SICK navigation scans unchanged. AMCL's separate projection is described under
[Shipped Mid-360 localization map](#shipped-mid-360-localization-map). Stale or duplicate source
stamps are not replayed, so an overloaded renderer can reduce the observed rate.
A 40 s mounted-A2 check on MoveIt Pro 10.2.0 observed **7.58 Hz**, with 303 distinct
acquisition stamps and a mean of 12,670 valid points per message. The task processes
were restricted to CPUs 0–5, nice 19 and ordinary non-real-time scheduling; this is
an observed loaded-runtime rate, not proof of sustained 10 Hz. The model and the
live `/livox/lidar` cloud were also checked in the installed Desktop App.

The envelope, FOV, 200,000 points/s nominal rate and conservative range come from
[Livox's Mid-360 specifications](https://www.livoxtech.com/mid-360/specs).

**Capture calibration.** A stationary 60 s Mid-360s capture (609 messages) measured
10.000 Hz, 19,936 points per steady message, and 63.6% nonzero returns (startup/short
packets reduce the whole-bag mean to 16,972). Robust plane-fit range residuals were
5.6 mm on a ceiling patch at 2.0 m and 7.7 mm on a wall patch at 3.3 m. The 7 mm noise
and approximately 64% valid-point budget match that evidence without pretending the
capture's room-specific occluded wedge is intrinsic to the sensor.

**Mount comparison.** The initial upright/inverted studies were followed by a cap-only
fore/aft and height sweep. The actual top-cap footprint spans X = 0.132000–0.287500 m,
Y = ±0.1505 m at Z = 1.461 m; its center is X = 0.209750 m. Each of the back/current,
center and front positions was tested at 1.50–1.90 m, with and without an illustrative
support. The selected **C170** stays mechanically centered and clears the full downward
cap cone: the measured farthest cap radius is 0.159356 m, so scan Z must exceed
`1.461 + 0.159356 * tan(52°) = 1.664966 m`. The 1.70 m sample gives about 35 mm margin.
The B2/stowed study, with its common illustrative support, reported:

| Candidate scan origin in `base_link` (m) | Horizontal / full-FOV self blockage | Open near-floor visibility (0.5–3 m) | Open-start forward floor |
|---|---|---|---|
| Centered default `(0.209750, 0, 1.500)` | 1.5% / 55.2% | 1.5% (actual first floor 3.08 m) | 0% |
| Centered `(0.209750, 0, 1.600)` | 1.5% / 9.7% | 71.5% (first floor 1.25 m) | 78.5% |
| **C170** `(0.209750, 0, 1.700)` | 1.5% / 1.8% | 77.8% (first floor 1.33 m) | 80.2% |

Those are deterministic geometric LOS metrics, not detection probabilities; they use
an illustrative support, not the final A2 bracket. The extra 100 mm above 1.60 m adds
6.4 percentage points of B2 floor visibility and removes 7.9 points of full-FOV blockage.
Fore/aft shifts at 1.70 m do not give a material forward-coverage advantage over center.
At 1.90 m the horizontal ring is above the 1.836 m shelves and loses all shelf-face hits.
Those height-study numbers use the **earlier illustrative single-post support**, not
A2. In the compiled A2 model, 720 bearings × 60 mounted elevations give **6.94%**
bracket blockage, with two rear-diagonal sectors of about 12.5° each; horizontal
side aisle views remain clear. Other robot first hits are retained: 2.52% at the
initial pose and 0.26% in Stow, rising to 20.97% with the carriage raised. These are
geometric first-hit counts, not sensor detection probabilities or localization scores.
Its inverted −52° edge reaches nearby floor instead of looking mostly at the ceiling.
The low 1.50 m default's cap can be clipped by the image renderer's near plane; native
floor points leaked behind that cap were not counted as geometric visibility.
Livox's Mid-360 User Manual explicitly supports upside-down mounting with mounting
surface at least 0.5 m above the ground; the existing low safety scanners remain necessary.
The A2 mount is collision-checked at ready, Stow and raised-Stow, with zero and
10 mm self-padding. This is **not** arbitrary extended-arm or full-carriage-motion
acceptance; raised carriage notably worsens full-FOV visibility. Native rendered occlusions remain
in the cloud; no opaque robot self-return is hidden. The closed optical-window
illustration is a URDF visual shown in the Desktop only; MuJoCo has no geom for that
transmissive window, so lidars and color cameras still see every opaque housing, bracket,
arm, tower and store surface but no obstacle at the scan origin. SRDF excludes only the two fixed
attachment contacts (spine/bracket and bracket/sensor), not sensor contacts with arms
or world obstacles.

**Limitations.** Sampling is an organized angular approximation, not the real
non-repeating rosette/time-per-point pattern. Dropouts approximate one capture's valid
budget, not a reflectivity-dependent return model. No intensity, tag, line or per-point
timestamp fields are synthesized, and no additional Livox IMU is emulated. The Duo's
existing IMU is unchanged; a lidar-inertial consumer expecting `/livox/imu` therefore
has no simulated Livox IMU input. AMCL uses the 2D projection described below; this does not
establish real-Livox or lidar-inertial localization support.

Run normally with `moveit_pro run -w "$PWD" -c market_sim` (use the repository's
worktree-specific config/image/instance procedure when validating from a worktree).
In the Desktop App, open **3D Visualizer Settings → Sensor Streaming** and enable
**livox_lidar** only; enabling **livox_mujoco** would display the uncropped diagnostic
source. Use `ros2 topic info -v /livox/lidar` and a bounded `ros2 topic hz /livox/lidar`
inside the selected runtime graph to verify publishing. `test/test_livox_mid360.py`
checks the compiled mount, horizontal first hits, full kept-ring coverage through the
installed native stitcher, range noise/dropouts, frame/schema conversion and fresh 10 Hz
ROS delivery. To revert just the slice integration, restore the previous AMCL/SLAM scan and map
selection, refinement scan checks and launch additions together. Removing the accessory also
requires reverting its model and xacro additions; no external Livox driver is installed.

## Teleoperation

Teleoperation jogs each arm, and the `mobile_base` group jogs the base (joint jog through
`base_jvc`, pose jog through `base_vfc`). Nav2 drives the base through `base_jgvc`.
The `spine` group joint-jogs the spine through `spine_jvc`.

## Localization and navigation Objectives

- **Localize Robot**: place an interactive marker where the robot really is; the Objective seeds
  Beluga AMCL there, asks it for in-place updates, and keeps the result only if the Mid-360 slice fits
  its survey map (ScanMatchResidual), otherwise it restores the previous estimate. **Refine Localization
  In Place** does the same from the current estimate, without the marker. **Localize Robot If
  Needed** and **Refine Localization In Place Subtree** are their building blocks. They follow
  `hangar_sim`'s Objectives of the same names.
- **Two active maps.** AMCL and Refine Localization In Place use the surveyed
  `maps/market_livox_h170.yaml` / PNG on `/localization_map`. Nav2's costmaps still use the generated
  `maps/market.yaml` / PNG on `/map`, every gondola's full footprint, so the base never plans onto
  a shelf deck. The old generated SICK-plane `market_localization` map is retained as a geometry
  reference, not selected for AMCL.
- AMCL consumes `/livox/scan`, the [Mid-360 projection](#shipped-mid-360-localization-map).
  Nav2 still reads `/scan_front` and `/scan_rear` for obstacle detection. `/scan` and
  `/scan_mapping` remain merged SICK diagnostics only. Neither localization nor ordinary
  navigation uses live ground truth; the slice projection reads no odometry or truth.
- **Navigate with NavFn** and **Navigate with SMAC**: click a goal on the map. They differ only in
  the planner. NavFn plans for the robot's centre point; the Smac State Lattice planner checks the
  whole footprint along the plan. The Navigate to Aisle Objectives use NavFn.
- Every Objective that drives the base to one goal, including **Navigate to Clicked Point** and **Move Base to Ready**, calls **Drive Base using Nav2**: it runs **Stow Arms for Navigation** (both arms and the spine go to the `Stow` waypoint), gives Nav2 the base controller, and drives with Nav2's default navigate-to-pose tree, which replans and recovers. Its `planner_id` port picks the planner: Nav2's installed tree serves NavFn, and a copy with SmacLattice as its planner serves SMAC. The launch file that starts `bt_navigator` writes that copy at startup, and `test/test_nav_tree_files.py` keeps the Objective and the launch file on the same two paths. Both costmaps use the Duo's asymmetric stowed polygon. Tests verify containment of compiled collision geometry, rack clearance at aisle goals and turns, and sufficient inflation radius. Navigation with extended arms is not covered by this footprint.
- Omni MPPI permits lateral translation and rotation without a differential-drive rotation shim. Jazzy MPPI clips X/Y independently, so each navigation axis is limited to 0.247 m/s: the rectangular feasible set's diagonal is below the bridge's 0.35 m/s total-speed cap. The velocity smoother uses the same axis limits. Straight-axis navigation is therefore slower than the bridge's total-speed limit; no downstream diagonal scaling changes the predicted yaw/translation ratio. Beluga AMCL uses `OmniMotionModel`; the State Lattice planner uses Nav2's matching omnidirectional primitives.
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
contact docking controller. `Undock From D4` uses the installed `PublishVelocityForceCommand` Behavior to publish velocity-only commands through the existing command bridge, not directly to a controller. The eight-second reverse and half-second zero command are bounded; cleanup also sends zero on failure or cancellation. The Objective acquires exclusive bridge command ownership before reversing and releases it only after zero-command cleanup; release also runs when acquisition fails or the Objective is stopped, so a failed acquisition clears an owner leaked by an earlier execution. Nav2 and `/cmd_vel` messages are discarded while undock owns the command, even if undock input is stale or invalid. Release publishes zero and clears stored commands; only a newly received external command can resume motion. This source priority does not cancel an old Nav2 goal. The bridge retains its localization/encoder watchdog and rejects stale input after 0.5 s.
If the agent crashes mid-undock and Nav2 or `/cmd_vel` stays ignored, recover with `ros2 service call /market_sim/release_undock std_srvs/srv/Trigger`.

In-place refinement samples a 10 cm / 0.1 rad standard deviation around the current estimate. A
broader global localization seed can lose the correct mode in the repetitive store geometry; the
general localization subtree retains its broader defaults. The Mid-360 slice must still pass the
90% scan-fit check against its survey map.

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
them from both generated maps; the survey map instead records returns in its
[projection band](#shipped-mid-360-localization-map). Both SICK lidars and the live costmaps still
detect the carts as unmapped navigation obstacles. **Add Racks to Planning Scene** loads
`description/market_racks.urdf`, one collision box per rack, wall, register, the D4 dock and each
cart, plus a floor slab.

The shared MuJoCo render buffer and color cameras use 1280×721 pixels. An odd render height places
the horizontal lidar ring on the optical-axis pixel; an even height samples an adjacent elevation,
which can hit a shelf deck while publishing the return as horizontal.

The cameras and lidars render at most 2000 objects. `test/test_market_scene.py` keeps the compiled
store and robot below that limit with a margin.

## The keyframe and resets

The store is empty: every gondola is a static pair of shelf units that collides as one box, and
the scene has one keyframe, `default`, with the robot at the start pose.

`Reset MuJoCo Sim` returns to `default`. `reset_controller_guard.py` snapshots active command controllers through `/market_sim/prepare_reset_controllers` and verifies deactivation before the keyframe reset. The snapshot survives failed Objective instances and partial manual reactivation; empty controller switches are skipped. The Objective calls `/market_sim/synchronize_localization`, then `/market_sim/restore_reset_controllers` verifies restoration of the original set and clears the snapshot only on success. No controller set is guessed if all command controllers were intentionally inactive. Do not restart the guard to recover a failed reset: its snapshot is process-local. `reset_relocalizer.py` uses the
explicit reset truth pose to reset Fuse, waits for fresh fused odometry and planar TF, then seeds
AMCL and waits for a matching pose and TF. The encoder bridge discards the teleport interval. A
failed or timed-out reset leaves command controllers inactive and the Nav2 command bridge at zero.
With the Mid-360 slice, AMCL can converge incorrectly after a teleport and trigger an
`amcl_pose_and_tf` timeout. A fresh stationary known-pose seed alone does not release the failed
reset's controller hold; do not bypass it. Check the `reset_relocalizer`, Fuse and AMCL logs,
restore localization inputs and retry **Reset MuJoCo Sim**, or restart the simulation at its
known default pose. Reset/odometry discontinuity handling is unchanged. Teleports from the
MuJoCo viewer also trigger synchronization; they do not restore controllers themselves.

The command bridge requires fresh `/market_sim/localization_ready` and `/market_sim/reset_controller_hold` heartbeats, and stops when either expires or reset controller preparation holds it. The controller hold remains asserted through any reset failure and is released only after successful localization synchronization and verified controller restoration. Outside a reset, missing, stale or invalid ground truth also holds commands at zero, and
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

It needs numpy. It writes `mjcf/market/`, `mjcf/assets/market/`, the generated `market` and
`market_localization` maps in `maps/`, `description/market_racks.urdf` and the `Navigate to Aisle`
objectives. Never edit those generated files by hand: a test fails when they differ from what the
script writes. Regenerate after changing the scene geometry or the lidar mounts. It does **not**
regenerate or overwrite the survey-built `market_livox_h170` map; re-survey after a scene change.

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
Mapping replaces both map servers and AMCL with slam_toolbox and a map saver; only
slam_toolbox publishes `map -> odom`. Fuse remains the planar-encoder/IMU odometry source and
receives no live truth corrections. slam_toolbox consumes the same `/livox/scan` h170 projection as
AMCL; unobserved bins are infinity, not fabricated free-space endpoints. Scan matching and
loop closure are enabled. The local search and odometry penalties constrain matching in repetitive
shelving; the separate loop search can correct accumulated position drift.

### Shipped Mid-360 localization map

`maps/market_livox_h170.png` is the verified **h170-pm02 survey-built** map, not a rasterized
geometry control. It came from a **partial whole-store survey: 52 of 56 ordered waypoints,
1,205.52 m over 7,006.67 s**. The survey timed out before the final four waypoints; unknown cells
and gaps are retained, so this is not a claim of complete store coverage. `slam_toolbox` 2.8.5 built
it from projections of 49,514 native `/livox/lidar` clouds and 335,245 recorded fused-odometry
messages, with zero filter/queue drops, scan matching and loop closures. Ground truth guided only
the survey vehicle, never the mapper's odometry or scan projection.
Map loading metadata is owned by [`maps/market_livox_h170.yaml`](maps/market_livox_h170.yaml).
The installed image and metadata are pinned by
[`test_installed_map_is_the_verified_partial_survey_not_the_geometry_control`](test/test_livox_localization_scan.py).
The PNG SHA256 is `4434bfc188506534226b1ee9b5893ff2b2736e98cfd6c1b43f5219ef291abb7a`; only the YAML
image basename was changed when installing it here.

`scripts/livox_localization_scan.py` reproduces the study's projection exactly:

| Projection contract | Value |
|---|---|
| Input / output | `/livox/lidar` PointCloud2 → `/livox/scan` LaserScan |
| Height band | 1.70 ±0.02 m in `base_link`, including both boundaries |
| Self gate | sensor-centred XY range ≥0.50 m; not distance from `base_link` |
| Source / planar range | finite 3D source range 0.1–40 m; nearest planar hit per bin |
| Bearings | 333 circular nearest-centre bins; first centre `-π + π/333`, spacing `2π/333` |
| Scan frame | `livox_slice`, X forward/Y left, origin `(0.210, 0, 0)` m in `base_link` |
| Timing / empty bins | original acquisition stamp, 0.1 s scan time; infinity for unobserved bins |

The original inverted `livox_frame` is unchanged. The scan origin projects its XY onto base Z=0,
matching the mapper; using `base_link` as the scan origin or forgetting the inverted Y/Z would
change the studied measurement. The small adapter is necessary because `pointcloud_to_laserscan`
uses edge-floor bins and a planar-only range gate rather than the study's nearest-centre circular
bins and separate 3D-source/self gates. The original cloud remains untouched for other consumers.

Installed Beluga AMCL settings are unchanged except for `scan_topic`; the existing 1,440 maximum
beams setting naturally uses all 333 available rays. Normal startup seeds the known default pose
at the origin, and reset retains the existing explicit known-pose synchronization. This is
**known-start tracking only**, not cold-start/global localization or real Livox acceptance. In
five held-out known-start replays on this map the study reported median 99.4% correct-and-narrow
tracking and 0.152 m route p95 error, with all five ending locked; those study scores are not a
new runtime measurement for every host or route.

A live MoveIt Pro 10.2.0 known-origin check navigated to B2 with AMCL, refined from the current
estimate, then tracked a 16.0 m aisle out-and-back: all goals succeeded, 100% of sampled poses were
correct-and-narrow, XY p95 error was 0.145 m, and it ended locked. Truth was scorer-only; no
parameters changed. An earlier 25.3 m open-floor loop also completed and ended locked (0.132 m XY
p95), but longitudinal covariance widened for part of the route: 83.1% correct-and-narrow samples.
Here “correct-and-narrow” means XY error ≤0.20 m, yaw error ≤0.10 rad, XY standard deviations
≤0.20 m and yaw standard deviation ≤0.10 rad, sustained for two seconds to confirm lock.
These are short known-start checks, not a new whole-store or cold-start acceptance claim.

For post-teleport AMCL convergence failures and controller-hold recovery, see
[The keyframe and resets](#the-keyframe-and-resets).

### Rebuilding a slice map

1. Rebuild the workspace, then launch `market_sim` with `MARKET_SIM_SLAM=true` and, for the
   simulation-only survey, `MARKET_SIM_GROUND_TRUTH_NAV=true`. Use the worktree-specific
   config/image/instance procedure if applicable. Confirm `/slam_toolbox` uses `/livox/scan`,
   `/livox/scan` has a publisher, and AMCL is absent. Do not change the projection above.
2. Drive a survey through the aisles and distinctive endcaps, recording `/livox/lidar`,
   `/odom_filtered`, `/tf` and `/tf_static` if a replayable capture is wanted. The supplied survey
   Objectives below cover a smaller area than the historical 52-waypoint whole-store prefix;
   collect additional aisle passes for wider coverage. Check for dropped scans/TF and retain
   uncovered cells as unknown rather than filling them from the generated footprint map.
3. Save outside the source tree (the directory must already exist), for example:

   ```bash
   ros2 run nav2_map_server map_saver_cli -t /map -f /tmp/market_livox_h170 \
     --fmt png --free 0.196 --occ 0.65 \
     --ros-args -p map_subscribe_transient_local:=true
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     '{filename: /tmp/market_livox_h170_posegraph}'
   ```

   Verify successful responses, both image/YAML and pose-graph files, and map/scan alignment.
   A fresh survey will not be byte-identical to this stochastic historical map. The retained
   study capture is identified by MCAP SHA256
   `20f8a6e5da6fb9813f923d5d96821d6b20444fa5fe34832cd7ca5cad431f8fd1`; exact study replay additionally
   used original-stamp fused-odometry interpolation and offline-only queue/cache controls
   (`scan_queue_size: 128`, `tf_buffer_duration: 7200.0`), not changes to matching or raster settings.
4. Install the replacement PNG/YAML together in `maps/` and update the launch file's
   `localization_map` default (or pass `localization_map:=/absolute/path/to/map.yaml` when launching
   the driver file directly). Keep `/map` on the full-footprint navigation map. Restart with both
   survey flags **false**, provide a known pose, and verify localization against the slice before
   navigating. Saving a map alone never switches AMCL to it.

### Survey Objectives

Run `Build Market Map` from the Desktop App to survey D4, the full B2 run and the aisles
immediately north and south, with two closed survey loops at the normal configured speed. It
requires both `MARKET_SIM_SLAM=true` and the survey-only `MARKET_SIM_GROUND_TRUTH_NAV=true`
described below: its goals are in the `world` frame, and Nav2 plans with the SmacLattice tree on
the generated `/navigation_map`, not on the growing SLAM map. slam_toolbox builds the map being
saved from the Mid-360 projection and fused odometry. Follow
[Rebuilding a slice map](#rebuilding-a-slice-map) to save and select it.

In this survey-only mode, navigation is independent of the map being built. Nav2 uses the
Simulator's `world -> base_link_ground_truth` transform and `/ground_truth/odom`, a separate
generated `/navigation_map`, and `/navigation_scan_front` and `/navigation_scan_rear` anchored to
that frame with the same rigid sensor extrinsics. SLAM's slice input and fused planar-encoder/IMU
odometry stay independent of truth navigation. This is a mapping-only mode, not evidence of
lidar-localized navigation; the launch file refuses it without
mapping mode, and the flag should stay false for localization and navigation validation. The flag
affects the next launch.

`Finish Market Map Survey` makes one complete loop for continuing a saved SLAM session. Restore the
serialized graph at its first node before running it after a cold start. It uses the same
ground-truth navigation mode and leaves SLAM on fused planar-encoder/IMU odometry.
