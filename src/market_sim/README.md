# market_sim

A MoveIt Pro configuration for a Husky A300 in a simulated supermarket, under MuJoCo
physics, with Beluga AMCL localization and Nav2 navigation.

## The robot

The Husky A300 uses `lunar_sim`'s Clearpath description, four-wheel MuJoCo model, SRDF, and base teleoperation Objectives. It has no arms, grippers, or spine. `description/husky_a300_mujoco.xacro` adapts the model package, ground-truth odometry, and TF ownership to the supermarket. `mjcf/husky_a300.xml` retains the lunar model's geometry and actuators, with byte-identical copies of the required lunar meshes and wheel includes under `mjcf/`.

The controller configuration is local to the market floor in `config/control/market_a300.ros2_control.yaml`. Its effective track width and rolling-radius multipliers are measured against MuJoCo ground truth using in-place turns in both directions, an arc, and a straight run. The floor contact model differs from lunar terrain, so the lunar calibration is not reused. Wheel encoders still cannot observe lateral skid. AMCL uses `OmniMotionModel` to represent lateral displacement uncertainty during turns; the navigation controller keeps differential-drive velocity commands.

Frames: `world -> map` (static), `map -> odom` (Beluga AMCL), `odom -> footprint` (Fuse wheel/IMU state estimator), then the robot description to `base_link`, chassis, and wheels. The simulator publishes ground truth without TF; the reset relocalizer uses it to re-seed AMCL. The wheel controller keeps publishing raw `/odom` with TF disabled. Fuse combines its body-frame linear velocity with the body-aligned MuJoCo IMU orientation and yaw rate, publishes `/odom_filtered` and owns `odom -> footprint`. Nav2 consumes the filtered topic; AMCL uses the filtered TF. There is no planar-joint or synthetic odometry bridge.

The IMU uses the same `framequat`, `gyro`, `accelerometer` and `IMUSensorBroadcaster` contract as `hangar_sim`. The ideal simulated orientation constrains heading; it does not measure lateral skid or resolve repetitive aisle geometry. Fuse does not consume ground-truth odometry or AMCL poses. Its parameters are in `config/fuse/fuse.yaml`.

The A300 carries its lunar OAK-D color camera (`/oakd_color/color`) and two 270-degree lidars. Nav2 uses differential-drive MPPI, a conservative 1.20 m by 0.80 m footprint enclosing the bumpers and wheels, and a Twist-to-TwistStamped adapter into the A300 controller's `/cmd_vel` topic. Pose teleoperation reuses the lunar base Objective. Each lidar scan passes through a footprint-sized self-return filter so the enclosure and arch do not enter localization or the costmaps. Localize Robot transforms the UI marker from its returned robot frame into `map`.

## Localization and navigation Objectives

The B2 demonstrations use `params/navigate_through_poses.xml`, which checks passed waypoints every navigation tree tick independently of the global planner throttle. Checking only when replanning can miss a waypoint while the base crosses its acceptance radius, leaving a path that doubles back through a goal already passed. The planner rate, waypoint radius and speed limits are unchanged. Both B2 demonstrations select the differential-drive State Lattice planner for their single-pose and through-poses legs, without requiring a `planner_selector` publisher. Other planners remain selectable through that topic.

- **Localize Robot**: place an interactive marker where the robot really is; the Objective seeds
  Beluga AMCL there, asks it for in-place updates, and keeps the result only if both lidar scans fit
  the map (ScanMatchResidual), otherwise it restores the previous estimate. **Refine Localization
  In Place** does the same from the current estimate, without the marker. **Localize Robot If
  Needed** and **Refine Localization In Place Subtree** are their building blocks. They follow
  `hangar_sim`'s Objectives of the same names.
- **Navigate with NavFn** and **Navigate with SMAC**: click a goal on the map. NavFn plans for the
  robot's centre point; the Smac State Lattice planner checks the whole footprint along the plan.
  Nav2 also loads Smac 2D and Theta* for comparison. The B2 demonstrations use State Lattice by default; the Navigate to Aisle Objectives keep using NavFn.
- Every navigation Objective activates `platform_velocity_controller` and uses the existing market map and aisle goals. Both costmaps share the A300 footprint. Tests check that it clears the racks at the aisle goals, including turns, and that the inflation radius covers its circumscribed radius.
- Nav2's rotation shim aligns with a new goal's path before differential-drive MPPI follows it. `rotate_to_heading_once` prevents periodic replans to the same goal from restarting that initial alignment. MPPI's path-alignment critic keeps `use_path_orientations: false`; enabling it in the B2 experiment caused prolonged cycling near the southwest turn.

These Objectives need MoveIt Pro `main`: `ReinterpretPoseFrame`, `CallEmptyService`,
`GetOccupancyGrid`, `GetLaserScan` and `ScanMatchResidual` are not in 10.1.0, so they will not load
there. The reset relocalizer also re-seeds AMCL a few seconds after a keyframe reset; let it finish
before running Localize Robot.

**Known limit: repetitive aisle geometry can produce an incorrect localization mode.** Nav2 judges goal completion from the estimated pose, so navigation success alone does not verify the physical shelf section or clearance. Use distinctive endcap geometry when initializing localization, and verify the scan against the map after a long aisle drive.

AMCL reads `/localization_map`, a map of visible geometry at both lidar heights, generated from the default MuJoCo scene by `scripts/generate_localization_map.py`. Its thin occupied surfaces let the likelihood field distinguish a shelf face from a return inside the shelf footprint. Nav2 retains the solid-footprint `/map` for collision checking. AMCL consumes `/scan_mapping`, the merger's timestamp-matched 360-degree stream from both lidars, so each observation represents one acquisition instant and unobserved bins remain invalid ranges. Neither map uses live ground truth during navigation. Regenerate the localization map whenever the scene geometry or lidar mounts change.

**Demo D4 B2 South and Return** seeds AMCL from the simulated dock's ground-truth transform before every undock, with 0.1 m XY and 0.1 rad yaw standard deviations. It then refines the current estimate in place and uses lidar localization throughout the drive and return. Its B2 centreline is 0.9 m south of the shelf face. This is an experimental simulation route: the ten-centimetre whole-route target is not verified, and a successful Objective is not an accuracy measurement. Two recorded trials with the omni model and synchronized scans held the central B2 straight within 1.2 cm, but whole-route maxima were 8.54 cm and 11.31 cm at turns. Those measurements used a local `fuse_constraints` overlay with absolute-yaw wrapping fixed; repeated missions still seed from the known simulated dock rather than testing unseeded long-term localization.

## The store

`scripts/generate_market.py` builds the store from the text floor plan at the top of the script.

| Plan symbol | In the model |
|---|---|
| One text column | One shelf unit's width |
| `-` | A gondola: two shelf units back to back, facing north and south |
| `}{` | A gondola facing west and east (the E aisles) |
| `][` | One steel block per checkout lane (R1 to R11) |
| `\|`, `_` | Walls; the gaps in the south wall are the doorways |

The aisles between the rows of `-` are 1.6 m wide. The robot starts on the open floor south of
row 1, at the origin of the world and map frames.

## Loose products and keyframes

Products on the gondolas of E1, E5, I2, A1, B2, C3 and D4 are loose: each has its own free joint
and can be picked. Every other product is drawn as part of its gondola, which collides as one box.

The 24 loose shelf units, 16 loose products each, are MuJoCo mocap bodies, so a keyframe can move
them. Twelve solid gondolas fill whichever spots the loose units leave, so every spot is filled in
every keyframe and one occupancy map fits them all.

| Keyframe | Loose shelf units are in |
|---|---|
| `default` (also the scene file's poses) | A1, B2, C3 and D4: three gondolas per row |
| `loose_a_d` | The same as `default` |
| `loose_e_i` | Every gondola of E1, E5 and I2 |

In A1, B2, C3 and D4 the gondolas on both sides of each loose one are empty in every keyframe.
`Reset MuJoCo Sim` returns to `default`; `Reset Market Keyframe` loads any keyframe by name (its
`keyframe_name` port). Every keyframe puts the robot back at the start pose, a jump that AMCL
cannot follow on its own, so `scripts/reset_relocalizer.py` re-seeds AMCL at the true pose a few
seconds after any reset.

The cameras and lidars render at most 2000 objects. The store draws 894 in every keyframe: 416
shelves, 384 loose products, the robot and the walls. `test/test_market_scene.py` keeps it under
that limit.

The scene enables multi-point collision detection and sets MuJoCo's sleep tolerance to 0.001. A standing can on a shelf keeps a tiny rocking
motion that the default tolerance never treats as still, so the cans would stay awake forever.
With the higher tolerance every resting product sleeps and costs almost nothing per step.

## Regenerating

```bash
python3 src/market_sim/scripts/generate_market.py
python3 src/market_sim/scripts/generate_localization_map.py
```

The store generator needs numpy. It writes `mjcf/market/`, `mjcf/assets/market/`, `maps/market.{png,yaml}` and the
`Navigate to Aisle` objectives. Never edit those files by hand: a test fails when they differ from
what the script writes. The localization-map generator also needs MuJoCo, Pillow and PyYAML from the workspace image; it writes `maps/localization.{png,yaml}` and has a separate reproducibility test.

## Asset source

`market_assets/` holds the shelf and grocery-product models, generated in Blender by PickNik and
licensed BSD-3-Clause (`market_assets/LICENSE`). Only the generated meshes, MuJoCo bodies and
layout data are included; the Blender build scripts and preview renders are not.

## Camera renderer

The default color renderer is MuJoCo OpenGL. To use Nyx for `scene_camera` and `oakd_color`, build the workspace image with the Nyx dependencies in the workspace Dockerfile, export `mjcf/scene.xml` into `$USER_WS/.nyx_scenes/market_sim` with `ros2 run picknik_nyx_renderer nyx_export_scene`, then set `MOVEIT_PRO_CAMERA_RENDERER=nyx` before launching. Export again after changing the MJCF or its assets. Both nodes use wall time, matching this configuration's `ros_global_params.use_sim_time: false`.

Nyx publishes each camera's `color` and `camera_info`; MuJoCo continues to publish depth, point clouds, and lidar. The optional visual-odometry stereo pair remains in MuJoCo. `MOVEIT_PRO_CAMERA_RENDERER=both` puts the color streams under `opengl/` and `nyx/` for comparison; consumers of the usual `color` topic must use `opengl` or `nyx` mode.

## Mapping the market

The gondolas are 2.0 m deep, with 4.0 m clear aisles between the horizontal rows. Decks and backs are deepened together; product dimensions and shelf-run lengths are preserved. The B2 demo centreline is 0.9 m south of the shelf face. Regenerate both maps after geometry changes.

Set `MARKET_SIM_SLAM=true` before starting the Runtime, or pass `slam:=true` directly to the additional driver launch file. The workspace Compose file forwards this flag to the drivers. Mapping replaces both generated-map servers and AMCL with Nav2's `slam_launch.py`; only slam_toolbox publishes `map -> odom`. Fuse remains the wheel/IMU odometry source and receives no ground truth. The two lidars are combined on `/scan_mapping` only when their acquisition timestamps match; empty bins are invalid rather than free-space rays. Scan matching and loop closure are enabled. The local search and odometry penalties constrain matching in repetitive shelving; the separate loop search can correct accumulated position drift.

Run `Build Market Map` from the Desktop App to survey D4, the full B2 run and the aisles immediately north and south, with two closed survey loops. It requires mapping mode and uses SMAC against the growing map; it navigates using the growing SLAM map at the normal configured speed. Save the occupancy grid using `nav2_map_server map_saver_cli` and the pose graph using `/slam_toolbox/serialize_map`; verify both service responses and files before using them. Saving does not select the map for AMCL.

For mapping drives that must be independent of SLAM, also set `MARKET_SIM_GROUND_TRUTH_NAV=true`. Nav2 then uses the Simulator's `world -> chassis_link_ground_truth` transform and `/ground_truth/odom`, a separate generated `/navigation_map`, and `/navigation_scan_front` and `/navigation_scan_rear` anchored to the ground-truth chassis with the same rigid sensor extrinsics. SLAM keeps its original filtered scans and fused wheel/IMU odometry. Neither SLAM's map nor its `map -> odom` estimate can steer this navigation mode. The flag affects the next launch; changing the host environment does not change a running drive.

`Finish Market Map Survey` makes one complete loop for continuing a saved SLAM session. Restore the serialized graph at its first node before running it after a cold start. It uses the same ground-truth navigation mode and leaves SLAM on fused wheel/IMU odometry.

The D4 demonstration initializes from the known simulated dock with 0.01 m² /0.01 rad² covariance and uses 150 updates during the unchanged 90% scan-fit refinement. It uses a lidar-visible simulated box dock, backs straight out for four seconds at 0.5 m/s, stops, and runs `Refine Localization In Place` using the current estimate before navigating. The dock face leaves 0.2 m beyond the nominal A300 footprint at `(25, 0)`; this is a simulated stand-off, not a contact docking controller. Nav2 permits 1.2 m/s translation and 1.2 rad/s rotation, within the simulated wheel controller's 1.5 m/s and 2.0 rad/s limits. The undock command timestamps refresh each cycle; interruption is bounded by the wheel controller's 0.5 s command timeout.

In-place refinement samples a 10 cm / 0.1 rad standard deviation around the current estimate. A broader global localization seed can lose the correct mode in the repetitive store geometry; the general localization subtree retains its broader defaults. Each lidar must still pass the 90% scan-fit check.

The store uses Poly Haven anti-skid floor tiles, generated plaster textures, ceiling light panels and two fixed stocking carts from PickNik simulation assets. Asset attribution and retained licenses are in `mjcf/assets/store_visuals/NOTICE.md`. Shelf geometry keeps this config's 2 m gondolas and 4 m aisles. Cart templates live in `market_assets/visuals/`; `generate_market.py` writes their physical world geometry while omitting them from the navigation map; `generate_localization_map.py` also excludes them. Both lidars and the live costmaps still detect the carts as unmapped obstacles.

The shared MuJoCo render buffer and color cameras use 1280×721 pixels. An odd render height places the horizontal lidar ring on the optical-axis pixel; an even height samples an adjacent elevation, which can hit a shelf deck while publishing the return as horizontal. Lidar field of view, output beam count, sensor mounts and map geometry are unchanged.
