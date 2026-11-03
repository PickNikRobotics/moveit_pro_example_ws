# market_sim

A MoveIt Pro configuration for a Franka Mobile FR3 Duo in a simulated supermarket, under MuJoCo
physics, with Beluga AMCL localization and Nav2 navigation.

## The robot

`market_sim` is a standalone configuration. Its robot description, planning groups, controllers,
odometry bridge and scan and base-velocity nodes are copied from the Franka Mobile FR3 Duo
configuration in `moveit_pro_franka_ws` (`mobile_fr3_duo_mock` and `mobile_fr3_duo_sim`, at commit
`940aa42`). The copy is deliberate: each sim in this workspace is an independent, standalone
package, so robot files are duplicated across sims instead of shared. The robot configuration was
copied from another workspace at a fixed commit, and `market_sim` does not track that source, so an
upstream fix has to be copied here by hand. `NOTICE` lists the third-party sources of the package's
files. The robot macros come from the vendored `franka_description`. Around the robot it adds:

- its MuJoCo scene, with one keyframe (below);
- its driver launch file: the supermarket map, Beluga AMCL and Nav2;
- its Objectives, listed last.

Frames: `world -> map` (static) `-> odom` (AMCL) `-> planar_x -> planar_y -> planar_theta ->
base_link`. The odometry bridge publishes the planar joints from the simulator's true base pose
plus drift, so AMCL has a real correction to make.

Teleoperation jogs each arm, and the `mobile_base` group jogs the base (joint jog through
`base_jvc`, pose jog through `base_vfc`). Nav2 drives the base through `base_jgvc`.
The `spine` group joint-jogs the spine through `spine_jvc`.

`mjcf/mobile_fr3_duo.xml` and `mjcf/sensors.xml` are also copied from `mobile_fr3_duo_sim`. The
robot meshes come from their own sources (`NOTICE`): the arm, hand, spine, head, mount and base
collision meshes load from the vendored `franka_description` through `package://` paths, the finger
meshes are MuJoCo Menagerie's, and the base's five visual meshes, one per colour, are converted from
`franka_description`'s `tmrv0_2.dae`. The planar base joints' travel range covers the whole store, in
both `mjcf/mobile_fr3_duo.xml` and `description/mobile_fr3_duo.urdf.xacro`; a test keeps the two
equal.

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
  panels, the registers and the walls); against the full footprint, AMCL could slide the base by
  the deck's depth. Nav2's costmaps use `maps/market.png`, every gondola's full footprint, so the
  base never plans onto the deck. A test casts the lidar plane in the scene and checks it against
  the localization map.
- **Navigate with NavFn** and **Navigate with SMAC**: click a goal on the map. They differ only in
  the planner. NavFn plans for the robot's centre point; the Smac State Lattice planner checks the
  whole footprint along the plan. The Navigate to Aisle Objectives use NavFn.
- Every Objective that drives the base, including **Navigate to Clicked Point** and **Move Base to
  Ready**, calls **Drive Base using Nav2**: it runs **Stow Arms for Navigation** (both arms and the
  spine go to the `Stow` waypoint), gives Nav2 the base controller, and drives with Nav2's default
  navigate-to-pose tree, which replans and recovers. Its `planner_id` port picks the planner: Nav2's
  installed tree serves NavFn, and a copy with SmacLattice as its planner serves SMAC. The launch
  file that starts `bt_navigator` writes that copy at startup, and `test/test_nav_tree_files.py`
  keeps the Objective and the launch file on the same two paths. Nav2's footprint is the stowed-arms outline, the same in both costmaps (a test checks
  it). The Navigate to Aisle goals stand far enough back from the shelf
  that this footprint clears both racks while it turns to face the shelf, also checked by a test.
- Nav2's controller turns the base in place toward a new path before following it (Nav2's rotation
  shim around MPPI), so it never arcs into a shelf it faces, and its obstacle costs are weighted up to
  keep the arms off the racks. The inflation radius is the footprint's circumscribed radius, rounded
  up to the next 0.1 m; a test keeps them in step.

These Objectives need MoveIt Pro `main`: `ReinterpretPoseFrame`, `CallEmptyService`,
`GetOccupancyGrid`, `GetLaserScan` and `ScanMatchResidual` are not in 10.1.0, so they will not load
there. The reset relocalizer also re-seeds AMCL a few seconds after a keyframe reset; let it finish
before running Localize Robot.

**Known limit: AMCL can drift along the long aisles.** Inside an A to D aisle the lidars see two long,
parallel rack rows, which hold the estimate across the aisle but only weakly along it. After a drive
into an aisle AMCL has been measured up to 1.03 m off along the aisle, with the base up to 0.57 m
short of a Navigate to Aisle goal; across the aisle and in heading it stays within a few centimetres.
Nav2 judges the goal on that estimate, so the base can stop facing the wrong shelf section. Run
**Refine Localization In Place** after the drive to correct it (MoveIt Pro `main` only, see above).

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

## The keyframe and resets

The store is empty: every gondola is a static pair of shelf units that collides as one box, and
the scene has one keyframe, `default`, with the robot at the start pose. `Reset MuJoCo Sim` returns
to it. The reset is a jump that AMCL cannot follow on its own, so `scripts/reset_relocalizer.py`
re-seeds AMCL at the true pose a few seconds after it.

The cameras and lidars render at most 2000 objects. The store draws 502: 404 shelf pairs, the
registers, the walls and the robot. `test/test_market_scene.py` keeps it under that limit.

The scene keeps `lab_sim`'s grasp settings (`impratio` and `noslip_iterations`) and MuJoCo sleeping
for objects at rest.

## Regenerating

```bash
python3 src/market_sim/scripts/generate_market.py
```

It needs numpy. It writes `mjcf/market/`, `mjcf/assets/market/`, both maps in `maps/` and the
`Navigate to Aisle` objectives. Never edit those files by hand: a test fails when they differ from
what the script writes.

## Asset source

`market_assets/` holds the shelf models, made in Blender by PickNik and licensed BSD-3-Clause
(`market_assets/LICENSE`). The shelf meshes and their XML were made in Blender once; no build
script is included, and nothing in the build needs Blender. `market_assets/` is input for
`scripts/generate_market.py` only: no scene includes its XML files, and it is not installed.
