# market_sim

A MoveIt Pro configuration for a Franka Mobile FR3 Duo in a simulated supermarket, under MuJoCo
physics, with Beluga AMCL localization and Nav2 navigation.

## The robot

`market_sim` is based on `mobile_fr3_duo_sim`, vendored from `moveit_pro_franka_ws` under
`src/external_dependencies/moveit_pro_franka_ws`. That package and its parent,
`mobile_fr3_duo_mock`, give the robot description, the planning groups, the controllers, the
odometry bridge and the scan and base-velocity nodes. `market_sim` sets only:

- its MuJoCo scene (`mujoco_model_package`), with the keyframes below;
- `publish_object_tf` off, so the loose products add no TF frames;
- its driver launch file: the supermarket map, Beluga AMCL and Nav2;
- its Objectives, listed last.

Frames: `world -> map` (static) `-> odom` (AMCL) `-> planar_x -> planar_y -> planar_theta ->
base_link`. The odometry bridge publishes the planar joints from the simulator's true base pose
plus drift, so AMCL has a real correction to make.

Teleoperation jogs each arm, and the `mobile_base` group jogs the base (joint jog through
`base_jvc`, pose jog through `base_vfc`). Nav2 drives the base through `base_jgvc`.

`mjcf/mobile_fr3_duo.xml`, `mjcf/sensors.xml` and the robot meshes in `mjcf/assets/` are copies of
`mobile_fr3_duo_sim`'s. MuJoCo resolves an included file's meshes against that file's own folder,
so the scene cannot include the robot from the other package. A test fails when the copies differ,
apart from one change: the planar base joints' travel range is widened to cover the store, and
`config.yaml`'s `base_travel_limit` sets the same limit in the URDF for MoveIt.

## Localization and navigation Objectives

- **Localize Robot**: place an interactive marker where the robot really is; the Objective seeds
  Beluga AMCL there, asks it for in-place updates, and keeps the result only if both lidar scans fit
  the map (ScanMatchResidual), otherwise it restores the previous estimate. **Refine Localization
  In Place** does the same from the current estimate, without the marker. **Localize Robot If
  Needed** and **Refine Localization In Place Subtree** are their building blocks. They follow
  `hangar_sim`'s Objectives of the same names.
- **Navigate with NavFn** and **Navigate with SMAC**: click a goal on the map. NavFn plans for the
  robot's centre point; the Smac State Lattice planner checks the whole footprint along the plan.
  Nav2 loads both planners; the Navigate to Aisle Objectives keep using NavFn.
- Every Objective that drives the base, including the inherited **Navigate to Clicked Point** and
  **Move Base to Ready**, first runs **Stow Arms for Navigation** from `mobile_fr3_duo_mock`: both
  arms and the spine go to the `Stow` waypoint. Nav2's footprint is that package's stowed-arms
  outline, kept equal by a test. The Navigate to Aisle goals stand far enough back from the shelf
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

The scene sets MuJoCo's sleep tolerance to 0.001. A standing can on a shelf keeps a tiny rocking
motion that the default tolerance never treats as still, so the cans would stay awake forever.
With the higher tolerance every resting product sleeps and costs almost nothing per step.

The simulated Franka Hand squeezes lightly, so, as in `lab_sim`, the loose products have a high
friction coefficient and the scene sets `impratio` and `noslip_iterations`; otherwise a gripped can
slides out of the fingers.

## Regenerating

```bash
python3 src/market_sim/scripts/generate_market.py
```

It needs numpy. It writes `mjcf/market/`, `mjcf/assets/market/`, `maps/market.{png,yaml}` and the
`Navigate to Aisle` objectives. Never edit those files by hand: a test fails when they differ from
what the script writes.

## Asset source

`market_assets/` holds the shelf and grocery-product models, generated in Blender by PickNik and
licensed BSD-3-Clause (`market_assets/LICENSE`). Only the generated meshes, MuJoCo bodies and
layout data are included; the Blender build scripts and preview renders are not.
