# Saved lunar localization map

`lunar_map.yaml` loads `lunar_map.png` at 0.05 m/cell, with origin
`[-13.226, -13.579, 0]` in the spawn-anchored `map` frame. The 552 × 585 raster
contains 232,636 free, 7,226 occupied and 83,058 unknown cells. It is an occupancy
map, not a manually drawn obstacle mask.

**slam_toolbox map from a MuJoCo-ground-truth-navigated run**

![The actual saved slam_toolbox map loaded by Beluga](lunar_map.png)

[Original annotated mapping image](lunar_map_annotated.png) ·
[Retained PGM/YAML, pose graph, trajectory and reproducibility evidence](evidence/README.md)

## Mapping provenance

The completed 11-goal mapping run used measured front lidar and wheel/IMU fuse
odometry. Experimental navigation used an independent, private ground-truth TF
tree; ground truth was not supplied to slam_toolbox. One tilt-guard stop required
an explicit clearance waypoint before retrying the interrupted goal. No original
goal was skipped. The saved map and original evidence were preserved unchanged;
two later tuning candidates regressed and were rejected.

- Known area: 599.655 m²; 97.15% of the earlier reference map's known area recovered.
- Timestamp-matched live mapping pose error: 0.364 m RMS, 0.961 m maximum.
- Visible-surface boundary distance against native MuJoCo geometry: pooled median
  0.112 m, 95th percentile 0.378 m. This measures lidar-visible surfaces, not all
  collision footprints or low rocks.

The later scene changes relative to the mapping source were ground texture tiling,
an additional non-colliding rear camera, and camera-renderer selection. The terrain,
landmarks and lidar mounts used by this map were unchanged.

## Beluga validation

With `slam: false` and `localization: true`, Beluga AMCL 2.1.1 loaded this map and
completed seven estimate-driven Nav2 goals over a 47.25 m measured trajectory,
including return to the spawn region. Both filtered lidar streams were interleaved
for Beluga, and fuse supplied odometry. No ground-truth navigation bridge, SLAM
node, or external initial-pose/reset publisher was present.

Independent MuJoCo truth and estimated TF were compared at the same timestamps,
without best-fit alignment:

| Metric | Measured result |
| --- | --- |
| Position error RMS / median | 0.240 m / 0.144 m |
| Position error 95th percentile / maximum | 0.514 m / 0.956 m |
| Yaw error RMS / maximum | 3.19° / 23.49° |
| Return localization error | 0.358 m |
| Largest truth distance from an accepted goal | 0.579 m |
| Real-time factor 5th percentile / median / 95th percentile | 0.956 / 0.968 / 0.985 |

The route completed in 291 s with tilt and pad-clearance guards active. This is a
bounded localization/navigation validation, not a full-field or global
relocalization guarantee. An earlier cold start left localization inactive after
`amcl/get_state` exceeded its service-client deadline; manual lifecycle resume
restored localization, but navigation startup had already timed out. The final
cold start activated both stacks without intervention. Startup repeatability is
therefore a remaining limitation; no unproven startup parameter workaround is
included.

### Frame and costmap stability

Fuse was verified live: wheel `/odom` and IMU data reached `state_estimator`, which
published `/odom_filtered` and `odom -> footprint`. Beluga alone corrected
`map -> odom`; the `mj_world -> map` anchor stayed static. The initial seed is an
intentional initialization, not a travel-time correction.

The longer route was **not jump-free**: the largest consecutive matched correction
at the robot was 0.213 m and 7.75° over 0.150 s during the second goal. The
95th-percentile correction was 0.012 m / 0.30°. This is measured localization
correction, not evidence that the saved raster changed or that the hangar timing
bug is reproduced here.

A separate short forward/spin check completed with a largest 0.022 m correction
over 0.150 s and largest 0.53° yaw correction. Actual rotation was 337° with 0.52 m
translation during the commanded in-place turn. Latest base TF age stayed below
0.092 s with no matched lookup failures after initialization. Beluga's
`map -> odom` is intentionally future-dated by its 1 s transform tolerance;
underlying correction-source age stayed below 0.170 s.

Across 26 global-costmap snapshots in that short check, lethal marks stayed within
0.5 m of saved occupied cells (final 95th-percentile separation 0.173 m), with no
broad displaced trails observed. That comparison is not independent obstacle
truth, and costmap stability during the longer route's large correction was not
recorded. Do not infer a universal stability or safe-navigation guarantee.

### Clicked-point Objective check

A bounded backend check on 2026-11-12 used the committed raster (live `/map`
pixels matched exactly), fuse and opt-in Beluga, with SLAM off. A headless UI
stand-in answered the real `GetPoseFromUser` and path-approval JSON prompts; this
exercises the Objective, not browser clicking. The `(2, -1)` goal succeeded
(0.109 m truth miss). After a guarded spin, the return Objective aborted on a
Nav2 map-to-odom TF extrapolation error (0.458 m truth miss); a cold repeat also
aborted on that error. Instantaneous path approval failed in an earlier attempt;
a two-second approval delay did not make navigation reliably succeed.

Across forward travel, spin and return, 669 timestamp-matched samples gave
0.144 m position RMS / 0.312 m maximum and 1.43° yaw RMS / 3.07° maximum.
Applying consecutive map corrections to the **same** odom-to-base sample gave a
largest sampled displacement of 0.0146 m over 0.150 s and largest yaw change of
0.358° over 0.153 s. These are sampled/interpolated TF corrections, not exact
AMCL update steps. Latest odom/base and map/base TF ages reached 0.121 s and
0.199 s; ten matched lookup failures occurred after initialization. The spin
rotated 314° and translated 1.06 m, so it was not an ideal in-place turn.

In 26 concurrent rolling global-costmap snapshots, the worst 95th-percentile
lethal-cell separation from saved occupied cells was 0.320 m, with up to eleven
cells beyond 0.5 m (maximum 1.357 m). This flags sparse displaced/live-only
marks, not independently proven smear or obstacle truth. The previously measured
0.213 m / 7.75° correction remains unresolved and was not reproduced here.
Tilt and padded pad-clearance guards stayed active; no failed goal was skipped
or localization pose corrected from truth. Navigation/TF repeatability remains
limited; these checks do **not** establish universal costmap stability.

The executable `saved_map_test` loads the installed YAML and image through Nav2's
real map loader and checks their occupancy and coordinate contract. It fails when
the map asset is missing.

## Scope and use

Set `slam: false` and `localization: true` in `config/config.yaml`, rebuild, and
restart the Runtime. Both switches remain off by default; SLAM takes precedence
if both are enabled. The ordinary product launch does not need the experimental
truth-navigation scripts.

The global and local costmaps still use live lidar obstacle layers. This map is
not a static costmap or a keep-out mask. Low rocks, pad edges and sloped terrain
below the scan planes require separate keep-out work; successful localization
does not establish that those routes are safe.
