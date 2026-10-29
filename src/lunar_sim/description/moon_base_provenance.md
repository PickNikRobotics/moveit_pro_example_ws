# Moon base geometry and materials

The moon base uses hand-authored MuJoCo boxes, cylinders, and capsules. It has no
texture files, meshes, downloaded imagery, or runtime-generated assets. The shapes
are simplified engineering geometry for collision and sensor landmarks, not a
reconstruction of a particular spacecraft or an illustration of a proposed mission.

Source: `moon_base_assets.xml` and `moon_base_geoms.xml` in this directory.
License: the package's [BSD-3-Clause license](../LICENSE), Copyright 2026 PickNik Inc.
Source URL: [lunar_sim in moveit_pro_example_ws](https://github.com/PickNikRobotics/moveit_pro_example_ws/tree/main/src/lunar_sim).

No measured material dataset is used. Untextured materials approximate plausible
visible albedo: off-white thermal blankets, dull grey metal, dark door hardware,
dark blue solar cells, muted gold insulation, and a dark grey prepared pad. Their
RGB values and highlights are engineering estimates, not measured lunar hardware
reflectances. There are no invented surface textures or synthetic space images.
The existing Apollo-derived regolith texture retains its separate
[provenance record](assets/ground_colormap_provenance.txt).

## Placement

All coordinates are metres in MuJoCo world space. The habitat, lander, solar mast and cargo sit north of the demo route; the rest are spread around the 10-15 m drivable band. Each has solid geometry at both lidar scan heights (0.46 m and 0.79 m above the ground). The habitat's door faces south-east, toward the route. Ten fixed bodies add no degrees of freedom. Every base geom has collision enabled and uses rendered group 1, so the same shapes can obstruct a rover and appear in depth-camera or image-based lidar returns.

| Body | Centre x, y | Yaw, rad | Sampled terrain z | Body datum z |
| --- | --- | --- | --- | --- |
| Habitat | -2, 6.6 | 0.464 | -0.010982 | -0.050982 |
| Landing pad | 7, 6.6 | 0 | 0.040610 | 0.000610 |
| Lander | 7, 6.6 | 0 | 0.040610 | 0.000610 |
| Vertical solar mast | -5.5, 6.6 | -0.6 | 0.002476 | -0.037524 |
| Cargo | -5, 8.4 | 0.3 | 0.004720 | -0.035280 |

These are offline samples of the committed `assets/lunar_hfield.png`, using the
row reversal and nearest-cell convention in `test_husky_mujoco_geometry.py`'s
`_hfield_elevation`. PNG values map to `value / 255 * 0.286 - 0.2095` metres.
Each datum is 4 cm below its centre sample. Supports, cabinets, the antenna foundation, the cargo, and the pad slab extend below the lowest terrain in their footprints.
The lander's feet meet the pad top at local z = 0.18 m.

The band bodies sit 4 cm below the bilinear height of the band's collision strips (`assets/lunar_band_*.png`, from `generate_band_terrain.py`) under their origin:

| Body | Centre x, y | Yaw, rad | Body datum z |
| --- | --- | --- | --- |
| Solar array | -12.3, 6.5 | 1.57 | -0.046032 |
| Antenna | 12.6, 3 | 1.2 | -0.042619 |
| Fission surface power | -12.6, -4 | 0 | -0.030276 |
| Logistics module | -4, -12.4 | 0.15 | -0.044122 |
| Rover shelter | 12.4, -9.5 | 3.4416 | -0.062495 |

The shelter's open front faces the base. Its walls, the cargo, the logistics skids and the power unit's pad reach below their datums, so uneven ground shows no gap under them.

By the validator's world-aligned bounding boxes, the southernmost geometry of the northern group is the landing pad at y = 4.50 m. `validate_moon_base.py` checks that every base geom stays outside the recorded square's footprint plus a 0.6 m rover radius, leaves at least 2 m beyond that radius around the square's far corner, and stays inside the 30x30 m drivable area.
