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

All coordinates are metres in MuJoCo world space. The habitat faces south, toward
the demo route. Five fixed bodies add no degrees of freedom. Every base geom has
collision enabled and uses rendered group 1, so the same shapes can obstruct a
rover and appear in depth-camera or image-based lidar returns.

| Body | Centre x, y | Sampled terrain z | Body datum z |
| --- | --- | --- | --- |
| Habitat | 0, 6.8 | -0.002010 | -0.042010 |
| Solar array | -4, 6 | -0.019955 | -0.059955 |
| Antenna | -2.8, 8.1 | 0.009206 | -0.030794 |
| Landing pad | 6, 6.4 | -0.002010 | -0.042010 |
| Lander | 6, 6.4 | -0.002010 | -0.042010 |

These are offline samples of the committed `assets/lunar_hfield.png`, using the
row reversal and nearest-cell convention in `test_husky_mujoco_geometry.py`'s
`_hfield_elevation`. PNG values map to `value / 255 * 0.286 - 0.2095` metres.
Each datum is 4 cm below its centre sample. Habitat feet, array feet, the antenna
foundation, and the pad slab extend below the lowest terrain in their footprints.
The lander's feet meet the pad top at local z = 0.18 m. No ground asset is changed.

The southernmost habitat geometry is the threshold at y = 4.83 m. The pad reaches
y = 4.30 m but is farther east, with x >= 3.90 m. The recorded far corner before
adding the base is approximately 0.996, 2.170. The base leaves at least 2 m of
clear ground beyond a conservative 0.6 m rover radius around that corner and
stays outside the full recorded square.

A future navigation demo can start at the default keyframe, x = -0.30, y = 0.50,
and end near x = 0, y = 3.8, facing north toward the closed habitat door. That
approach point stops short of the threshold. It is a suggested goal, not a claim
that navigation or traversability over the existing craters has been validated.
