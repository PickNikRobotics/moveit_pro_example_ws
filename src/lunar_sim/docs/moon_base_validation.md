# Moon base validation

![Moon base from a chase view and from the centre looking south over the band](moon_base_render_sheet.png)

The sheet uses two offline inspection cameras. The configured scene cameras are unchanged. The base has ten fixed bodies and 68 visible collision geoms.

## Recorded-state comparison

Measured on the scene without the current base layout and rocks, then repeated with them, using `validate_and_render.py` with MuJoCo 3.6.0. Both runs include the rover's IMU body, which changes this open-loop route; record a new baseline whenever the rover's mass model changes.

| Measurement | Before | After |
| --- | --- | --- |
| Closure error, m | 1.28100276146052 | 1.28100276146052 |
| Turn 1, degrees | 117.044049304336 | 117.044049304336 |
| Turn 2, degrees | 94.2037099372136 | 94.2037099372136 |
| Turn 3, degrees | 88.0877338907876 | 88.0877338907876 |
| Turn 4, degrees | 69.45411711271146 | 69.45411711271146 |

These are chassis `xpos` and `xquat` measurements, not wheel odometry. The current
keyframe starts on the heightfield at -0.30, 0.50 m. Historical numbers elsewhere
in the package README differ from this measurement. The base preserves the current
route's errors; it does not retune the skid-steer calibration.

`validate_moon_base.py` also checks that all ten bodies are fixed, collision geometry belongs to rendered groups, body elevations match the committed heightmaps, and the base is outside the route footprint and inside the drivable area. The minimum far-corner clearance is **2.22912 m beyond a 0.6 m rover radius**, using conservative world-aligned geom bounding boxes. Rays from the start corner at 1.2 m height hit the door, solar mast panel, lander payload, and rover shelter with hidden geom groups disabled.

## Reproduce

The package build passed on MoveIt Pro `10.1.0-rc9` with Jazzy on 2026-09-14:

```bash
moveit_pro build user_workspace -w "$PWD" --colcon-args '--packages-up-to lunar_sim'
```

All three selected packages finished: `clearpath_platform_description`,
`lunar_sim_behaviors`, and `lunar_sim`. An isolated GPU-backed deployment reached
healthy driver and runtime states, with `/health` reporting `Idle`. The Desktop
App's live `/scene_camera/color` feed showed the habitat, solar array, pad, and
lander alongside the rover, satisfying the live launch check. That check was recorded on the
earlier layout, before the solar array moved out to the band.

The separate 3D point-cloud display has a pre-existing unconnected-tree error
between `scene_camera_optical_frame` and `world`, outside this change's scope.

Run with MuJoCo, numpy, and Pillow available, in `description/`. The output
directory must exist for `validate_and_render.py`.

```bash
MUJOCO_GL=egl python3 validate_and_render.py husky_scene.xml /out/moon_base
MUJOCO_GL=egl python3 validate_moon_base.py husky_scene.xml /out \
  --baseline ../docs/moon_base_baseline.json \
  --calibration /out/moon_base_calibration.json
```

Use the same simulator version for before and after comparisons. When changing
the underlying rover or terrain, first record a new unchanged-scene baseline
instead of attributing that change to the moon base.
