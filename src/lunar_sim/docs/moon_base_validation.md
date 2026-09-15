# Moon base validation

![Moon base from a chase view and the route start corner](moon_base_render_sheet.png)

The sheet uses two offline inspection cameras. The configured scene cameras are
unchanged. The base has five fixed bodies and 35 visible collision geoms.

## Recorded-state comparison

Measured on the unchanged scene at commit `106bc37f`, then repeated after adding
the moon base, using `validate_and_render.py` in
`picknikciuser/moveit-pro:main-jazzy-amd64-cuda13.2-cudnn9`.

| Measurement | Before | After |
| --- | --- | --- |
| Closure error, m | 0.6289748929625518 | 0.6289748929625518 |
| Turn 1, degrees | 112.41969736142369 | 112.41969736142369 |
| Turn 2, degrees | 89.71613637558676 | 89.71613637558676 |
| Turn 3, degrees | 94.06058646662831 | 94.06058646662831 |
| Turn 4, degrees | 41.21809188541886 | 41.21809188541886 |

These are chassis `xpos` and `xquat` measurements, not wheel odometry. The current
keyframe starts on the heightfield at -0.30, 0.50 m. Historical numbers elsewhere
in the package README differ from this measurement. The base preserves the current
route's errors; it does not retune the skid-steer calibration.

`validate_moon_base.py` also checks that all five bodies are fixed, collision
geometry belongs to rendered groups, body elevations match the committed PNG,
and the base is outside the route footprint and inside the terrain boundary.
The minimum far-corner clearance is **2.08255 m beyond a 0.6 m rover radius**,
using conservative world-aligned geom bounding boxes. Rays from the start corner
at 1.2 m height hit the door, solar panel, antenna panel, and lander payload with
hidden geom groups disabled.

## Reproduce

The package build passed on MoveIt Pro `10.1.0-rc9` with Jazzy on 2026-09-14:

```bash
moveit_pro build user_workspace -w "$PWD" --colcon-args '--packages-up-to lunar_sim'
```

All three selected packages finished: `clearpath_platform_description`,
`lunar_sim_behaviors`, and `lunar_sim`. An isolated GPU-backed deployment reached
healthy driver and runtime states, with `/health` reporting `Idle`. The Desktop
App's live `/scene_camera/color` feed showed the habitat, solar array, pad, and
lander alongside the rover, satisfying the live launch check.

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
