# Norma Core parallel gripper assets

Source: [`norma-core/norma-core`](https://github.com/norma-core/norma-core/tree/dea97c596aa5e3833c26586ec3edf74cd717144a/hardware/pgripper)

Upstream commit: `dea97c596aa5e3833c26586ec3edf74cd717144a`

License: Apache License 2.0; see `LICENSE` in this directory.

The following files are unmodified upstream STL exports, renamed for stable URL use:

- `gripper_base.stl`
- `gripper_base_shield.stl`
- `gripper_gear.stl`
- `gripper_jaw.stl`
- `wiring_bracket.stl`
- `camera_mount_square_27mm.stl`
- `reference_u20.png`

The following files are derived from upstream `STEP/Gripper.stp` by triangulating
the named assembly components and transforming their vertices into the SO-101
wrist-horn frame used by this MJCF:

- `norma_u20_camera.stl`
- `norma_st3215_servo.stl`
- `norma_gripper_hardware.stl`

The wrist-horn frame maps the upstream assembled coordinates as follows:

```text
SO-101 X = -(upstream Y - horn Y)
SO-101 Y =  (upstream Z - horn Z)
SO-101 Z = -(upstream X - horn X)
horn = (24.7807, -2.5394, 62.7769) mm
```

The supplied real-robot witness image has SHA-256
`c0c1b4c1347489d99a0f28aad53c9106975a609ca02119304a8a835af5ed964d`,
identical to upstream `images/cameras/u20.png`. That establishes the 27 mm U20
camera mount as the correct variant.
