# so101_sim

A thin overlay of [`so101_base_config`](../so101_base_config/README.md) — the
robot description, `ros2_control` hardware interfaces, MoveIt configuration,
the wiggle-test bridge, and every Objective for a LeRobot SO-101 follower arm
all live there. This package does exactly one thing: it forces
`hardware_interface: "mock"` (`config/config.yaml`, via `based_on_package`),
so `so101_sim` always runs against `mock_components/GenericSystem`, a digital
twin driven by a joint source outside the Runtime — regardless of what
`so101_base_config`'s own default is set to. Everything else is inherited
unchanged.

Run it:

```bash
moveit_pro run --config so101_sim
```

Then run the **Mirror SO101 Follower** Objective (inherited from
`so101_base_config`): the twin starts moving.
`so101_arm_bridge.py` runs in `--fake` mode and publishes a sine on the
`joint_trajectory_controller` topic, which `mock_components/GenericSystem`
echoes back as `/joint_states`. Planning, teleoperation and the waypoint
Objectives work against that same twin. See
[`so101_base_config`'s README](../so101_base_config/README.md#what-is-here) for
what the wiggle test is and why it doubles as the real-hardware bring-up check.

## What is here

| Path | What it is |
|---|---|
| `config/config.yaml` | `based_on_package: so101_base_config`, overriding only `hardware_interface: "mock"`. Everything else — description, controllers, SRDF, joint limits, waypoints, Objectives, the runtime launch file and its bridge node — is inherited unchanged. |

## Real hardware

See [`so101_base_config`](../so101_base_config/README.md) for the Feetech
STS3215 bus bench procedure, calibration, known driver gaps, torque lifecycle,
and safe bring-up order. This overlay's own `hardware_interface: "mock"`
override has to be reverted (or run `so101_base_config` directly, or a
real-hardware config that inherits it) to reach real hardware — `so101_sim`
always forces mock. The inherited waypoints are sim-picked, not bench-taught;
see
[Waypoints are not taught poses](../so101_base_config/README.md#waypoints-are-not-taught-poses).
