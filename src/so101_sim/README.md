# so101_sim

A thin overlay of [`so101_base_config`](../so101_base_config/README.md) — the
robot description, `ros2_control` hardware interfaces, and MoveIt
configuration for a LeRobot SO-101 follower arm all live there. This package
only:

- forces `hardware_interface: "mock"` (`config/config.yaml`, via
  `based_on_package`), so `so101_sim` always runs against
  `mock_components/GenericSystem`, a digital twin driven by a joint source
  outside the Runtime — regardless of what `so101_base_config`'s own default
  is set to;
- adds the wiggle-test bridge (`script/so101_arm_bridge.py`) and its
  `Mirror SO101 Follower` Objective, which feed the twin so it moves with no
  arm plugged in.

Run it:

```bash
moveit_pro run --config so101_sim
```

Then run the **Mirror SO101 Follower** Objective: the twin starts moving.
`script/so101_arm_bridge.py` runs in `--fake` mode and publishes a sine on the
`joint_trajectory_controller` topic, which `mock_components/GenericSystem`
echoes back as `/joint_states`. Planning, teleoperation and the waypoint
Objectives — all inherited from `so101_base_config` — work against that same
twin.

The sine is a **wiggle test**, not a sweep: each joint swings
`wiggle_amplitude_rad` (0.1 rad, about 6°, by default) either side of the pose
the arm is measured in when mirroring starts, read once from `/joint_states`.
That makes it the same joint-by-joint "is everything alive and moving the right
way" diagnostic as `lab_sim`'s, and it is the reason the Objective is safe to
run on the powered follower as well as on mock — see
[`so101_base_config`'s bench procedure](../so101_base_config/README.md#bench-procedure)
for running this bridge against real hardware. Raise `wiggle_amplitude_rad`
for a livelier sim demo; the amplitude is clamped to the URDF joint limits, so
a joint already parked on a limit is never commanded past it.

Mirroring will not start until a **complete** `JointState` has arrived — one
carrying all six joints of `JOINT_NAMES`. A message naming only a subset is
ignored rather than partially applied, because a centre assembled from a
partial pose would put the missing joints at whatever the last full sample
said. The sample must also be fresh: older than `joint_states_timeout_s`
(2.0 s by default) and the `~/mirror` tick returns failure instead of centring
on a stale pose, since a stale sample means the broadcaster died or the bus
went quiet. With no usable pose there is no safe centre, and the tick fails
rather than guessing.

Mirroring is off until that Objective asks for it, and stops when the Objective
is stopped. The trajectory controller has one owner at a time: a stream of topic
messages restarts its trajectory on every tick, so a plan's goal would be
accepted and then hang forever, or abort on a path tolerance the moving twin
violated. The Objective keeps mirroring alive by ticking the bridge's
`~/mirror` `Trigger` service in a loop; one second without a tick and the bridge
goes quiet. That heartbeat gate is the primary guard: **stop the Mirror
Objective before planning or executing a motion.** The bridge also skips a
publish while it believes a `follow_joint_trajectory` goal is live, but that is
best-effort only — the flag is set from `GoalStatusArray` messages, so a goal
started while the Mirror Objective is still publishing can lose the race with a
20 ms bridge tick.

## What is here

| Path | What it is |
|---|---|
| `config/config.yaml` | `based_on_package: so101_base_config`, overriding only `hardware_interface: "mock"` and this package's own `runtime_launch_file` / `Mirror SO101 Follower` objective entry. Everything else — description, controllers, SRDF, joint limits, waypoints, the other Objectives — is inherited unchanged. |
| `script/so101_arm_bridge.py` | The wiggle-test joint source. `--fake` publishes a small sine about the measured pose; `--real` is still a stub. |
| `objectives/mirror_follower.xml` | `Mirror SO101 Follower`, the only Objective this package adds on top of the base config's `Move SO101 to Waypoint`, `Close Gripper`, `Open Gripper`, and `Teleoperate`. |
| `launch/runtime.launch.xml` | Overrides `so101_base_config`'s runtime launch file to also start the bridge node unconditionally. |

## Real hardware

See [`so101_base_config`](../so101_base_config/README.md) for the Feetech
STS3215 bus bench procedure, known driver gaps, torque lifecycle, and safe
bring-up order. This overlay's own `hardware_interface: "mock"` override has to
be reverted (or run `so101_base_config` directly, or a real-hardware config
that inherits it) to reach real hardware — `so101_sim` always forces mock.
The inherited waypoints are sim-picked, not bench-taught; see
[Waypoints are not taught poses](../so101_base_config/README.md#waypoints-are-not-taught-poses).
