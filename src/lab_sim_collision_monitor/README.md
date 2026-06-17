# lab_sim_collision_monitor

Usage example showing how to configure the vendor-neutral **collision monitor**
node for the `lab_sim` robot (a UR arm on a linear rail).

> **This package does not contain the collision monitor.** The monitor is a
> separately licensed product, shipped as a pre-bundled binary image/overlay.
> This package contains only the robot-specific glue — launch, params, and an
> integration test — that *consumes* it. Running the example requires a valid
> `MOVEIT_LICENSE_KEY`; without one the monitor's engine constructor fails
> loudly by design.

## What the monitor does

Each tick (~100 Hz here) the monitor forward-projects the arm to its
max-effort braking-stop configuration and collision-checks that stopping
configuration. If a clean stop can no longer be guaranteed it trips. For
`lab_sim`, with the generic node's empty collision world, the collisions it
sees are **self-collisions** (robot-link vs robot-link, per the SRDF ACM).

### Wiring (all robot specifics live here, not in the monitor core)

- **StateProvider** ← `/joint_states`. `lab_sim` reports driving joint angles
  (rad) and the rail position (m) directly, in model-DOF order, so no
  actuator→joint conversion is needed. (If a robot reported actuator units, the
  conversion would live in a Layer-C adapter that republishes `/joint_states`,
  never in the monitor core.)
- **Robot model** ← `/robot_description` + `/robot_description_semantic`
  (the SRDF supplies the allowed-collision matrix).
- **TripSink** → a `lab_sim` stop. The production stop action is a controller
  halt: deactivate `joint_trajectory_controller` via
  `/controller_manager/switch_controller`. The integration test demonstrates
  this on the first trip.
- **Outputs**: `…/collision_monitor/trip` (`std_msgs/String`, one per trip with
  reason + contacts) and `…/collision_monitor/status` (`std_msgs/Bool`, latched
  engine state each tick).

Parameters are in [`config/lab_sim_collision_monitor.yaml`](config/lab_sim_collision_monitor.yaml);
notably `default_accel_limit` is set to the lab_sim arm's 30 deg/s² braking
spec (the 6 UR joints carry no URDF acceleration attribute).

## Run it locally

The monitor must run **in parallel** with the lab_sim MoveIt Pro backend, which
supplies `/robot_description(_semantic)` and `/joint_states`. The integration
test does exactly this.

```bash
# From the example_ws root, inside the MoveIt Pro dev container, with a valid
# MOVEIT_LICENSE_KEY in the environment (example_ws ships one in .env) AND the
# licensed monitor bundle present on the overlay:
export MOVEIT_LICENSE_KEY=...        # see .env
colcon build --packages-select lab_sim_collision_monitor
colcon test --packages-select lab_sim_collision_monitor \
    --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

To launch only the monitor (against an already-running lab_sim backend):

```bash
ros2 launch lab_sim_collision_monitor collision_monitor.launch.py
```

## The integration test (Layer 2)

`test/collision_monitor_integration_test.py` brings up lab_sim + the monitor and
asserts **both** directions:

1. **No false trips** while the safe `Move Along Square` objective runs.
2. A **deliberate self-collision** pose (commanded straight to the
   `joint_trajectory_controller`, bypassing planning) **must** trip the monitor
   with a `PROJECTED_COLLISION`.

`test_monitor_becomes_active` doubles as the **license gate**: if
`MOVEIT_LICENSE_KEY` is missing/invalid the monitor node dies on init, `status`
never publishes, and the test fails — the example refuses to run unlicensed.

## CI

`.github/workflows/collision-monitor-integration.yaml` (downstream, in this
repo) runs this test against a pre-bundled licensed monitor image. It is
triggered by `repository_dispatch` (`collision-monitor-integration`) from the
monitor's upstream deploy pipeline, or manually via `workflow_dispatch`. It
**pulls** the licensed bundle image and never clones or builds monitor source.
