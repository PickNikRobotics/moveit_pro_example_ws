# Blue-on-red stack validation

**Five distinct placements passed with the explicitly enabled SIM-ONLY grasp
proxy. Physical grasp fidelity is not established. No real-arm motion was run.**

The five placements and the final repeat are historical: they ran on an earlier
twin whose robot base carried the calibration's full 6-DoF solve (z +4.7 mm,
roll -4.0 deg, pitch 1.8 deg) with the nominal wrist-camera mount, and with an
earlier grasp proxy that armed the weld from the gripper command alone. Each
directory's `bench_layout.yaml` records the configuration it actually ran. The
shipped twin uses the level base and measured wrist mount of the ChArUco
calibration and a proxy that verifies contact itself; see the
[level-base far sanity run](#level-base-far-sanity-run).

All poses are MuJoCo ground truth in the printed `world` frame, in metres.
The Objective uses SAM3 RGB masks and calibrated TF; it never reads this truth.
The pass condition requires the released blue cube to remain within 5 mm of
red horizontally, a 50 mm centre-height difference within 5 mm, both cubes
within 5 degrees of upright, and less than 1 mm motion in the final two seconds.
Each run also completed its retreat and returned `SUCCEEDED`.

| Placement | Blue start XY (m) | Red start XY (m) | Final XY error (mm) | Height difference (mm) | Max tilt (deg) |
| --- | --- | --- | --- | --- | --- |
| 1: Far original layout | 0.317, 0.051 | 0.288, 0.148 | 4.998 | 49.888 | 0.098 |
| 2: Mid-reach | 0.360, 0.060 | 0.400, 0.200 | 4.106 | 49.884 | 0.116 |
| 3: Red and blue swapped | 0.400, 0.200 | 0.360, 0.060 | 4.750 | 49.840 | 0.071 |
| 4: Near wall: vertical fallback | 0.460, 0.100 | 0.400, 0.260 | 2.876 | 49.892 | 0.053 |
| 5: Front edge and side | 0.350, 0.160 | 0.310, 0.020 | 2.879 | 49.886 | 0.098 |

The far placement has only 0.002 mm margin to the horizontal threshold at the
recorded end. These five results are a bounded validation set, not a claim of
repeatability or full workspace coverage. The unchanged tilted route failed
near the wall (`link4` collision); a collision-checked vertical hover succeeded.
The durable tree now selects that fallback, with no manual motion mid-run.

After the release-failure retreat handler was added, the final Objective passed
an additional full run at the original far placement: 4.889 mm horizontal error,
49.890 mm height difference, 0.081 degrees maximum tilt. Its raw truth and
Objective result are in [`final_default/`](final_default/). This confirms the
final control flow on the far route; it does not widen the five-placement claim.

## Level-base far sanity run

After moving to the level robot base (x, y and yaw from the hand-eye solve,
roll and pitch 0, z -3 mm on the bench top) and the measured wrist-camera mount,
the far placement was run once more as a sanity check, from a build of the
fixed source with the original layout unchanged. It passed: 4.827 mm horizontal
error, 49.888 mm height difference, 0.093 degrees maximum tilt, under 0.03 mm
motion in the final two seconds, Objective `SUCCEEDED` in 127.9 s. The grasp
proxy welded once, at finger contact, and released once, on opening. This run
also used the measured-jaw release-failure retreat, whose failure branch it did
not exercise, and invoked **Stack Blue on Red** with a `DoObjectiveSequence`
goal on `/do_objective`, the action MCP `run_objective` submits. An attempt
just before it, in the same deployment, failed at Behavior Tree creation, before
any motion, on a SubTree port declared without a default; that was fixed
before this run. One run with 0.17 mm of horizontal margin is a sanity check,
not a repeatability or workspace claim. Its raw truth, renders and result are in
[`level_base_far/`](level_base_far/).

| Level-base far placement | Before | After |
| --- | --- | --- |
| Blue 0.317, 0.051; red 0.288, 0.148 | ![Before, level base](level_base_far/before.png) | ![After, level base](level_base_far/after.png) |

None of these twin runs used the later hardware corrections, such as opening the
jaw at home first, the per-leg speed settings file or anchoring each leg to the
previous command. Stack Blue on Red with those corrections was run in the twin
only as the stack leg of the episode cycle; see
[Unstack and cycle twin runs](../../README.md#unstack-and-cycle-twin-runs).

## Labelled render pairs

| Placement | Before | After |
| --- | --- | --- |
| 1: Far original layout | ![Before 1](placement01/before.png) | ![After 1](placement01/after.png) |
| 2: Mid-reach | ![Before 2](placement02/before.png) | ![After 2](placement02/after.png) |
| 3: Red and blue swapped | ![Before 3](placement03/before.png) | ![After 3](placement03/after.png) |
| 4: Near wall: vertical fallback | ![Before 4](placement04/before.png) | ![After 4](placement04/after.png) |
| 5: Front edge and side | ![Before 5](placement05/before.png) | ![After 5](placement05/after.png) |

## Continuous video and raw evidence

[Full far-placement run, continuous 30 fps H.264 video](placement01/run.mp4):
4,573 live frames, 152.433 seconds, encoded at 30/1 fps. Source timestamps report
30.0003 fps with a maximum 33.9 ms inter-frame gap. This is a live camera stream,
not stitched screenshots. `record_stack_evidence.py` starts before the Objective
and finalizes on a stop file or bounded duration.

Each placement directory contains the layout used for its rebuild, raw
`qpos.jsonl.gz`, the simulator schema hash, before/after renders, `result.json`,
and the ordinary MCP Objective result. The result explicitly identifies the
proxy. The weld latches at actual finger contact and releases on opening; no
pose or orientation is teleported for the pass. The strict expected-failure
physical lift regression runs with the weld disabled. Representative failed
physics trials and the original rejected wall approach remain in
[`failed_attempts/`](failed_attempts/).

## Build and execution

MoveIt Pro PR 22998 was built at
`329427726d3bbb408311bef3298c75bd43482e50`, with one local-only patch removing
the Direct Motion server's hardcoded 10 mm padding floor so it honors the
configured scene padding. That issue and suggested patch were posted on
`PickNikRobotics/moveit_pro` PR 22998 (private repository).
All seven Direct Motion tools were exercised through host-user stdio MCP.
The final runs use the ordinary `run_objective` tool only.

For each scatter placement: edit the cube poses in `config/bench_layout.yaml`,
rebuild `rebot_bench_sim`, restart the twin, wait for the recorder's `READY`,
then invoke **Stack Blue on Red**. Do not regenerate the Objective per run.
The shared packages are `rebot_bench_behaviors` and `rebot_bench_base_config`;
a user-workspace colcon build and backend restart deploy them on an existing
MAIN-based MoveIt Pro runtime. Direct Motion is not required there. See the
[hardware-independent run recipe](../../../rebot_bench_base_config/README.md#sam3-blue-on-red-stack).

## Automated checks

Alongside the level-base run, model, configuration and grasp-proxy checks pass,
with the physical 50 mm lift regression retained as one strict expected failure.
The grasp-proxy regression drives the node through its topics: a closed gripper
command without finger contact never welds, and a simulation reset releases a
held cube. The behavior library's C++ tests pass, including holding a measured
jaw. The shipped Objective trees run in BT.CPP with stubbed motion: the jaw
opens at home first and a short opening is retried once, then fails before the
survey; release success, a release opening that fails or stays short after its
retry (retreating with the measured jaw) and detach failure all return home
with the failure preserved; a rejected retreat stops; and every leg plans at its
kind's factors from the settings file. The
integration suite skips the stateful stack entry points.

```sh
colcon build --packages-up-to rebot_bench_behaviors rebot_bench_base_config rebot_bench_sim
MOVEIT_CONFIG_PACKAGE=rebot_bench_sim colcon test --packages-select rebot_bench_behaviors rebot_bench_base_config rebot_bench_sim --ctest-args -E objectives_integration_test
colcon test-result --all --verbose
```
