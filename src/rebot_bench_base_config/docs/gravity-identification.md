# Static gravity data collection

**Collect reBot Gravity Data** is a plain sequence of the standard **Move to
Waypoint**, followed by settling and recording waits. It visits **Gravity 01**
through **Gravity 48** from `waypoints/rebot_waypoints.yaml`, between the existing
Raised and HOME waypoints. Edit those named waypoints and the sequence in the
normal Desktop editor; there is no custom motion generator, endpoint bookkeeping,
scripted pre-check, or approval-string port.

The Objective collects data only. It does not fit or activate a model, change
gravity/impedance gains, or disable motors. Static data cannot identify a full
dynamic mass matrix or rotational inertia.

## Before a physical run

Obtain explicit physical-run approval. Start at measured and physical **HOME**,
empty jaw closed, normal configured impedance and unchanged gravity settings.
Keep other trajectory, leader and teleoperation publishers idle. The arm is
brakeless: HOME before any disable or restart remains mandatory.

Remove cubes and loose objects physically and remove only stale cube collision
objects using normal scene tools. The Objective adds **reBot Fixed Bench Scene**,
the same mat/table and foam-wall boxes used by Stack, without adding cubes or
clearing any scene objects. Native planning and collision checking remain enabled.
Inspect wrist, camera and cable clearance; they are not completely collision-modeled.

Ensure `/joint_states` contains finite measured effort for joint1–joint7 alongside
position/velocity. The real Xacro declares the effort state, and the joint state
broadcaster includes `effort`; no driver change is needed. Verify healthy, current
motor feedback and the existing thermal guard through the normal bench preflight.
These are operator prerequisites, not duplicated Objective pre-checks.

## Motion, waits and recording

The standard **Move to Waypoint** supplies planning, collision checking, controller
selection and execution. This release aliases it to **Move to Named Waypoint**.
Its normal velocity/acceleration defaults are **1.0**, using the configured arm
limits of 0.6 rad/s and 1.2 rad/s². No reduced first-run scaling or custom speed
guard is used. Standard collision padding and planner checks are not overridden.
The manipulator group excludes the jaw, which stays closed under its existing
controller. Joint7 is recorded but is not fitted.

Each measurement waypoint is visited once. The default waits are **1.5 s to
settle**, then **2.1 s of static recording**, enough for a one-second averaging
window and one retry. A standard LogMessage marks each static window in `/rosout`
for offline extraction; it does not gate or change motion. The 53 moves are
Raised → Gravity 01…33 → Gravity Elbow Clearance → Gravity 34…48 → Gravity
Retreat → Raised → Zeroed Rest, starting from HOME. The two extra transit
waypoints fold the elbow before sweeping the wrist and lower the shoulder
before unfolding on return. This simplified route does **not** create opposing
approach pairs; do not describe its future data as friction-averaged paired data.
Keep any earlier paired dataset and its generating Objective separately.

`RecordEpisode`, `SaveEpisode` and `StopRecording` wrap the whole successful run.
The existing `rebot_bench_jpeg` Trainer config uses its already configured camera
streams and captures raw joint/controller messages. Training fps is a conversion
setting, not a throttle on raw MCAP messages. Keep the raw MCAP, not a converted
position-only LeRobot dataset.

Use a unique `dataset_name` in the Desktop. After physical-run approval, the
supported action equivalent is:

```bash
ros2 action send_goal /do_objective moveit_studio_sdk_msgs/action/DoObjectiveSequence \
"{objective_name: 'Collect reBot Gravity Data', parameter_overrides: [{description: {name: 'dataset_name', type: 0}, string_value: 'rebot_gravity_waypoints_01'}], caller: {type: 3, name: 'gravity_collection'}}" --feedback
```

For motor health and whole-run evidence retained even after cancellation, start
this separate read-only ROS recorder first from the normal Runtime shell:

```bash
ros2 bag record --node-name gravity_health_recorder -s mcap -o ~/gravity_waypoints_health_01 \
  /joint_states /robstride_hardware_interface/robstride_state \
  /robstride_hardware_interface/thermal_tripped /rosout \
  /joint_trajectory_admittance_controller/controller_state
```

After the Objective finishes, stop only this recorder with Ctrl-C and archive
both bags with the waypoint file, Objective and expanded robot description.
Never use a second CAN owner or vendor CLI to obtain these measurements.

## Offline extraction and torque provenance

Motor effort is decoded from complete RobStride Type-2 feedback bytes4–5 using
each model's Nm range (RS06 ±36Nm, RS00 ±14Nm). The driver publishes
`direction * measured_torque`; this robot uses direction+1 and URDF axes encode
joint signs. It is motor-reported torque, not commanded feedforward or a calibrated
external torque measurement. Map JointState arrays by name.

Fit measured positions, not waypoint angles. After settling, average one second
at about100Hz. Reject bit-identical effort, nonzero `comm_state`, nonfinite/missing
samples, or position range above0.003rad; retry once from the next one-second
window, then skip the pose. ROS stamps re-stamp cached driver feedback, so this
heuristic cannot establish complete CAN freshness and can reject truly constant
quantized torque. A proper source-freshness export is a separate driver follow-up.
RobStride health headers can use a steady clock while JointState uses ROS epoch;
align bag receive times rather than comparing incompatible header clocks.

## Failure, stop and thermal handling

A failed standard move stops the sequence. Cancellation uses native motion
cancellation. Downstream HOME, SaveEpisode and StopRecording are **not guaranteed
to run after failure or halt**. Keep the separate raw bag, save/stop the Trainer
through its normal controls, and use existing supervised HOME recovery. A hardware
fault or joint-loss freeze can prevent HOME; a failure result is never proof of it.

The existing **100°C** cutoff and thermal guard remain unchanged. Its established
cancel → HOME → fresh stationary-HOME verification → OFF handling is external to
this sequence. Never disable away from proven HOME or bypass a native planning,
fault, or thermal refusal.

## Offline checks

`test/test_gravity_collection.py` loads the configured model and real collision
meshes. It verifies the shared fixed scene, 48 named waypoint states, candidate
transit/HOME paths at0.005rad spacing, native full-speed waypoint timing, and the
plain sequence contract. Standard 1cm environment padding is included. A known
self-collision canary detects missing mesh resources.

These discrete checks establish model-space candidate paths, not every path the
runtime planner might select or physical tracking/clearance. Standard Move to
Waypoint collision-checks its actual plan against the current scene. No twin or
hardware run is needed for the offline checks. Fit and calibration application
remain separate from this reusable collection Objective.

## Leaned-back coverage and torque limits

Gravity 25…48 extend J2 over six shoulder levels **1.3, 1.6, 1.9, 2.15, 2.3 and
2.4 rad**, with J3 spanning **1.2332…3.7851 rad** and J4 **−1.3712…0.9768 rad**.
J5/J6 remain within the earlier cable/wrist envelope. All targets and candidate
transits were checked with the same bench/wall and standard 1cm padding.

These are **powered data-collection poses, not approved float poses**. The prior
fit extrapolates beyond its training range. Its predicted J2 gravity exceeds the
existing9Nm feedforward clamp at Gravity28,34,37,38,39,41,42,43,45,46,47, and at
both added transits. Predicted J3 gravity stays below9Nm. J2 at several targets
also exceeds the RS06 vendor's11Nm rated load; that rating is specified at100rpm,
not proven indefinite stall capacity. Do not raise a clamp or claim continuous
float support merely because a pose is collision-free. See the [per-pose torque preview](gravity-torque-preview.md) for all new
poses and clamp flags. No clamp, gains,
thermal limits or driver behavior are changed here.

## Refit multiple recordings in one command

The offline `scripts/fit_gravity.py` reads closed MCAP bags, never ROS services
or command topics. Repeat `--dataset` for each Trainer bag and matching raw
health bag. Give the original nominal-model recording first. Example in the
normal sourced environment (replace the four bag paths):

```bash
ros2 run rebot_bench_base_config fit_gravity.py \
  --dataset /data/original_recording /data/original_health \
  --dataset /data/leaned_back_recording /data/leaned_back_health \
  --output /data/gravity_refit_01
```

The output directory must not already exist. It writes `gravity_calibrated.urdf`,
per-dataset raw arrays/window audits, recorded model snapshots, and
`fit_report.json` with nominal/fitted residuals on training and held-out poses.
Every fifth waypoint ID is held out across **all** runs to avoid same-pose
leakage; held-out observations are not refit into the delivered model. The
original opposing-approach markers and the new named-waypoint markers are both
supported. New single-approach observations are not claimed to cancel friction.

The regressor fits identifiable static first-moment combinations, retaining
nominal link masses, rotational inertia tensors and unobservable components.
The URDF's equivalent CoMs need not be physically inside their links and are not
unique mass/CoM identification. Inspect residuals/rank/conditioning, the input
model association and clamp requirements before any separate HOME-only adoption.
Different chain or fixed attachment geometry is rejected; controller gains are
never fitted, divided out, forced to1, or activated by this tool.
