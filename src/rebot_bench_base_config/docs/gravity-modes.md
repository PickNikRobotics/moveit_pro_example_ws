# Spring and gravity-compensation modes

These mode defaults target the **existing physical KDL gravity-only integration**:
`gravity_compensation_controller`, J1–J6 effort command interfaces alongside the
trajectory controller's position interfaces, and the fitted gravity model. They
are not a standalone mock/simulation configuration or a replacement driver.
Preserve that integration, its approved torque clamps, and its fitted inertials
when adopting the mode files; do not overwrite them with an older description.

Physical launch starts in `arm_control_mode: motion`: spring/MIT with configured
kp/kd, not the motor's `position_csp` loop. Both
`joint_trajectory_admittance_controller` (JTAC) and `gravity_compensation_controller`
start active, with gravity gains **`[0,1,1,1,0,0]`** and zero offsets. Startup is
**not float**. Gravity stays on during spring control, ordinary motion and float.
Selecting real hardware, building into the deployment and restarting remain
HOME-only operations.

## Two short Objectives

Both use existing `GetCurrentPlanningScene` → `GetJointState` →
`CreateStationaryTrajectory` → `ExecuteTrajectory` to re-anchor all seven JTAC
joints to measured position. The hold has two identical points at 0/0.2 s, zero
velocity/acceleration, and the standard execution pipeline's nonzero dispatch
stamp. The trajectory must succeed **before any gain request**.

- **Start Gravity Comp** then writes J1–J6 kd `[1.5,3.5,5,1.25,1,1]`, followed
  by kp zero. It **finishes while the arm remains floating**.
- **Stop Gravity Comp** then restores J1–J6 kd `[3,10,10,5,4,4]`, followed by
  kp `[50,150,150,50,50,50]`. It finishes in spring mode at the newly anchored
  pose, with gravity still on at `[0,1,1,1,0,0]`.

Neither Objective switches a controller, writes gravity parameters, goes HOME,
disables torque or installs a monitor. JTAC must already be active and remains
active throughout float. Controller reactivation alone is **not** a substitute
for the explicit anchor: the installed JTAC can retain old command positions
when activated. Stop therefore always makes a fresh anchor.

The jaw's gains are never changed. Stop's values match the arm GPIO parameters
in `description/rebot.urdf.xacro`; keep them aligned if that configuration changes.
Each `SetRebotMotorGain` write is a standard asynchronous service request with a
0.5 s response bound. A response acknowledges the driver's pending value, not
independent motor readback; the twelve writes are not atomic.

Run these by name from the Desktop Objective list only after physical use is
approved and the thermal dependency below is satisfied. Support the arm and keep
it stationary during either switch. The measured pose is a snapshot, not a
continuously tracked hand-guiding target. Ordinary trajectory tolerances apply.
An anchor refusal stops before gain writes; there is no custom precheck, fallback,
automatic retry or halt restoration. If interrupted after some writes, partial
settings remain: support the arm and inspect before an owner-directed Stop.
Canceling an already completed Start cannot stop floating. Do not run another
motion, jog or leader Objective while floating.

## Thermal adoption dependency

**Do not use these on hardware with the older thermal guard.** On **every** native
100 C latch the updated `rebot_thermal_guard.py` must first explicitly re-anchor
JTAC to measured position, then restore configured kp/kd through the existing
driver gain service, **before** its existing cancel → HOME → verify → torque-OFF
flow. That guard update and its real-ControllerManager anchor-before-kp proof
are separate from these Objective/config files. No driver modification is
required or supplied by the mode switches. The previous proposal to restore
stiffness inside the driver while cycling JTAC is not this design.

The guard's owner must complete and verify that update at HOME before adoption.
Neither Objective starts a 90 C monitor. The earlier attended float trial owned
one only while its separate process ran; there is no always-on 90 C monitor in
these mode files. No new monitor or safety-rated protection is claimed.

The fitted model estimates static gravity, not dynamics or guaranteed hands-off
balance. Deep leaned-back poses can overheat J2 rapidly. Planning-scene checks
and position limits do not physically constrain a hand-pushed kp=0 arm.
Transport loss, native faults, competing targets or process death can prevent
restoration. Retain the native 100 C cutoff and reach verified HOME before any
disable; a successful mode switch is not a recovery guarantee.
