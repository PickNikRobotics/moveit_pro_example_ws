# Lower-limit commissioning — 2026-10-14

The operating lower bounds are **J2 −0.02 rad, J3 −0.01 rad, J7 0 rad**.
They provide conservative home margins for planning; they are not measured
negative hard stops or a guarantee that every state within bounds is
collision-free. The URDF, explicit MoveIt position overrides and MuJoCo twin
use these same bounds. J7's dependent sliders retain zero lower bounds and
the existing 0.008 m/rad coupling. Upper bounds and speed limits are unchanged.

## Method and observations

Native joint-space Objectives planned each leg from commanded arm zero,
anchored the trajectory to the controller command, and kept trajectory
collision validation enabled, under a temporary −0.10 rad planning lower
bound on J2, J3 and J7. Each executed batch changed J2 alone, returned
the command to zero, and confirmed home before disabling all seven motors.
Reference speed was at most 0.02 rad/s. Scene and wrist frames were captured
before and after; the desk camera was unavailable. Joint-state and controller
telemetry were recorded independently of the Objective result.

| Trial | Commanded minimum (rad) | Measured minimum (rad) | Observation |
| --- | --- | --- | --- |
| J2 first step | −0.020 | −0.010930 | Completed; no contemporaneous torque stream saved |
| J2 second step | −0.040 | −0.015915 | Completed; sparse torque observation about 0.169 Nm |
| J2 third step | −0.060 | −0.020517 | Torque concern; further negative steps stopped |
| J3 first step | −0.020 | No negative travel measured | Collision validation refused the path before execution |
| J7 | No negative command sent | No negative clearance measured | Zero lower bound retained |

The J2 third trial's torque observations rose in magnitude to approximately
6.455 Nm. Responses accumulated before inspection; the concern was seen after
the normal return had completed. A subsequent cancellation found no active
goal. This does not demonstrate cancellation recovery or establish that the
measured extreme was contact-free. No −0.080 or −0.100 rad trial was executed.
J3 validation reported link2–link5 collision at 23.8% of the −0.020 rad path;
the accepted −0.010 rad bound was not physically commissioned.

The impedance controller has no gravity feedforward, so command success and
under-tracking do not establish measured endpoint clearance. For example,
J2's first trial had about 0.00907 rad command-to-measurement discrepancy,
larger than the approximately 0.000384 rad encoder quantization. The selected
J2 bound is only about 0.000517 rad above the deepest measured excursion,
which occurred during the torque concern; this is not a validated safety
reserve. J3's bound is a selected home allowance without a successful negative
trial. No uncertainty budget establishes a physical clearance margin for
either joint. Keep collision validation and independent hardware safeguards.
Offline model tests verify the source contract; they do not certify negative
physical clearance.

J2 started at 30 °C and peaked at 32 °C over these trials; all motor peaks
were at most 33 °C. Recorded fault and communication counts were zero.
The last checkpoint confirmed home with all motors disabled. No further
limit tests were authorized after the final bounds were selected.
