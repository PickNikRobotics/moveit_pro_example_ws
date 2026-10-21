# reBot motor firmware settings

This is the commissioned B601-RS bench profile for position-mode leader
teleoperation. It describes motor parameter writes and nonvolatile saves, not
installing a firmware binary. Replacing motor firmware requires a separate,
model-specific procedure. These values belong to this arm and are not a general
RS00/RS06 safety rating.

## Joint settings and evidence

Motor IDs match joint numbers; the configured host ID is 253 (`0xFD`). J1–J3
are RS06 and J4–J7 are RS00. All positions, speeds and torques below are in the
motor frame. Keep the [configured joint limits](../README.md#coordinates-and-limits);
raising a firmware speed limit does not expand the allowed joint travel.

| Joint / motor ID | Model | Driver mode | Torque limit `0x700B` (Nm) | Speed limit `0x7017` (rad/s) | Current limit `0x7018` (A) | Zero configuration |
| --- | --- | --- | --- | --- | --- | --- |
| J1 / 1, base yaw | RS06 | `position_csp` (5) | 36 | 10 | 57 | Arm at commissioned HOME; `zero_sta=1` |
| J2 / 2, shoulder | RS06 | `position_csp` (5) | 36 | 10 | 57 | Arm at commissioned HOME; `zero_sta=1` |
| J3 / 3, elbow | RS06 | `position_csp` (5) | 36 | 10 | 57 | Arm at commissioned HOME; `zero_sta=1` |
| J4 / 4, wrist tilt | RS00 | `position_csp` (5) | 14 | **33** | **16** | Arm at commissioned HOME; `zero_sta=1` |
| J5 / 5, wrist yaw | RS00 | `position_csp` (5) | 14 | 33 | 16 | Arm at commissioned HOME; `zero_sta=1` |
| J6 / 6, wrist roll | RS00 | `position_csp` (5) | 14 | 33 | 16 | Arm at commissioned HOME; `zero_sta=1` |
| J7 / 7, jaw | RS00 | `motion` (0) | **1** | 33 | 16 | Jaw physically closed/touching; `zero_sta=1` |

The complete limits/`zero_sta` baseline was read on **2026-10-09**, after
the bench's zero-and-store procedure. J4 originally read 1 rad/s and J7 read
14 Nm. Those two settings changed on **2026-10-20**: later independent Type-17
readbacks confirmed J4 at 33 rad/s and J7 at 1 Nm. The J7 post-save snapshot also
confirmed 33 rad/s, 16 A, motion mode and `zero_sta=1`. The other baseline
settings were not all freshly re-read that day. Check the actual motor before
relying on an older value or configuring a replacement.

`zero_sta` is the byte register `0x7029`, selecting the startup coordinate
reporting convention; it is not the mechanical zero offset. The driver selects
`run_mode` (`0x7005`, signed byte) at activation. The table's modes describe the
intended running configuration, not evidence that those modes were saved in
every motor. `CAN_TIMEOUT` (`0x7028`, unsigned 32-bit) is set to **0** by this
driver at activation for all seven motors. This disables the motor watchdog so
the brake-free arm can keep holding after a clean driver stop; loss of commands
does not therefore guarantee torque removal. See [stopping behaviour](../README.md#stopping-behaviour).

### Gains are two different controls

J1–J6 in CSP use the motor's internal position/speed loops. Driver GPIO `kp`/`kd`
do not tune those loops. J4's recorded inner-loop values were `loc_kp=30`
(`0x701E`), `spd_kp=8` (`0x701F`) and `spd_filter_gain=0.05` (`0x7021`);
these were observations, not a request to copy them to other joints.

J7's teleop motion-mode gains are **kp=12 Nm/rad, kd=0.5 Nm·s/rad**, supplied in
each Type-1 command by the driver. They are not the firmware's current-loop or
position-loop gains, and Save All does not replace the need to configure them
in the driver. The J7 post-save snapshot separately read `loc_kp=10`, `spd_kp=5`,
`spd_ki=0.02`, `spd_filter_gain=0.05`, `cur_kp=0.125`, `cur_ki=0.0158` and
`cur_filter_gain=0.1`. Preserve these unless separately commissioned.

The 1 Nm jaw setting is not a calibrated fingertip-force limit or a hard
instantaneous torque bound: a recorded tuned session reached approximately
2.834 Nm measured motor torque magnitude. Grip force, transient overshoot and contact
detection need their own physical validation.

## Write settings through the hardware owner

The arm has no brakes. Begin maintenance at verified physical and measured HOME,
with a clear workspace, no running Objective or leader stream, healthy motor
feedback, and the approved temperature/fault monitoring. Disable torque only at
HOME and keep the arm supported. Deactivate commanding controllers before
changing parameters or zero; an old setpoint becomes a motion command after
re-zeroing. Keep one hardware owner for the bus throughout maintenance.

**Do not use `motorbridge-cli robstride-read-param` on an enabled motor.** The
installed CLI opens a `Controller` context whose `__exit__` calls `shutdown()`;
even a parameter read can disable the registered motor. This caused a J7
shutdown during the 2026-10-20 bench session. Do not open a second SDK controller
beside the driver. Use audited Type-17 reads coordinated with the existing owner
and reject replies with the wrong motor, echoed index, error status or age.

For the two teleop changes, `rebot_leader_tune.py` first prints a dry run. Its
explicit `--apply` path requires healthy HOME with all seven motors OFF and
uses the owning driver; it writes volatile settings and never saves them.
For a separately approved float-register adjustment, the driver's service is:

```bash
# Examples for a commissioned maintenance session, not an automatic setup step.
ros2 service call /robstride_hardware_interface/set_data_to_robstride \
  robstride_interfaces/srv/SetDataToRobstride \
  "{id: 4, item_name: '0x7017', item_data: 33.0}"
ros2 service call /robstride_hardware_interface/set_data_to_robstride \
  robstride_interfaces/srv/SetDataToRobstride \
  "{id: 7, item_name: '0x700B', item_data: 1.0}"
```

The same service can write the table's other float limits by joint ID and
register index. Do not bulk-replay the table without inspecting the target
motor. A successful service response means the CAN frame was sent, not that
the firmware accepted it. Independently read back every changed register
before enabling; partial writes are possible. A mismatch or missing reply
leaves the arm OFF at HOME for diagnosis, without automatic retries.

The service encodes numeric indices as **float32**. Do not use it to set
byte-valued `run_mode`/`zero_sta` or uint32 `CAN_TIMEOUT`. Those require the
correctly typed owner maintenance path. The driver already writes its mode and
watchdog settings at activation. Its `get_data_from_robstride` service reads
cached position/velocity/torque/temperature/health fields; it does **not** read
arbitrary firmware registers when given a hexadecimal index.

## Mechanical zero and persistent saving

1. Record all existing settings and verify the intended physical alignment.
   Do not infer HOME from encoder numbers if the zero is in doubt. Keep wrist
   cables clear and preserve the commissioned arm alignment. For J7, zero
   means the jaw is physically closed with the fingers touching, not merely
   that a close command completed.
2. With the affected joint OFF at supported HOME and no controller chasing an
   old reference, use the owner's `set_zero_robstride` service (request `id`)
   for that joint only. It sends the Type-6 mechanical-zero command. Verify
   near-zero feedback, physical alignment and the separate `zero_sta` setting
   before considering a save. The service alone does not save parameters or
   write `zero_sta`.
3. Review the full parameter snapshot before authorizing persistence.
   RobStride **Save All** is communication type **22** (`0x16`), not a
   selected-register save. There is no proven firmware-specific EEPROM
   whitelist here, and no way to promise that only zero and torque limit will
   be saved. Runtime targets, modes and per-command MIT gains must not be
   presumed persistent just because a tool exposes them.
4. Through the approved owner-coordinated maintenance path, send one Save All
   to the intended motor and check its response. The driver currently exposes
   no Save All ROS service. The vendor wire contract below is for an audited
   maintenance implementation, not a second controller alongside the driver.
   Do not substitute a factory-default import or a fault-clear/reboot request.
5. Re-read the changed values and the complete available snapshot; retain the
   response and before/after health. A save acknowledgement plus immediate
   readback establishes only that stage. Verify retention in a **separately
   approved** power-cycle test at supported HOME, then re-read settings and
   confirm physical zero before enabling. Re-seed controllers from the new
   measured state before any subsequent movement.

All frames below use extended CAN identifiers and eight data bytes. For this
bench, `host=0xFD`, and `motor` is 1–7:

| Operation | Extended identifier | Payload |
| --- | --- | --- |
| Read parameter, Type 17 | `(17 << 24) \| (host << 8) \| motor` | Little-endian uint16 index, two zero bytes, four zero bytes |
| Write float parameter, Type 18 | `(18 << 24) \| (host << 8) \| motor` | Little-endian uint16 index, two zero bytes, little-endian float32 value |
| Set mechanical zero, Type 6 | `(6 << 24) \| (host << 8) \| motor` | `01 00 00 00 00 00 00 00` |
| Save All, Type 22 | `(22 << 24) \| (host << 8) \| motor` | `01 02 03 04 05 06 07 08` |

For example, J7 Save All is identifier `0x1600FD07`. One such save was
acknowledged on **2026-10-20 at 20:27:47 UTC**, followed by successful J7
register readbacks. No power-cycle retention test followed that save, and it
does not establish that J4's new speed limit was saved. J7 was mechanically
re-zeroed with the jaw touching at 20:13:56 UTC; independent feedback changed
from approximately −0.194243 rad to −0.000192 rad. The inaccessible mechanical
offset register `0x2005` returned error 1, so no numeric saved-offset claim is
made.

Protocol names/types come from the manufacturer's
[parameter and communication definitions](https://github.com/RobStride/Python_Sample/blob/cbf977e56c842d57a65f3f17c1b1ecaef002c424/robstride_dynamics/protocol.py).
The workspace's [driver callbacks](../../external_dependencies/robstride_hardware_interface/robstride_hardware_interface/src/robstride_hardware_interface.cpp)
and [frame encoders](../../external_dependencies/robstride_sdk/robstride_sdk/include/robstride_sdk/robstride_motor.hpp)
define the ROS service behaviour described above.
