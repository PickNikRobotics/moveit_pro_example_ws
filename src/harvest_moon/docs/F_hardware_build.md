# F. Hardware Build Guide

Step-by-step assembly of the prototype rig. Follow in order; each step
verifies the previous one, so a failure isolates cleanly.

Before starting:
- Get all parts assembled per [E. Bill of Materials](E_bom.md).
- Read [D. System Architecture Overview](D_architecture.md) for context.
- Have a multimeter on hand for the power and ground checks.

---

## §1. Bench / DIN-rail layout

The prototype mounts the switch, PSU, and DC-DC converter on a single
length of 35 mm DIN rail. The Jetson sits on its dev-kit feet next to the
rail, the cameras and lights are positioned wherever the application
requires, and the Teensy lives in a small perfboard enclosure (or bare
on the bench during dev).

Suggested left-to-right order on the rail:
```
[ 24 V PSU ] [ 24V→12V DC-DC ] [ TrendNet PoE switch ] [ terminal blocks ]
```

Reasoning: power sources on the left feed loads to the right via terminal
blocks. Keeps wire runs short and signal/power separation clean.

> **[FIELD]** Exact rail length depends on the customer's enclosure
> footprint. Allow ~10 cm headroom past the rightmost component for
> service.

---

## §2. Power distribution

### 2.1 Mount and wire the 24 V PSU

1. Mount the PSU on the DIN rail at the left.
2. Wire mains in (per the PSU's terminal labels — typically L / N / PE).
3. With nothing else connected, power up. Verify 24 V (±0.5 V) at the
   PSU's `V+` / `V−` output terminals with a multimeter. Power down.

### 2.2 Establish the common ground bus

Add a multi-tap GND terminal block on the DIN rail. Wire the PSU's `V−`
into it. Every GND in this system terminates here:

- PSU `V−` (24 V negative)
- 12 V DC-DC output GND
- ZED Link J17 Pin 2
- Basler camera 1 Pin 5 (Gray)
- Basler camera 2 Pin 5 (Gray)
- LHI-DO 1 Blue wire
- LHI-DO 2 Blue wire
- Trigger box GND (Teensy GND, transistor emitter)

> **Why one bus matters:** the opto-isolated trigger circuits *float*
> until both sides share a return path. Without a common GND, the
> Baslers' opto inputs may read as triggered when not, and may miss real
> triggers. The "everything tied to one bus" rule eliminates ambiguity.

### 2.3 Mount and wire the 12 V DC-DC converter

1. Mount on the DIN rail next to the PSU.
2. Input: 24 V from the PSU (`V+` and `V−` to the DC-DC's input terminals).
3. Output: 12 V to a separate output terminal block, plus GND tied to the
   common GND bus.
4. Power up; verify 12 V (±0.3 V) at the output terminal with a multimeter.

### 2.4 Power the Jetson

The prototype powers the Jetson Orin AGX via **USB-C** from a
multi-output power adapter (separate from the 24 V rail). The Jetson
ships with a USB-C PD power input.

> **[FIELD]** This is the prototype's bench arrangement. For the
> ruggedized field rebuild, the customer's integrator will decide
> whether to keep USB-C PD (and source a DIN-rail PD supply) or
> migrate to a different input.

---

## §3. Network

### 3.1 Mount and wire the TrendNet TI-PG80B

1. Mount on the DIN rail.
2. Wire 24 V (`V+`) and GND (`V−`) from the PSU's output and the common
   GND bus.
3. Power up; the `PWR1` LED should turn solid green.

### 3.2 Cable the cameras and Jetson

1. Cat6 from Basler 1 Ethernet port → any unused PoE port on the switch.
2. Cat6 from Basler 2 Ethernet port → another unused PoE port.
3. Cat5e from the switch's uplink port (or any spare port) → Jetson
   `eno1`.

Camera ports should show solid green + solid orange (link + PoE) once
Cat6 is plugged in. Jetson port shows solid green.

The Jetson's `eno1` IP and camera IPs are configured in
[G. Software Setup Guide](G_software_setup.md):
- Jetson `eno1`: `192.168.5.2`
- Basler 1: `192.168.5.3`
- Basler 2: `192.168.5.4`

---

## §4. ZED X and ZED Link Quad

### 4.1 Install the ZED Link Quad

1. Insert the ZED Link Quad PCIe card into the Jetson's PCIe slot.
2. Boot the Jetson; install the ZED SDK and ZED X driver (see
   [G. Software Setup Guide](G_software_setup.md)).

### 4.2 Connect the ZED X

The ZED Link Quad has two GMSL2 camera ports, silkscreened **J1** and
**J2** on the PCB. The trigger pin on J17 depends on which port the
camera is in: J1 → Pin 16, J2 → Pin 13.

1. Plug one end of the 5 m GMSL2 cable into the **J1** port (matches
   the prototype's wiring; the J17 trigger jumper goes to Pin 16).
2. Plug the other end into the ZED X.
3. Power up the 24 V rail (which the ZED Link draws from). The ZED X
   should be detectable by the SDK.

### 4.3 Wire the J17 trigger header

The ZED Link Quad's J17 header carries trigger I/O. With the ZED X on
J1, the trigger signal lives on **J17 Pin 16** (input when ZED is in
slave mode, output when ZED is in master mode — we run slave).

| ZED J17 pin | Function          | Connect to                            |
|-------------|--------------------|---------------------------------------|
| Pin 16      | Trigger (J1 port)  | Teensy Pin 2 (via 2.54 mm jumper lead) |
| Pin 2       | GND                | Common GND bus                         |

> Pin 13 is the J2-port trigger pin. Don't use it — that's a documented
> source of confusion in this system's history.

---

## §5. Cameras (Basler acA2040-35gc)

### 5.1 Mount the cameras

Mount per the application's framing requirements. The acA2040-35gc has
M3 threaded mount points on multiple faces; use whichever face aligns
with the field of view.

### 5.2 Wire the I/O cables

Each camera has a 6-pin Hirose I/O port (HR10A-7R-6PB) that mates with a
Hirose I/O cable to flying leads (Basler "Power-IO HRS 6p/open"). For
each camera:

| Wire   | Camera pin | Connect to                                      |
|--------|------------|--------------------------------------------------|
| Brown  | 1          | unconnected (PoE powers the camera)              |
| Pink   | 2          | Trigger box collector — see §7                   |
| Green  | 3          | unconnected                                      |
| Yellow | 4          | This camera's paired LHI-DO White wire — see §6  |
| Gray   | 5          | Common GND bus                                   |
| White  | 6          | unconnected                                      |

Do this twice — once per camera. Both Pinks land on the same node
(transistor collector); each Yellow goes to **its own** strobe.

---

## §6. Strobes (LHI-DO)

### 6.1 Mount the strobes

Aim each strobe so it illuminates one Basler's field of view. The
prototype uses one strobe per camera; if your scene allows a single
strobe to cover both cameras, you can wire both Yellow Line 2 outputs to
one strobe's NPN trigger — but the prototype's per-camera-per-strobe
setup is cleaner.

### 6.2 Wire each strobe

The LHI-DO uses an M12 5-pin connector. Per strobe:

| Wire   | Pin | Connect to                                      |
|--------|-----|--------------------------------------------------|
| Brown  | 1   | 24 V rail (also jumpered to Grey, see below)     |
| White  | 2   | Paired Basler's Yellow wire (Line 2 output)      |
| Blue   | 3   | Common GND bus                                   |
| Black  | 4   | unconnected                                      |
| Grey   | 5   | Jumpered to Brown (24 V) for max intensity       |

> The Brown↔Grey jumper sets the strobe to maximum intensity. To dim,
> drive Grey with a 1–10 V analog signal instead. See
> [B. Operations & Tuning §5.2](B_operations.md).

### 6.3 Verify strobe power-up (without trigger)

With 24 V applied and NPN trigger (White) floating, the strobe should be
**off**. Briefly tying White to GND with a test jumper should make the
strobe fire. Test each strobe this way to confirm wiring is good before
moving on.

> Don't hold White to GND for more than ~1 second; the LHI-DO's max
> continuous-on duty cycle is limited and prolonged DC excitation will
> trip its thermal protection (or damage the LEDs over time).

---

## §7. Trigger box (Teensy + NPN driver)

### 7.1 Build the NPN driver circuit

Schematic (text form):

```
         Teensy Pin 12 ──[ 560 Ω ]──┬── 2N3904 base
                                     │
                                     │
    Both Baslers'                    │
    Pin 2 (Pink) ──────────────────► 2N3904 collector
                                     │
                                     │
    Common GND bus ─────────────────► 2N3904 emitter
```

Solder onto a small perfboard. Keep leads short. The 2N3904 is a TO-92
package; orient per its flat side / pin numbering.

> **Why a transistor:** The Teensy GPIO is 3.3 V at low drive strength.
> Each Basler opto input requires ≥5 mA at HIGH; two in parallel is
> 10 mA. The transistor switches the larger current through the opto
> LEDs without loading the Teensy directly.

### 7.2 Wire the Teensy

| Teensy pin | Wire to                               |
|------------|----------------------------------------|
| Pin 2      | ZED Link J17 **Pin 16** (trigger)      |
| Pin 12     | NPN base via 560 Ω resistor            |
| Pin 13     | (no external wire — onboard LED)       |
| GND        | Common GND bus + NPN emitter           |
| 5V (or VIN)| (no external wire — USB-powered)       |

The Teensy is powered via USB from the Jetson; no external power
connection is needed.

### 7.3 Optional: enclose the trigger box

Drop the perfboard, Teensy, and a USB cable strain-relief into a small
project box. Bring out:
- 1× USB cable to the Jetson
- 1× wire to the common GND bus
- 1× wire to the ZED Link J17 Pin 16
- 2× wires to the two Baslers' Pin 2 (Pink) — joined inside the box

> **[FIELD]** The prototype trigger circuit is currently bare on the
> bench. The customer's integrator will likely want to enclose it for
> the field rebuild.

---

## §8. Final wiring checks

Two checks you can run before any software setup, plus a third that
requires the Teensy firmware to be flashed (jump to
[G. Software Setup Guide](G_software_setup.md) for that step, then come
back).

### 8.1 Continuity check (system off, no power)

With 24 V rail OFF and Jetson OFF:

- Continuity from PSU `V−` to: ZED J17 Pin 2, Basler 1 Pin 5, Basler 2
  Pin 5, both Light Blues, Teensy GND, NPN emitter. **All should ring
  through.**
- No continuity between PSU `V+` and any GND tap. (Confirms no shorts.)

### 8.2 Power-up voltage check (24 V on, Jetson and Teensy still off)

Power up the 24 V rail. Verify with a multimeter:

- 24 V (±0.5 V) at the PSU output terminals.
- 12 V (±0.3 V) at the DC-DC output.
- 24 V at each light's Brown wire.
- 24 V at each light's Grey wire (jumpered to Brown).
- TrendNet `PWR1` LED solid green; `RLY` LED solid orange.
- TrendNet camera ports: solid green + solid orange after a few
  seconds (PoE delivery + link).
- Strobes idle (no light output).

If anything above is wrong, fix it before going further.

### 8.3 Trigger-chain visual test (after Teensy is flashed)

This is a hardware-only sanity check — no ROS, no scripts, just power
and your eyes. Run this after completing
[G. Software Setup §1 (Flash the Teensy)](G_software_setup.md), once
the firmware is on the Teensy.

**Setup:** 24 V rail on, Jetson on (so the Teensy is USB-powered),
Teensy firmware running in default `RUNNING` mode at production
settings (`BASLER_DIV = 3`, 5 Hz Basler).

**Expected:**

- Teensy onboard LED (Pin 13): pulses ~17 ms every 200 ms (5 Hz blink).
- Both LHI-DO strobes: flash visibly in lockstep at 5 Hz.

**That's it.** A regular 5 Hz strobe pulse on both lights confirms the
whole chain works:
Teensy fires → NPN sinks → Basler optos receive trigger → Basler
exposes → Basler Line 2 sinks → strobe NPN trigger fires → strobes
illuminate.

If the strobes don't behave as expected, isolate by section:

| Symptom                              | Likely cause + check                             |
|---------------------------------------|--------------------------------------------------|
| Teensy LED not blinking               | Firmware not flashed, USB not connected, or `s` (stop) was sent over serial. Send `r` over `/dev/ttyACM0`. |
| Teensy LED blinks but no strobes      | NPN driver or Pink wires; check transistor orientation, base resistor, Pink continuity to NPN collector. |
| Strobes irregular / inconsistent rate | Check both Baslers' Pin 2 (Pink) are joined to the same NPN collector node — a flapping Pink will inject noise. Check 560 Ω base resistor is right value. |
| One strobe works, the other doesn't   | Swap Yellow→White wires between the two cameras. Failure follows the camera → Basler-side issue. Failure follows the strobe → light-side issue. |
| Strobes always on, never strobing     | Yellow→White wire shorted to GND, or strobe NPN input miswired (check it's pin 2/White, not pin 4/Black). |

Once §8.3 passes, the hardware is ready. The remaining setup —
provisioning the Baslers' UserSet1, configuring the ZED daemon, and
bringing up MoveIt Pro — is all in [G. Software Setup Guide](G_software_setup.md).
