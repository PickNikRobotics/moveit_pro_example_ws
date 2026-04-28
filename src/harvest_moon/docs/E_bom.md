# E. Bill of Materials

Every part needed to recreate the prototype rig. Items marked **[FIELD]**
need to be filled in from the physical inventory — the prototype was
assembled before this doc was written and exact part numbers/vendors for
some commodity items aren't recorded here yet.

For *why* each component is what it is, see
[D. System Architecture Overview §5 (Design rationale)](D_architecture.md).
For wiring of these parts, see
[F. Hardware Build Guide](F_hardware_build.md).

---

## §1. Compute and capture

| Item                 | Model / Vendor                              | Qty | Notes                                       |
|----------------------|----------------------------------------------|-----|---------------------------------------------|
| Compute platform     | NVIDIA Jetson Orin AGX 64 GB Developer Kit  | 1   | Includes 64 GB module on dev-kit carrier    |
| Storage              | 1 TB NVMe SSD                               | 1   | **[FIELD]** — exact model not recorded      |
| GMSL2 capture card   | StereoLabs ZED Link Quad (PCIe)             | 1   | Mounts in Jetson dev-kit M.2/PCIe slot      |

---

## §2. Cameras

| Item                | Model / Vendor                              | Qty | Notes                                                |
|---------------------|----------------------------------------------|-----|------------------------------------------------------|
| Stereo camera       | StereoLabs ZED X                            | 1   | Serial **47845360**                                  |
| Stereo cable        | StereoLabs GMSL2 cable, 5 m                 | 1   | Connects ZED X ↔ ZED Link Quad                       |
| Area camera         | Basler acA2040-35gc                         | 2   | Serials **25233972**, **25435376** (= `device_user_id`) |
| Basler lens         | C-mount lens                                | 2   | **[FIELD]** — record focal length, aperture, vendor  |

> The convention `device_user_id = camera serial number` is set during
> first-time provisioning via `harvest_moon/scripts/provision_basler.py`. See
> [G. Software Setup Guide](G_software_setup.md).

---

## §3. Lighting

| Item             | Model / Vendor                              | Qty | Notes                                       |
|------------------|----------------------------------------------|-----|---------------------------------------------|
| Strobed LED bar  | Smart Vision Lights LHI-DO 600 mm           | 2   | Built-in driver, NPN-triggered              |
| Light cable      | M12 5-pin to flying leads                   | 2   | **[FIELD]** — length and vendor             |

LHI-DO datasheet path on the Jetson: `~/LHI-DO_Datasheet.pdf`.

---

## §4. Network

| Item                    | Model / Vendor                       | Qty | Notes                                          |
|-------------------------|---------------------------------------|-----|------------------------------------------------|
| GigE PoE+ switch        | TrendNet TI-PG80B                    | 1   | 24 V DC input, 8-port PoE+, DIN rail mount     |
| Cat6 patch cable        | **[FIELD]** length(s)                 | 2   | Camera ↔ switch (one per camera)               |
| Cat5e patch cable       | **[FIELD]** length                    | 1   | Switch uplink ↔ Jetson `eno1`                  |

Network IP plan: cameras on `192.168.5.0/24`, Jetson `eno1` at
`192.168.5.2`, cameras at `192.168.5.3` and `192.168.5.4`. Set via
`PylonGigEConfigurator` during software setup; see
[G. Software Setup Guide](G_software_setup.md).

---

## §5. Power

| Item                     | Model / Vendor                              | Qty | Notes                                    |
|--------------------------|----------------------------------------------|-----|------------------------------------------|
| 24 V DC PSU              | 24 V / 500 W DIN-rail supply                | 1   | **[FIELD]** — exact model not recorded   |
| 24 V → 12 V DC-DC        | DIN-rail DC-DC converter                    | 1   | **[FIELD]** — Jetson + ZED Link rail     |
| DIN rail                 | Standard 35 mm DIN                          | —   | **[FIELD]** — length depends on enclosure |
| DIN-rail terminal blocks | Phoenix or equivalent, 24 V + GND distribution | several | **[FIELD]**                              |

Loads on the 24 V rail (sums to under 100 W in production):

| Load                      | Approx. draw     |
|---------------------------|------------------|
| TrendNet TI-PG80B + PoE   | ~10 W idle, +PoE |
| 2× LHI-DO 600 mm          | ~46 W avg per light at full power (0.97 A/300 mm × 2 segments × 24 V); strobed duty cycle drops this dramatically |
| 12 V DC-DC → Jetson       | up to 60 W       |
| ZED Link Quad             | small            |

**Common ground bus:** 24 V PSU negative is tied to: ZED J17 Pin 2,
both Baslers' Pin 5 (Gray, opto GND), both lights' Blue (GND), and the
12 V DC-DC output GND. Establishing this common ground is critical for
the opto-isolated trigger paths to work — see
[F. Hardware Build Guide](F_hardware_build.md).

---

## §6. Trigger box (Teensy + driver)

| Item                  | Model / Vendor                              | Qty | Notes                                       |
|-----------------------|----------------------------------------------|-----|---------------------------------------------|
| Microcontroller       | PJRC Teensy 4.0                             | 1   | USB-powered from Jetson                     |
| USB cable             | Type-A → micro-USB                          | 1   | **[FIELD]** — length                        |
| NPN transistor        | 2N3904 (TO-92)                              | 1   | Drives both Baslers' opto inputs in parallel |
| Base resistor         | 560 Ω, ¼ W                                  | 1   | Limits Teensy GPIO drive current            |
| Perf board / proto    | **[FIELD]**                                 | 1   | Mounts the transistor + resistor cleanly    |
| Project enclosure     | **[FIELD]**                                 | 1   | Optional — currently bare on bench          |

Driver circuit (text form; schematic in
[F. Hardware Build Guide](F_hardware_build.md)):
- Teensy Pin 12 ──[ 560 Ω ]──► 2N3904 base
- 2N3904 emitter ──► common GND bus
- 2N3904 collector ──► both Baslers' Pin 2 (Pink) tied together
- Camera Pin 5 (Gray) on each Basler ──► common GND bus

When Teensy Pin 12 goes HIGH, the transistor saturates, sinking ~6 mA
through both opto-LEDs in series with the Baslers' internal current
limit, well above the 5 mA minimum spec on Line 1.

---

## §7. Camera cables and connectors

| Item                            | Model / Vendor                              | Qty | Notes                                  |
|---------------------------------|----------------------------------------------|-----|----------------------------------------|
| Basler I/O cable                | Hirose HR10A-7P-6S to flying leads          | 2   | Mates with Basler HR10A-7R-6PB I/O port |
| Heat-shrink, ferrules, etc.     | **[FIELD]**                                 | —   | For terminating ground bus and signals |

**Basler I/O cable color code** (Basler "Power-IO HRS 6p/open" standard):

| Wire color | Camera pin | Function                  | Used as                         |
|------------|------------|---------------------------|---------------------------------|
| Brown      | 1          | Camera Power              | unconnected (PoE powers camera) |
| Pink       | 2          | Line 1 — Opto IN          | Trigger from Teensy NPN driver  |
| Green      | 3          | Line 3 — GPIO             | unconnected                     |
| Yellow     | 4          | Line 2 — Opto OUT         | Drives paired LHI-DO NPN trigger |
| Gray       | 5          | Opto GND                  | Common GND bus                  |
| White      | 6          | Camera Power / GPIO GND   | unconnected                     |

Reference: https://docs.baslerweb.com/basler-power-io-cable-hrs-6p-open-s

**LHI-DO M12 5-pin cable color code:**

| Wire color | Pin | Function              | Used as                           |
|------------|-----|-----------------------|-----------------------------------|
| Brown      | 1   | Power In (+24 VDC)    | 24 V DIN rail                     |
| White      | 2   | NPN trigger (sinking) | Driven by Basler Line 2 (Yellow)  |
| Blue       | 3   | GND                   | Common GND bus                    |
| Black      | 4   | PNP trigger (sourcing)| unconnected                       |
| Grey       | 5   | Intensity (1–10 VDC)  | Jumpered to Brown for max         |

---

## §8. ZED Link wiring

| Item                  | Model / Vendor                              | Qty | Notes                                                |
|-----------------------|----------------------------------------------|-----|------------------------------------------------------|
| ZED Link J17 jumper   | Generic 2-pin 2.54 mm header lead           | 1   | Connects J17 Pin 16 (trigger in) to Teensy Pin 2 + Pin 2 to GND |

The ZED Link Quad's J17 header carries the trigger input. Reference:
https://www.stereolabs.com/docs/embedded/zed-link/zed-link-quad-triggering

| ZED J17 pin | Function              | Used as                       |
|-------------|------------------------|-------------------------------|
| Pin 16      | Trigger IN (3.3 V)    | From Teensy Pin 2             |
| Pin 2       | GND                    | Common GND bus                |

---

## §9. Reference documents (not parts)

| Document                                | Location                                        |
|-----------------------------------------|------------------------------------------------|
| LHI-DO datasheet                        | `~/LHI-DO_Datasheet.pdf`                        |
| Basler acA2040-35gc docs                | https://docs.baslerweb.com/aca2040-35gc        |
| Basler HRS 6p/open cable                | https://docs.baslerweb.com/basler-power-io-cable-hrs-6p-open-s |
| ZED Link Quad triggering                | https://www.stereolabs.com/docs/embedded/zed-link/zed-link-quad-triggering |
| TrendNet TI-PG80B datasheet             | **[FIELD]** — link to vendor page              |

---

## §10. Optional / convenience items used in the prototype

These aren't strictly required to recreate the system but were used during
development:

| Item                          | Purpose                                          |
|-------------------------------|--------------------------------------------------|
| Multimeter                    | Verifying 24 V rail, GND continuity              |
| Oscilloscope / logic analyzer | Verifying trigger pulses, sync timing            |
| Wire labels / cable ID tags   | Sanity                                           |
| **[FIELD]** Project enclosure | If the customer wants the prototype fully boxed  |

