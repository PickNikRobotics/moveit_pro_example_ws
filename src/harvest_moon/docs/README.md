# harvest_moon Documentation

Documentation for the harvest_moon vision rig: 2× Basler acA2040-35gc +
1× ZED X stereo camera, hardware-synchronized via a Teensy 4.0, with 2×
LHI-DO strobes, running alongside MoveIt Pro 9.0.0 on a Jetson Orin AGX.

The docs are organized into two tracks. Pick the one that matches what
you're doing.

---

## Field Operator track

Read this if you're going to run the system, collect data, and
troubleshoot in the field.

- **[A. Field Quickstart](A_field_quickstart.md)** — power up, launch,
  capture, shut down. Nothing else.
- **[B. Operations & Tuning Playbook](B_operations.md)** — recipes for
  exposure, focus, frame rate, sync timing, strobe, storage. Each
  recipe is self-contained.
- **[C. Field Troubleshooting Runbook](C_field_troubleshooting.md)** —
  symptom → likely cause → fix, in priority order. Open this when
  something's wrong.

If you only have time for one doc, A is the one.

---

## System Rebuilder track

Read this if you're recreating the system from scratch (e.g. a customer
ruggedizing the prototype for field deployment).

- **[D. System Architecture Overview](D_architecture.md)** — what the
  system is, block diagrams, design rationale. Read this first.
- **[E. Bill of Materials](E_bom.md)** — every part, model number,
  serial, source. The shopping list.
- **[F. Hardware Build Guide](F_hardware_build.md)** — step-by-step
  assembly with wiring detail. Each step verifies the previous one.
- **[G. Software Setup Guide](G_software_setup.md)** — first-time
  software provisioning in order: Teensy firmware → pylon SDK →
  camera IPs → UserSet1 → ZED daemon → MoveIt Pro → workspace build →
  first launch.
- **[H. Configuration Reference](H_config_reference.md)** — every
  YAML / launch / firmware constant annotated. Use this when you want
  to know *why* a value is what it is, before changing it.

---

## Deep dives (referenced from both tracks)

- **[J. Deep Troubleshooting](J_deep_troubleshooting.md)** — long-form
  root-cause investigations for the GigE bandwidth bottleneck (the
  15 Hz dual-cam story) and the ApproxTimeSync stamp-offset
  compensation pattern. Both topics are referenced from C and D.

---

## Supporting scripts

The scripts referenced throughout these docs live in
`harvest_moon/scripts/`:

| Script                  | Purpose                                        | Doc reference |
|-------------------------|------------------------------------------------|---------------|
| `provision_basler.py`   | One-time Basler UserSet1 setup                 | G §4          |
| `record_dataset.py`     | Field data recorder (rosbag2 wrapper)          | A §5, B §6    |
| `ros_sync_test.py`      | ROS-side sync test, demonstrates stamp-offset relay | J §2     |
| `triple_sync_test.py`   | Standalone 3-camera sync test (no ROS)         | B §4          |
| `provision_basler.py` and friends | One-shot diagnostic / setup tools     | various       |

