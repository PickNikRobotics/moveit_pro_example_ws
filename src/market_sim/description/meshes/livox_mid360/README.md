# A2 Livox Mid-360 reference mount

A2 is a **printable reference design**, not validated production hardware. It has two
PLA blades bolted directly onto the fixed spine cap, a filleted crown, and a 4 mm
6061 heat plate flush with the crown. There is no base plate.

## Assets and coordinates

- `cad/`: original A2 part STEP/STL, dated 2026-11-11. STLs are **millimetres**, in
  scan-origin O coordinates with axes parallel to `base_link`. These are assembly
  coordinates, **not printer orientations**. `cad/source.json` records hashes,
  dimensions, source revision and material/mass assumptions. The four illustrative
  sensor parts were extracted from the A2 assembly's corresponding facets.
- `visual/`: **metre-scale** meshes for both URDF and MuJoCo. The three printed
  parts retain fillets, sockets and cable-tie slots. The plate retains its holes.
  Only the sensor meshes are rotated back into the native sensor frame.
- `collision/`: **metre-scale** conservative hulls, one per blade and plate, plus
  three separately clipped crown rails. There is deliberately **no whole-bracket
  hull**: it would fill the central sensor clearance. Fastener details are omitted.
- `conversion.json`: actual triangle counts, bounds, sampled surface differences
  and estimated mount mass properties.

The selected C170 scan origin is `(0.210, 0, 1.700)` m in `base_link`: X is rounded
0.250 mm forward from the measured cap center, with height unchanged. Subtracting
OEM fixed-spine origin `(0.138289, 0, 0.350)` m gives the mount joint translation
`(0.071711, 0, 1.350)` m. Do not independently round that subtraction.

The bracket stays unrolled. The native `livox_frame` rolls **once** by π about X,
so native −7°…+52° becomes mounted −52°…+7°. The CAD's inverted sensor housing
already places its mounting face **47 mm above O**. Rotating the sensor-only
visual back into native axes avoids double-inverting it at the joint. The nominal
65 × 65 × 60 mm, 265 g housing/window/connector illustration is not vendor CAD.

## Reproduce the conversion

This is an offline development tool, not an additional runtime requirement. Use a
throwaway Python environment with `trimesh==4.12.2`, `numpy`, `scipy==1.17.0`,
`rtree==1.4.1`, `shapely==2.1.2` and `fast-simplification==0.1.13`, then run:

```bash
python3 src/market_sim/scripts/convert_livox_assets.py
```

Quadratic decimation runs in **millimetres** because its tolerances are absolute.
It tries 70%, 85%, 95% and 98% retention, falling back to the original mesh only
when needed to preserve watertightness and a 0.20 mm sampled surface-error bound.
The retained output has **22,292 visual triangles**, down from 27,130. The largest
measured bidirectional difference is **0.041 mm**, checking every vertex and 6,000
seeded area-weighted surface samples per direction. This is a sampled check, not a
certified Hausdorff bound. It deliberately keeps modest meshes rather than erase
small through-holes for a more impressive reduction ratio.

The separate collision parts have **3,088 triangles** in total. The crown is split
at X=35 mm into a front rail and left/right rails before convexification. Tests
exercise the real compiled model and FCL: the crown clearance remains open, the
sensor contacts world obstacles, and only the two fixed attachment pairs are
excluded from self-collision checking.

## Rendering and dynamics

The closed optical-window illustration is a URDF visual shown in the Desktop
only; MuJoCo has no geom for it, so the transmissive window is not an opaque
depth obstacle inside its own scanner. Lidars render groups 0/1/2, which still
include every opaque housing, blade, crown, arm, tower and store surface. Physics uses separate group-3
collision envelopes. This does not remove real robot self-returns from the cloud.

Mount mass is **0.370887 kg**: 83.4/83.3/69.4 g PLA print estimates plus 134.787 g
solid aluminum plate at 2,700 kg/m³. The printed estimates are from an X1C slice
with four walls and 25% gyroid, **not weighed parts**. Inertia assumes uniform
density within each CAD part, and omits fasteners. The sensor uses an approximate
box inertia with its nominal 265 g mass. These values make simulation bookkeeping
explicit; they do not establish physical stiffness, thermal performance or
infill-resolved dynamics.

The original reference CAD, derived meshes and conversion script are distributed
under the repository's BSD-3-Clause license; see `market_sim/NOTICE`.
