# cad — parametric mechanical model

Code-CAD for the Recon UGV. Written as a script rather than modelled in a GUI, so the
dimension table in `docs/mechanical-requirements.md` drives the geometry directly and mass
and inertia can be computed from the solids instead of estimated.

## What it does

```
python3 cad/build.py            # export STEP + STL to cad/out/, and report mass and inertia
python3 cad/build.py --report    # report only, write nothing
```

STEP files open in any CAD package if you want to edit by hand; STLs go straight to a slicer.
`cad/out/` is gitignored — it is regenerated, like `protocol/generated/`.

## Layout

| File | Role |
|---|---|
| `parameters.py` | Every dimension, tagged OWNER / DERIVED / **ASSUMPTION**. The only place literals live. |
| `parts.py` | Eight parts as solids. Four shape types: tube, disc, flat plate, cylinder. |
| `build.py` | Exports, and computes mass and rotating inertia from the actual geometry. |

## This is a starting point, not a design

**Read the ASSUMPTION tags in `parameters.py` before printing anything.** Several dimensions
were invented because the geometry needed a number and no analysis supplies one. The weakest
are:

- **`WHEEL_OD = 150`** — must exceed the 135 mm casing OD or the casing drags. Ground
  clearance is currently 7.5 mm. Nothing derives this; it is a choice.
- **The wheel hub and axis interface** — how the wheel mounts and is driven is unresolved.
  The boss and bearing sizes make the parts printable, nothing more.
- **`PITCH_MOTOR_BORE = 28`** — a placeholder for a 28 mm gimbal motor. R1 flags the chassis
  frame diameter as the dimension the CAD must verify against the motor actually chosen.
- Camera module hole spacing, belt dimensions, and bearing sizes are nominal values to
  confirm against real datasheets.

## What building it revealed

The first pass came out at **855 g of printed plastic** against the owner's 700–900 g vehicle
target, leaving −155 to +45 g for motors, battery, electronics and bearings. No estimate had
caught this, because the estimate only costed the casing shell and never costed the end caps,
chassis discs or wheels at all.

Rebuilding the four offenders as rim-hub-and-spoke rather than solid discs, taking the wall to
the thin end of the owner's range, and reporting in ABS brought it to **398 g** — a 53%
reduction, leaving 302–502 g for everything else.

Measured rotating assembly: **209 g, I = 0.762e-3 kg m^2**, against an estimate of 0.647e-3
(light) to 1.534e-3 (heavy). The light-corner estimate was close; the heavy corner was
pessimistic.

## Known gaps

- The pitch motor is not modelled and rides in the casing, so it is missing from the rotating
  inertia. Add it before trusting the torque budget.
- No assembly model, so nothing checks for interference between parts. Each part is validated
  in isolation.
- The belt, bearings, slip ring, axle and fasteners are bought, not modelled.
