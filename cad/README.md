# cad — parametric mechanical model

Code-CAD for the Recon UGV. Written as a script rather than modelled in a GUI, so the
dimension table drives the geometry directly and mass and inertia can be computed from the
solids instead of estimated.

## What it does

```
python3 cad/build.py               # export STEP + STL to cad/out/, report mass and inertia
python3 cad/build.py --report      # report only, write nothing
python3 cad/assembly.py            # positioned assembly STEP + rotation-aware clearance sweep
python3 cad/assembly.py --report   # sweep only
pytest tests/python/test_cad.py    # skipped unless requirements-cad.txt is installed
```

STEP files open in any CAD package if you want to edit by hand; STLs go straight to a slicer.
`cad/out/` is gitignored — it is regenerated, like `protocol/generated/`.

## Layout

| File | Role |
|---|---|
| `parameters.py` | Every dimension, tagged OWNER / DERIVED / **ASSUMPTION**, and the axial layout as derived stations. The only place literals live. |
| `parts.py` | Seven printed parts: shell, two end caps, chassis spine, IMU bridge, camera mount, wheel. |
| `bought_parts.py` | Envelopes for bought components, tagged MEASURED / LISTING / GUESS. |
| `assembly.py` | Places everything and sweeps every pair for interference over relative rotation. |
| `build.py` | Exports, and computes mass and rotating inertia from the actual geometry. |

## The machine at a glance (ADR 0008, ADR 0010)

70 × 182 mm casing, 105 mm wheels, 214 mm overall. Direct drive: no belt, no pulley, no
bracket. From end A, along the axis:

```
wheel A | cap A | pitch motor (in the cup) | wheel motor A | waist + IMU | wheel motor B | cap B | wheel B
          ^ rotor bolts here                                   ^ IMU at r = 4.75 mm
```

The chassis is a single **spine** on the axis. The casing turns a full turn relative to it,
so anything the chassis has at radius r sweeps a whole ring and the casing loses that radius.
A spine keeps the chassis small everywhere. It also means nothing on the casing can be *on* the
axis, which is why the IMU rings the waist instead (ADR 0010).

## This is a starting point, not a design

**Read the ASSUMPTION tags in `parameters.py` before printing anything.** The weakest are:

- **`PITCH_MOTOR_HOLLOW_BORE = 5`** — the layout needs a hollow-shaft motor because wheel A's
  shaft passes through it. Whether a 28 mm one is buyable has not been checked.
- **`SPINE_WAIST_OD = 5`** — a bought rod carrying all the wheel loads. Sets the IMU offset.
- **`WHEEL_MOTOR_LENGTH = 40`** — N20 length varies with gear ratio and sets every station.
- The spine is drawn as one solid. In reality it is two printed ends plus a rod.
- Camera hole spacing, bearing sizes and bolt circles are nominal values to confirm.

## What building it revealed

- First design: **855 g** of plastic against a 700–900 g vehicle target, cut to 398 g by
  rebuilding discs as rim-hub-and-spoke. Current design: **209 g**.
- Current rotating assembly: **138 g, I = 0.135e-3 kg m^2**, within 2% of ADR 0008's hand
  figure. The shell is 90% of it.
- The first design's sweep checked one pose and passed an IMU bridge that went through a
  chassis standoff. That is what the rotation-aware sweep and its regression test are for.

## Known gaps

- Rotating inertia is **plastic only**. The camera, Pi, MCU and battery sit near the wall and
  will add a lot; the torque budget is not trustworthy until they are placed.
- Bought electronics have envelopes in `bought_parts.py` but are not placed in the assembly
  yet. The Nucleo-G474RE (70 × 82) cannot fit a 65 mm bore at all.
- ADR 0009's wire loop is not modelled, and neither is the chassis platform for the TB6612.
- Printed dimensional accuracy is not checked: FDM can close a 0.5 mm running gap entirely.
