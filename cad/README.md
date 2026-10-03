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
| `parts.py` | Eight printed parts: shell, two end caps, chassis spine, IMU bridge, camera mount, wheel hub, and its TPU tire (ADR 0017). |
| `bought_parts.py` | Envelopes for bought components, tagged MEASURED / LISTING / GUESS. |
| `assembly.py` | Places everything and sweeps every pair for interference over relative rotation. |
| `build.py` | Exports, and computes mass and rotating inertia from the actual geometry. |

## The machine at a glance (ADR 0008, ADR 0010)

70 × 182 mm casing, 105 mm wheels, 214 mm overall. Direct drive: no belt, no pulley, no
bracket. From end A, along the axis:

```
wheel A | cap A | pitch motor (in the cup) | wheel motor A | waist + IMU | wheel motor B | cap B | wheel B
          ^ stator bolts here                                  ^ IMU at r = 3.8 mm
```

The chassis is a single **spine** on the axis. The casing turns a full turn relative to it,
so anything the chassis has at radius r sweeps a whole ring and the casing loses that radius.
A spine keeps the chassis small everywhere. It also means nothing on the casing can be *on* the
axis, which is why the IMU lies beside the waist instead (ADR 0010).

## This is a starting point, not a design

**Read the ASSUMPTION tags in `parameters.py` before printing anything.** The weakest are:

- **The DM3505's through-bore** — the datasheet gives ⌀8.5 on the rotor face but not that it
  runs through the base. Confirm on the part before printing end cap A or the spine.
- **`SPINE_WAIST_OD = 5`** — a bought rod (M5 threaded rod or 5 mm tube) carrying all the
  wheel loads. Sets the IMU offset.
- **`IMU_CHIP_SIDE_HEIGHT = 1.0`** — the tallest part on the breakout's chip side. Also sets
  the IMU offset.
- **`WHEEL_MOTOR_LENGTH = 40`** — N20 length varies with gear ratio and sets every station.
- The spine is drawn as one solid. In reality it is two printed ends plus a rod.
- Camera hole spacing, bearing sizes and bolt circles are nominal values to confirm.

## What building it revealed

- First design: **855 g** of plastic against a 700–900 g vehicle target, cut to 398 g by
  rebuilding discs as rim-hub-and-spoke. Current design: **212 g**.
- Current rotating plastic: **140 g, I = 0.136e-3 kg m^2**, within 2% of ADR 0008's hand
  figure. With the motor stator, bearing races and electronics
  (`tools/pitch_inertia_budget.py`): 267 g, 0.180e-3, of which the shell is 67%.
- The first design's sweep checked one pose and passed an IMU bridge that went through a
  chassis standoff. That is what the rotation-aware sweep and its regression test are for.

## Known gaps

- The electronics are in the torque budget only as point masses at estimated radii. Placing
  their envelopes in the assembly would replace the estimates.
- Bought electronics have envelopes in `bought_parts.py` but are not placed in the assembly
  yet. The Nucleo-G474RE (70 × 82) cannot fit a 65 mm bore at all.
- ADR 0009's wire loop is not modelled. Its zone (around the spine near end B) is not yet
  reserved against the electronics.
- Printed dimensional accuracy is not checked: FDM can close a 0.5 mm running gap entirely.
