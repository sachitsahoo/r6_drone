# 0017 — Wheel tread: printed TPU tire on a printed hub

- **Status:** **ACCEPTED** by the owner, 2026-10-03 (chosen from three options in chat).

## Context

The first wheel carried its tread as a single 3 mm O-ring in a groove round the middle of a
12 mm rim. Two problems:

- **Contact is one narrow line.** The O-ring is the only rubber touching the ground, so
  grip on smooth floors is poor and the wheel tracks into soft surfaces.
- **The rim is thinnest where it is hit.** The groove left about 2.2 mm of plastic under the
  O-ring, at the outer edge of a part that takes the landing loads on an impact-tolerant
  robot.

The 105 mm outside diameter is fixed by ADR 0008 (ground clearance), so any tread has to keep
it.

## Options considered

| Option | For | Against |
|---|---|---|
| **TPU tire on a PLA/PETG hub** | full-width tread; a soft layer between ground and hub soaks impacts; tire is replaceable | needs TPU and a printer that feeds it; the fit has to be tuned by trial |
| Two or three O-rings | one-piece wheel, no new material | still line contacts; more grooves cut into the same thin rim |
| Tread printed into the hard wheel | one part, looks right | hard plastic grips badly on smooth floors; no shock absorption |

## Decision

**A separate TPU (95A) tire, stretched onto the hub.** In `cad/`:

- `parts.tire()`: 5 mm thick ring at WHEEL_OD, 1.5 mm deep transverse grooves, staggered
  between the two halves of the width so a block is always under the contact patch.
- `parts.wheel()`: now a hub. Its rim sits at 95 mm, with a 3 x 1.5 mm ridge round the middle
  that sits in a channel inside the tire, so side loads in a turn cannot walk the tire off.
- The printed tire's bore is 0.5 mm under the rim diameter (`TIRE_FIT_INTERFERENCE`, a guess)
  so it grips by stretch alone, with no glue.

## Consequences

- +42 g of printed material over the O-ring wheel (two 21 g tires; the hub is about the same).
  Plastic total 254 g, still under the test's 500 g ceiling.
- Pitch-axis inertia is unchanged: the wheels are not on the casing.
- The tire is a bench-tuned part. Print one first, fit it, adjust the interference, then print
  the pair (`docs/bringup/component-measurements.md`).
- O-rings leave the BOM; TPU filament joins it.
- If the makerspace cannot print TPU, the fallback is the two-O-ring option above, not the
  hard printed tread.
