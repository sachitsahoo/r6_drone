# 0008 — Reduced scale, and direct drive

- **Status:** Accepted (2026-09-30). Both decisions stand. **Amended by
  [0011](0011-pitch-motor-and-encoder.md)** and [0012](0012-power-and-electronics-placement.md),
  marked inline:
  - the motor is a DM3505 (40 mm, hollow shaft), not a 2208, because ADR 0010 found the
    shaft must pass through it
  - the 22.5 mN m budget omitted the electronics; ADR 0011 recomputes it (17.8–25.8 mN m
    trimmed, 38.8 untrimmed, against the DM3505's 90 rated)
  - "thermal evaporates" was overstated: ~0.5 W held untrimmed, ~0.02 W trimmed
  - the battery is a 3S 300 mAh slim pack, 60.5 × 16 × 11.5 (ADR 0012)
- **Supersedes:** [0006](0006-pitch-actuator.md) (belt/capstan reduction)
- **Related:** [0004](0004-pitch-axis-architecture.md), [0009](0009-wire-crossing.md)

## Context

The owner supplied a reference image of a throwable two-wheeled robot, noted the original
165 mm wheel-to-wheel was a starting guess, and asked for something longer. Two decisions
came out of working that through, and the second was a surprise.

## Decision 1: casing 70 x 182 mm, wheels 105 mm, 214 mm overall

Proportions matching the reference (length / wheel diameter ~2.0), at roughly Throwbot scale.

An important scaling result drove the shape. **Shell mass grows with radius and inertia with
mass times radius squared, so pitch inertia scales as r^3, while interior volume only scales
as r^2.** A longer, thinner casing therefore holds the same volume for markedly less
inertia. Stretching alone was the expensive option:

| Casing | Overall | L/wheel | Rotating I | Axis torque |
|---|---|---|---|---|
| 135 x 120 (original) | 165 mm | 0.97 | 0.680e-3 | 65.4 mN m |
| 135 x 200 (stretched only) | 245 mm | 1.44 | **1.060e-3** | 100.4 mN m |
| 100 x 260 (thinned too) | 302 mm | 2.24 | 0.573e-3 | 67.4 mN m |
| **70 x 182 (chosen)** | **214 mm** | **2.04** | **0.133e-3** | **22.5 mN m** |

Scaling every length by `k` drops inertia by roughly `k^4`, because both the mass and the
radius shrink. Going from 302 mm to 214 mm cut rotating inertia by a factor of 4.3.

## Decision 2: direct drive, superseding ADR 0006

**The motor drives the casing directly. No belt, no toothed band, no pulley, no tensioner.**

ADR 0006 argued for a zero-backlash belt reduction of 8:1. That argument was correct at
302 mm and is wrong at 214 mm, because two things change at once and the motor does not
shrink with the robot:

**The belt stops fitting.** A 28 mm motor plus belt needs more bore than a small casing has:

| Casing OD | Belt ratio available |
|---|---|
| 100 mm | 4.5:1 |
| 85 mm | 3.0:1 |
| 70 mm | 1.5:1 |
| 60 mm | impossible |

**And direct drive becomes easy.** At 70 x 182 the axis needs 22.5 mN m, which through
`Kt = 9.549/90 = 0.106 N m/A` is **0.21 A against a 2.5 A driver — 12x margin.**

> **Amended (ADR 0011):** 22.5 mN m was I·α for a 10° correction in 100 ms plus a 10 mm
> sideways CoM offset, with the contents guessed at 35% of the shell. With the real
> electronics it is 38.8 mN m untrimmed. The motor is now a DM3505 at 0.08 N m/A, so
> trimmed it draws 0.22–0.32 A: still direct drive, 7.7–11.2x driver margin.

So the reduction goes from necessary to pointless, and direct drive now wins on every axis
ADR 0006 argued:

- **Backlash**: zero by construction. This was the finding that disqualified gearing, and
  direct drive has always satisfied it; the objection was only ever torque.
- **Thermal**: ADR 0006's strongest argument for the belt was that direct drive needed
  0.600 A against 0.075 A, and 64x the I^2R inside a sealed printed casing. At this scale
  direct drive needs 0.21 A, so that argument evaporates.
  *(Corrected by ADR 0011: it does not evaporate, but it stays modest. The DM3505 holding an
  untrimmed casing dissipates ~0.5 W; trimmed, ~0.02 W.)*
- **Parts**: deletes the drive band, the pulley, the tensioner and the motor bracket.
- **Packaging**: frees the annular pocket that the motor and belt occupied.

## Consequences

- **ADR 0006 is superseded, not amended.** Its reasoning was sound for the machine it was
  written about. It stays in the record because the argument it makes about backlash landing
  inside the measurement remains true and still justifies rejecting a geared actuator.
- **The motor becomes coaxial with the wheel axis**, so it competes for the centreline with
  the wire crossing. The IMU is unaffected: ADR 0005's constraint is radial only, so it can
  sit on the axis at a different axial station.
  *(Resolved by ADR 0010: the chassis is a spine, the motor is hollow and wheel A's shaft
  runs through it, and the wire loop rings the spine at end B. **The IMU claim was wrong**:
  no casing part can be on the axis at any station; it sits 3.8 mm beside the waist.)*
- **The battery must shrink.** A 60 x 32 x 18 pack does not fit a 70 mm casing; something
  nearer 45 x 25 x 12 does. That costs run time, and run time has not been budgeted at all.
  *(Decided by ADR 0012: 3S is forced by the motor drivers, and a slim 3S 300 mAh pack,
  60.5 × 16 × 11.5 mm, fits along the axis. Estimated 35–55 minutes.)*
- **Friction now matters.** At 302 mm the torque budget was 67 mN m and bearing or slip ring
  drag was noise. At 22.5 mN m it is not — see ADR 0009.
- The reduction ratio disappearing means the motor sees the casing's full inertia directly.
  At 0.133e-3 kg m^2 that is fine, but it removes the `n^2` reflected-inertia cushion that
  made the belt forgiving of a poor controller.
- Every dimension in `cad/parameters.py` changes, and four printed parts are deleted. The
  CAD refactor is substantial and has not been done yet.

## What must be measured before this is trusted

Unchanged from ADR 0006, and still gating: the printed shell mass, and the casing's centre
of mass offset and pendulum period. Every torque figure above is computed from geometry and
assumed material density.
