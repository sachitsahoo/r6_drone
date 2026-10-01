# 0010 — Direct-drive axial layout: a spine chassis, and the IMU beside the axis

- **Status:** **PROPOSED** (2026-09-30). The layout was approved by the owner in
  conversation and the CAD implements it; this write-up, including the tangential-error
  finding, awaits review. The hollow-shaft motor it depends on has **not** been sourced.
- **Amends:** [0005](0005-imu-placement.md) (IMU placement) and R2 in
  `docs/mechanical-requirements.md`
- **Related:** [0008](0008-reduced-scale-direct-drive.md) (direct drive),
  [0009](0009-wire-crossing.md) (wire crossing)

## Context

ADR 0008 made the pitch motor coaxial with the wheel axis. Working out where it actually goes
turned up a geometric fact the whole design had been violating, and one the CAD's interference
check could not see.

## Finding: nothing on the casing can reach the axis

The casing turns through at least ±180° relative to the chassis — a full relative turn, needed
to bring the camera level from any landing orientation (ADR 0009). So every chassis feature
sweeps a complete ring about the axis. Draw the machine in (r, z), radius against axial
position:

- The chassis is one rigid body that has to reach wheel A and wheel B, so in (r, z) it is a
  connected region that crosses **every** axial station of the casing.
- The casing must stay outside that region at every station, or it collides at some angle.
- A connected region that spans the casing end to end separates the axis (r = 0) from the shell
  (r = R). A casing part on the axis could never connect to the shell.

So "IMU on the centreline, at an axial station clear of the hub" — ADR 0005 and R2 — is
impossible for any chassis layout, not just this one. Bounding the rotation does not help unless
travel is cut below a full turn, which gives up the leveling that ADR 0004 exists for.

The first CAD already contained the violation. Its chassis discs were joined by four standoffs
at r = 35 mm, and the IMU bridge ran from the wall to the axis at z = 40, straight through where
the standoff at (35, 0) would be. The sweep passed it for two reasons: the standoffs were holes,
never modelled as rods, and the sweep checked a single pose.

## Decision

### 1. The chassis is a spine on the axis

One solid of revolution: a cup at end A, a pocket for each wheel motor, a 5 mm waist, and a
boss at end B. No discs, no standoffs. Whatever radius the chassis claims at a station, the
casing loses at that station, so the chassis is kept as thin as it can be, everywhere.

### 2. Hollow-shaft pitch motor in a cup at end A

The motor's stator bolts to the cup floor; its rotor bell bolts to end cap A. Wheel A's drive
shaft has nowhere to go except **through the motor's centre**, so the motor must be hollow-shaft
with a bore larger than the 3 mm shaft. The casing's end-A bearing (61807, 35 × 44 × 5) sits on
the outside of the cup wall, so impact loads go through a real bearing rather than the gimbal
motor's small internal ones.

### 3. The IMU rings the spine's waist

The IMU pad has a hole the 5 mm waist passes through, and the sensor sits at the hole's edge:
**4.75 mm from the axis**. R2's target becomes "under 5 mm, with the error budgeted".

### 4. The interference check sweeps relative rotation

Parts are tagged with the body they move with (casing, chassis, each wheel). Pairs in different
bodies are checked over a full relative turn at 5° steps, unless their (r, z) extents are
disjoint, which proves them clear at every angle exactly. A regression test reproduces the
first design's hidden collision and asserts the sweep catches it.

## Options considered

| Option | Why not |
|---|---|
| Keep standoffs, cut travel below a full turn and pass parts through the dead zone | Loses leveling for some landings; ADR 0009's wire loop and unwind lose their purpose; every casing part would have to share one narrow gap |
| Split the chassis into two halves, one pitch motor each | Two actuators, two loops, and the halves can twist relative to each other |
| Large-bore motor mid-spine, spine passing through it | A ~10 mm+ bore needs a much bigger motor: more mass, more cogging, inside the loop being measured |
| Frameless motor (stator on the spine, magnets in the casing) | Custom magnetics. Not a portfolio-scale part |

## Error budget at 4.75 mm

Gyro rate is the same everywhere on a rigid body, so the offset does not touch the inner loop.
It corrupts the accelerometer's gravity reference:

| Term | Condition | Apparent tilt |
|---|---|---|
| centripetal `w^2 r` | 2 rad/s, normal stabilizing | 0.1° |
| centripetal | 5 rad/s, hard bump | 0.7° |
| centripetal | 10 rad/s, tumble (loop disarmed, ADR 0009) | 2.8° |
| **tangential `alpha r`** | **200 rad/s^2, aggressive correction** | **5.5°** |

**The tangential term is the larger one,** and it shows up exactly when the stabilizer is working
hardest. It is also the easiest to remove: its direction is known (tangential), and alpha is
the derivative of the gyro rate the loop already has. Correcting it is an estimator change and
so owner-reviewed; this ADR only establishes that the term exists and is about 5°.

## Consequences

- **ADR 0005's placement stands; its geometry does not.** Its "Mounting constraint" now reads
  "as close as the spine's waist allows". Its claim that every millimetre gives a *quadratic*
  reduction is wrong: both terms are linear in r and quadratic only in rate.
- **The waist is a bought rod**, so the spine is two printed ends plus a rod in reality. The
  rod's diameter sets the IMU offset directly.
- **A custom ring breakout is needed** to put the chip at the hole's edge. Off-the-shelf
  breakouts put the chip mid-board, which would add several millimetres.
- **Wheel A needs a ~55 mm shaft extension**, since the N20's own shaft is ~10 mm.
- **The chassis has nowhere for electronics.** The TB6612 and any chassis-side battery have no
  platform. Either they move to the casing, which adds circuits to ADR 0009's wire loop, or the
  spine grows a platform at end B, which costs casing room there.
- **Wheel motor A's wires** have to pass the waist on their way to wherever the wire loop sits.
  A 5 mm tube with a 3 mm bore is tight for six conductors.
- Measured from the new geometry: 138 g of rotating plastic at **0.135e-3 kg m^2**, within 2% of
  ADR 0008's 0.133e-3. Plastic only; the electronics have not been added.

## What must be verified

1. **That a 28 mm hollow-shaft gimbal motor with a bore of 5 mm or more can actually be bought.**
   Not verified. If it cannot, this layout needs a larger motor and cup, or option 3 above.
2. That a 5 mm rod survives a throw as the only structure between the two wheels.
3. The IMU offset as built, measured, since it feeds any analytical correction.
