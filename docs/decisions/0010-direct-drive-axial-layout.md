# 0010 — Direct-drive axial layout: a spine chassis, and the IMU beside the axis

- **Status:** **ACCEPTED** by the owner, 2026-10-03 (reviewed with 0009–0012 together). Proposed 2026-09-30; the
  CAD implements it.
- **Amends:** [0005](0005-imu-placement.md) (IMU mounting geometry) and R2 in
  `docs/mechanical-requirements.md`
- **Related:** [0008](0008-reduced-scale-direct-drive.md) (direct drive),
  [0009](0009-wire-crossing.md) (wire crossing), [0011](0011-pitch-motor-and-encoder.md)
  (the motor that fits this layout), [0012](0012-power-and-electronics-placement.md)

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
- A connected region spanning the casing end to end separates the axis (r = 0) from the shell
  (r = R). A casing part on the axis could never connect to the shell.

So "IMU on the centreline, at an axial station clear of the hub" (ADR 0005, R2) is impossible
for **any** chassis layout, not just this one. Bounding the rotation does not help unless
travel is cut below a full turn, which gives up the leveling ADR 0004 exists for.

The first CAD already contained the violation. Its chassis discs were joined by four standoffs
at r = 35 mm, and the IMU bridge ran from the wall to the axis at z = 40, straight through where
the standoff at (35, 0) would be. The sweep passed it for two reasons: the standoffs were holes,
never modelled as rods, and the sweep checked a single pose.

## Decision

### 1. The chassis is a spine on the axis

One solid of revolution: a cup at end A, a pocket for each wheel motor, a 5 mm waist, and a
boss at end B. No discs, no standoffs. Whatever radius the chassis takes at a station, the
casing loses at that station, so the chassis is kept as thin as possible everywhere.

The waist is a bought rod joining two printed ends: **M5 threaded rod** (cheapest; nuts clamp
the ends together) or a **5 mm brass or aluminium tube** (lets wheel motor A's wires pass
through it).

### 2. Hollow-shaft pitch motor in a cup at end A

The motor's stator bolts to end cap A and its rotor bell to the cup floor. That way round, the
windings and their phase leads stay in the casing with the power stage and never cross the
wire loop. Wheel A's drive
shaft has nowhere to go except **through the motor's centre**, so the motor must be
hollow-shaft. ADR 0011 selects the DM3505 (40 mm, 8.5 mm bore). The casing's end-A bearing,
a **6709ZZ (45 × 55 × 6)**, sits on the outside of the cup wall, so impact loads go through a
real bearing rather than through the gimbal motor's small internal ones. End B keeps a 6704ZZ
on the spine's boss.

### 3. The IMU lies beside the spine's waist

An off-the-shelf breakout lies flat, running along the waist with its **chip side facing the
rod**, on a carrier plate reached from the casing wall. Centred over the rod, the chip's
distance from the axis is just the stack-up from the rod's surface, **wherever the chip sits on
the board**:

```
r = waist radius 2.5 + running gap 0.75 + chip-side height 1.0 - half the chip 0.46 = 3.8 mm
```

So no custom PCB is needed (CLAUDE.md puts those out of scope). The cost: nothing on the chip
side may be taller than about 1 mm, so headers go on the back or are replaced by wires.

### 4. The interference check sweeps relative rotation

Parts are tagged with the body they move with (casing, chassis, each wheel). Pairs in different
bodies are checked over a full relative turn at 5° steps, unless their (r, z) extents are
disjoint, which proves them clear at every angle exactly. A regression test rebuilds the first
design's hidden collision and asserts the sweep catches it.

## Options considered

| Option | Why not |
|---|---|
| Keep standoffs, cut travel below a full turn and pass parts through the dead zone | Loses leveling for some landings; ADR 0009's wire loop and unwind lose their purpose; every casing part would have to share one narrow gap |
| Split the chassis into two halves, one pitch motor each | Two actuators, two loops, and the halves can twist relative to each other |
| Large-bore motor mid-spine, spine passing through it | Needs a ≥ 16 mm bore: GM4108-class, 47 mm OD, far heavier |
| IMU on a ring-shaped board around the waist | A custom PCB, which is out of scope, and it puts the chip further out (4.75 mm) than the flat layout |

## Error budget at 3.8 mm

Gyro rate is the same everywhere on a rigid body, so the offset does not touch the inner loop.
It corrupts only the accelerometer's gravity reference:

| Term | Condition | Apparent tilt |
|---|---|---|
| centripetal `w^2 r` | 2 rad/s, normal stabilizing | 0.1° |
| centripetal | 5 rad/s, hard bump | 0.6° |
| centripetal | 10 rad/s, tumble (loop disarmed, ADR 0009) | 2.2° |
| **tangential `alpha r`** | **200 rad/s², aggressive correction** | **4.4°** |

**The tangential term is the larger one**, and it shows up exactly when the stabilizer is
working hardest. It is also the easiest to remove: its direction is known (tangential), and
alpha is the derivative of the gyro rate the loop already has. Correcting it is an estimator
change, so owner-reviewed. This ADR only establishes that the term exists and is about 4°.

## Consequences

- **ADR 0005's placement stands; its mounting geometry does not.** It now reads "beside the
  spine's waist, 3.8 mm". Its claim that every millimetre gives a *quadratic* reduction was
  wrong: both terms are linear in r and quadratic only in rate.
- **Wheel A needs a ~48 mm shaft extension** (3 mm rod plus a coupler), since the N20's own
  shaft is ~10 mm and wheel motor A sits behind the pitch motor.
- **The chassis carries nothing but the wheel motors.** ADR 0012 puts every board and the
  battery in the casing.
- Measured from the geometry: 140 g of rotating plastic at **0.136e-3 kg m²**, within 2% of
  ADR 0008's 0.133e-3. The budget with electronics is in ADR 0011.

## What remains open after acceptance

1. Whether a 5 mm rod survives a throw as the only structure between the wheels. Cheap to
   find out: if it bends, the rod gets bigger and the IMU moves out by the same amount.
2. The real breakout's chip-side component height, which sets the offset directly.
3. The IMU offset as built, measured, since any analytical correction needs it.
