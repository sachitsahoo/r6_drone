# Mechanical requirements

Design inputs for the CAD, ordered by **cost to retrofit**, highest first. Derived from
[ADR 0004](decisions/0004-pitch-axis-architecture.md) (casing rotates about the wheel axis),
[ADR 0005](decisions/0005-imu-placement.md) (IMU on the casing),
[ADR 0008](decisions/0008-reduced-scale-direct-drive.md) (70 × 182, direct drive),
[ADR 0010](decisions/0010-direct-drive-axial-layout.md) (spine chassis),
[ADR 0011](decisions/0011-pitch-motor-and-encoder.md) (motor and balance),
[ADR 0012](decisions/0012-power-and-electronics-placement.md) (power), and the
[inertia and torque analysis](theory/pitch-axis-inertia-and-torque.md).

**The numbers live in code, not here.** Every dimension is in `cad/parameters.py`, tagged
OWNER / DERIVED / ASSUMPTION; the budget is `tools/pitch_inertia_budget.py`. This document
says *why*. An earlier revision kept a dimension table here and it went stale within a day.

---

## R1. ~~Axial band for a belt track~~ — retired

Retired by ADR 0008: direct drive, no belt. The requirement existed to give an 8:1 belt
reduction somewhere to run. At 70 mm the belt no longer fits, and direct drive no longer
needs one.

## R2. IMU as close to the rotation axis as the spine allows

**Requirement:** the IMU within **5 mm radially** of the wheel axis, with the error budgeted.
Currently **3.8 mm**: an off-the-shelf breakout lying chip-side-down beside the spine's 5 mm
waist (ADR 0010).

**Why:** an accelerometer offset from the rotation axis sees centripetal (`w^2 r`) and
tangential (`alpha r`) acceleration it cannot tell from gravity. At 3.8 mm that is 0.6° at
5 rad/s and **4.4° at 200 rad/s²**, the tangential term being the larger. The gyro is
unaffected at any radius.

**Not achievable: r = 0.** An earlier revision said the IMU could sit on the centreline "at a
different axial station". It cannot. The casing turns a full relative turn and the chassis
passes every axial station, so no casing part can reach the axis anywhere (ADR 0010).

## R3. The pitch motor's windings stay in the casing

**Requirement:** bolt the motor's **stator** (windings) to the casing, at end cap A, and its
**rotor** to the chassis, at the cup floor.

**Why electrical, not mechanical:** the power stage is in the casing (ADR 0012). Bolted this
way round, the three phase leads stay inside the casing next to it. The other way round they
would cross the wire loop: switching currents in a flexing bundle running alongside the IMU's
SPI lines. The encoder follows the same rule: sensor on the casing side, magnet on the rotor.

The motor's position is otherwise set by the topology: coaxial at end A, hollow-shaft, with
wheel A's shaft running through it (ADR 0010).

## R4. Trim mass bosses, at known radii

**Requirement:** six M3 bosses on the shell's inner wall, every 60°, at the trim station
(`Z_TRIM_BOSSES`), for adding mass after assembly. **Trimming the sideways centre-of-mass
offset to 2 mm or less is recommended.**

**Why:** holding gravity is the largest term in the torque budget (ADR 0011). Untrimmed, at
10 mm sideways, the DM3505 runs at 43% of its rated torque and dissipates about 0.5 W for as
long as the camera holds level. Trimmed to 2 mm, that drops to 20–29% and about 0.02 W. Not
needed to stay within rating, but it is cheap margin for the drag terms nobody has measured.

The *vertical* offset — balanced versus bottom-heavy — is a separate, still-open control
decision. A bottom-heavy casing has a pendulum resonance of about 1.9 Hz at 10 mm, inside the
loop. The bosses keep both options open.

## R5. ~~Slip ring on the axis~~ — replaced by a wire loop

Replaced by ADR 0009. A capsule slip ring has nowhere to sit, since the spine fills the
centreline, and its specified drag (20–70 mN m) is the whole torque budget. Instead:

**Requirement:** room for a flexible wire loop around the spine near end B
(z ≈ 142–166 mm), with casing contents kept outside about r = 20 mm there. It carries about
10 conductors: the wheel motor leads and their encoder lines, nothing else (ADR 0012).

**Do not route across the joint:** the IMU's SPI bus, the camera's CSI ribbon, or the motor's
phase leads (R3).

## R6. Ribbed shell, not uniformly thick

**Requirement:** get impact stiffness from ribs, and keep the wall between them as thin as
printing allows.

**Why:** the shell is **67% of the rotating inertia** (107 g at r ≈ 33 mm). A gram removed
from the wall saves about three times the inertia of a gram removed from a board at
r ≈ 20 mm. Not done yet: the shell is still a plain 2.5 mm tube.

## R7. Camera aperture, and bearing support at both ends

**Requirement:** an aperture or window for the camera, rotating with the casing; and real
bearings between casing and chassis at both ends.

**Why:** the aperture is a deliberate weak point in the surface that takes the hits, so its edge
needs reinforcing. Real bearings matter because at an ~18–26 mN m budget a sliding fit would
use up much of it. End A uses a 6709ZZ on the cup wall, so impacts bypass the gimbal motor's
own small bearings. End B uses a 6704ZZ on the spine's boss.

## R8. A datum feature for measuring inertia and balance

**Requirement:** a 3.2 mm hole at a known radius on end cap B (`PENDULUM_DATUM_*`).

**Why:** hanging the casing from a known offset and timing its swing gives the real inertia,
`f = sqrt(m g d / I) / 2 pi`, converting the budget from estimate to measurement. It also finds
the centre of mass, which R4's trimming needs.

---

## The parts

Seven printed parts. `cad/parts.py` is the source; this is the overview.

| # | Part | Shape | Rotates with |
|---|---|---|---|
| 1 | Casing shell | tube, with flanges, aperture, trim bosses | casing |
| 2 | End cap A | stepped disc: bearing groove, stator bolt circle, shaft bore | casing |
| 3 | End cap B | rim-hub-spoke disc: bearing seat, pendulum datum | casing |
| 4 | Chassis spine | solid of revolution: cup, two motor pockets, waist, boss | chassis |
| 5 | IMU bridge | arm plus carrier plate beside the waist | casing |
| 6 | Camera mount | flat plate | casing |
| 7 | Wheel × 2 | rim, hub, spoke web, tire-locating ridge | its wheel |
| 8 | Tire × 2 (TPU) | ring, staggered transverse tread, inner channel (ADR 0017) | its wheel |

The spine is drawn as one solid, but it is built as two printed ends joined by a bought rod at
the waist (M5 threaded rod or a 5 mm tube).

**Bought, not modelled:** the DM3505, 6709ZZ and 6704ZZ bearings, the waist rod, wheel A's
3 mm shaft extension and coupler, screws and heat-set inserts.
`cad/bought_parts.py` holds envelopes for the electronics; `docs/bom.md` is the shopping list.

## Modelling notes

- **Drive everything from named parameters.** The axial stations are derived from part
  lengths, so measuring a real N20 and changing one number moves everything downstream.
- **The interference sweep checks every relative angle**, not one pose. Parts that turn
  relative to each other can collide at an angle the assembled pose never shows.
- **Print a short axial slice first:** through end cap A, the cup and the 6709, to test the
  bearing fits and the 0.5 mm running gaps before committing to a full print. FDM can close a
  0.5 mm gap entirely.
- Skip fillets and cosmetics until it works.

## What to measure once parts exist

Feeds `tools/pitch_inertia_budget.py` and `cad/`. The checklist is
`docs/bringup/component-measurements.md`.

1. Weigh the shell and every casing component.
2. Find the centre of mass both ways with the R8 datum; trim the sideways offset, ideally to ≤ 2 mm.
3. Time the pendulum swing, and invert it for the real inertia.
4. Measure the wire loop's spring torque per turn, and how many turns it tolerates.
