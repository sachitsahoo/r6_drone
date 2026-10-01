# Mechanical requirements for the CAD

Written **before** the CAD exists, so these are design inputs rather than checks. Derived
from [ADR 0004](decisions/0004-pitch-axis-architecture.md) (casing rotates continuously),
[ADR 0005](decisions/0005-imu-placement.md) (IMU on the casing),
[ADR 0006](decisions/0006-pitch-actuator.md) (proposed belt drive), and the
[inertia and torque analysis](theory/pitch-axis-inertia-and-torque.md).

Ordered by **cost to retrofit**, highest first. R1 through R4 are cheap now and expensive
or impossible later.

Numbers are reproducible with `python3 tools/pitch_inertia_budget.py`.

---

## R1. Dedicate an axial band for the belt track

> **SUPERSEDED (2026-09-30) by ADR 0008: direct drive, no belt.** Kept for the record.

**Requirement:** a 10–15 mm axial band in which the **chassis diameter is locally reduced to
80 mm** and carries an outward-facing toothed track. The casing-mounted drive pulley reaches
inward to engage it.

**CORRECTION:** an earlier revision of this requirement put the track on the *casing's* inner
wall with the chassis absent from the band. That contradicts R3, which puts the motor in the
casing: if the track and the motor both ride on the casing they rotate together and nothing
moves. The track must be on the chassis.

**Why it fits:** the concern in ADR 0006 was that the casing's outer surface is the impact
surface, so the belt must engage internally, and 2.5 mm of rotational clearance is far too
little. The resolution is that **the chassis only needs full diameter at its two bearing
lands.** Between them it can be slim, which opens a deep annular pocket for both the drive
pulley and the pitch motor.

Radial stack-up, with a 10 mm pulley and a GT2 belt (1.4 mm back, 0.75 mm teeth) against a
casing inner radius of 65.0 mm:

| Track OD | Pulley tip radius | Gap to casing wall | Ratio |
|---|---|---|---|
| 90 mm | 57.1 mm | **7.4 mm** | **9.0:1** |
| 100 mm | 62.1 mm | 2.4 mm | 10.0:1 |
| 110 mm | 67.2 mm | does not fit | — |

**Use 80 mm, giving 8:1.** The table above only checks that the 10 mm *pulley* clears the
shell. The **28 mm motor body is coaxial with that pulley**, and at a 90 mm band it reaches
radius 66.2 mm against a wall at 65.0 mm — through the shell. The largest band that
fits a 28 mm motor with 2 mm of margin is 83.7 mm OD, so 80 mm and 8:1. Caught by
`cad/assembly.py`'s clearance report, not by inspection.

**Chassis frame diameter between the bearing lands** sets the motor pocket:

With an 80 mm band the motor's inner edge sits at radius 33.1 mm, so the frame must stay
under 62.3 mm OD to clear it. **Use 60 mm.**

**This is the one dimension the CAD must verify rather than inherit** — it depends on the
actual motor chosen, and it is the last place the packaging can bite.

**Axial budget** (165 mm wheel-to-wheel):

| Wheels | Available | Casing | Belt band | Chassis | Spare |
|---|---|---|---|---|---|
| 15 mm | 135 mm | 125 mm | 15 mm | 106 mm | 10 mm |
| 18 mm | 129 mm | 120 mm | 15 mm | 101 mm | 9 mm |
| 22 mm | 121 mm | 115 mm | 10 mm | 101 mm | 6 mm |

**Reduction available** from the band diameter against a small pulley:

| Band | 8 mm pulley | 10 mm | 12 mm | 16 mm |
|---|---|---|---|---|
| 125 mm | 15.6:1 | 12.5:1 | 10.4:1 | 7.8:1 |
| 130 mm | 16.2:1 | 13.0:1 | 10.8:1 | 8.1:1 |

A 10 mm pulley gives ~13:1, which is the ratio the torque budget assumed. **Target 10–13:1.**

## R2. IMU boss on the rotation centreline

**Requirement:** a mounting feature that puts the IMU within **3 mm radially** of the wheel
axis (5 mm tolerable), reached by a bridge or spider from the casing wall.

**Why:** an accelerometer offset radially from the rotation axis sees centripetal and
tangential acceleration it cannot distinguish from gravity. At a modest 10 rad/s:

| Radial offset | Apparent tilt error |
|---|---|
| 0 mm | 0.0° |
| 3 mm | 1.8° |
| 5 mm | 2.9° |
| 10 mm | 5.8° |
| 30 mm | 17.0° |

> **CORRECTED (2026-09-30) by ADR 0010.** The paragraph below was wrong. The casing turns a
> full turn relative to the chassis, and the chassis passes every axial station, so no casing
> part can reach the centreline at any station. The IMU now rings the spine's 5 mm waist at
> r = 4.75 mm, inside the "5 mm tolerable" limit above.

~~**The constraint is radial only, so axial displacement is free.** The axis is crowded with the
axle and the slip ring, but the IMU can sit on the centreline at a *different axial station* —
beside the hub rather than inside it — and still see `r ≈ 0`.~~

## R3. Put the pitch motor inside the casing

**Requirement:** mount the pitch motor and its pulley in the rotating casing, engaging a track
on the chassis (or on the casing's own band, driven from a chassis-fixed idler — either
kinematic arrangement works; this is about which side the motor is on).

**Why electrical, not mechanical:** with the motor in the casing, its phase wires stay inside
the casing alongside the MCU. With the motor in the chassis, three PWM phase currents plus
encoder signals would have to cross the slip ring, and high-frequency phase currents through
brush contacts mean noise and wear.

**Cost:** the motor joins the rotating mass.

| Motor | at r = 30 mm | at r = 45 mm |
|---|---|---|
| 30 g | +4% / +2% inertia | +9% / +4% |
| 40 g | +6% / +2% | +13% / +5% |
| 60 g | +8% / +4% | +19% / +8% |

(light corner / heavy corner). A 2–13% inertia penalty is a good trade for keeping switching
currents off the slip ring. **Mount it as close to the axis as the pulley geometry allows.**

**A benefit worth noting:** a belt drive puts the motor *off-axis*, which leaves the centreline
free for the axle, the slip ring and the IMU. A direct-drive actuator would have to be coaxial
and would compete with all three. This is an argument for the belt that the torque budget alone
did not surface.

## R4. Threaded bosses for trim mass, at known radii

**Requirement:** several threaded or press-fit bosses distributed around the casing's inner
wall at a documented radius, for adding or removing trim mass after assembly.

**Why:** whether the casing should be **balanced** (centre of mass on the axis) or deliberately
**bottom-heavy** (passive pendulum keeping the camera roughly upright) is a control design
decision that has not been made — and it has a sub-1 Hz consequence:

| CoM offset | Pendulum resonance | Continuous holding torque |
|---|---|---|
| 2 mm | 0.35–0.37 Hz | 3–7 mN m |
| 5 mm | 0.55–0.58 Hz | 9–18 mN m |
| 10 mm | 0.77–0.82 Hz | 17–36 mN m |
| 20 mm | 1.10–1.16 Hz | 34–73 mN m |

Designing the bosses in now keeps both options available and costs almost nothing. Retrofitting
balance adjustment into a closed printed shell is awkward.

## R5. Slip ring on the axis, 8–12 circuits

> **Under review (2026-09-30): ADR 0009 proposes a wire loop instead.** Vendor drag figures
> for small slip rings exceed the whole direct-drive torque budget.

**Requirement:** axial space on the centreline for a capsule slip ring carrying wheel motor
power and wheel encoder signals.

**Why:** ADR 0005 puts the MCU, Pi, camera and IMU all in the casing, so the only signals that
cross the rotating joint are the two wheel motors (power) and their encoders. That is roughly
8–12 conductors, all DC or low-rate, which is what a small capsule slip ring handles well.

**Do not route across the slip ring:** the IMU's SPI bus (1 kHz+), the camera's CSI ribbon, or
motor phase currents. All three are reasons R3 exists.

## R6. Ribbed shell, not uniformly thick

**Requirement:** achieve impact stiffness with ribs, and keep the wall between them as thin as
printing and sealing allow.

**Why:** the shell is **83–85% of the rotating inertia** — not the electronics. A thin wall at
66 mm radius puts every gram at the largest radius in the machine, so a gram removed from the
wall costs about **four times** the inertia of a gram removed at 33 mm. Total rotating mass is
only 174–370 g of the 700–900 g budget, so this is the single highest-leverage mass reduction
available, and it is geometric rather than a matter of component selection.

## R7. Camera aperture, and bearing support

**Requirement:** an aperture or transparent window for the camera, rotating with the casing;
and bearing support for the casing on the chassis at both ends.

**Why:** the camera looks radially outward through the impact surface, so the aperture is a
deliberate weak point in the structure that takes the hits. Reinforce its edge. Continuous
rotation also means the casing needs proper bearings rather than a sliding fit, or friction
will dominate the torque budget in a way none of the numbers above account for.

## R8. A datum feature for measuring inertia

**Requirement:** a way to hang the assembled casing off-axis repeatably — a hole, a flat, or a
documented fixture point.

**Why:** every torque figure in the analysis rests on an *estimated* inertia. The cheapest way
to get the real number is to hang the casing as a pendulum, time its period, and invert
`f = sqrt(m g d / I) / 2 pi`. That requires a known offset `d`, which requires a datum. Adding
one now converts the whole torque budget from an estimate into a measurement for the cost of a
hole.

---

## A note on derived dimensions

`CASING_ID`, `CHASSIS_OD` and the clearances below are **computed** in
`cad/parameters.py`, not chosen. Changing `CASING_WALL` moves all of them.

That happened once: thinning the wall from 3.0 to 2.5 mm to fix the mass budget took
`CASING_ID` from 129 to 130 and `CHASSIS_OD` from 124 to 125, and this document went stale
for several commits without anyone noticing. **`CHASSIS_OD` is now 125 mm, exactly at the
top of the owner's stated 115–125 range**, so further wall thinning pushes it out of range
and that is a decision, not a free optimisation.

Read the numbers out of `python3 cad/build.py --report` rather than from this table when
they matter.

## A concrete starting point

Ranges are right for analysis and wrong for sketching. This is one consistent set that
satisfies every constraint above, so the first CAD session is typing numbers rather than
re-deciding them. Reproduce with the derivation at the bottom of
`tools/pitch_inertia_budget.py`'s companion analysis.

| Dimension | Value | Note |
|---|---|---|
| Casing OD | **135 mm** | bottom of the 135–145 range; smaller is less inertia |
| Casing wall | **3.0 mm** | printable and robust; thin it later with ribs (R6) |
| Casing inner dia | **130 mm** | derived from OD 135 and a 2.5 mm wall |
| Rotational clearance | **2.5 mm** | mid-range |
| Chassis OD | **125 mm** | derived; **at the top of the stated 115–125**, so any
further wall thinning pushes it out of range |
| **Wheel OD** | **170 mm** | 17.5 mm ground clearance under the casing; 150 mm gave only 7.5 |
| Wheel width | **18 mm** each | |
| Casing length | **120 mm** | |
| Side clearance | **4.5 mm** each | |
| Belt band | **15 mm** | at one end, chassis absent here (R1) |
| Chassis length | **101 mm** | derived |
| **Total width** | **165 mm** | matches the target exactly |
| Belt band pitch dia | **80 mm** | on the chassis, not the casing — see R1's correction |
| Drive pulley | **10 mm** | |
| **Reduction** | **12.9:1** | the torque budget assumed ~13:1 |

Suggested fixtures, not derived from anything — adjust freely:

| Feature | Suggestion |
|---|---|
| Trim mass bosses (R4) | 6 x M3, at radius 55 mm, every 60 degrees |
| IMU boss (R2) | on the centreline, radial offset target under 3 mm |
| Slip ring envelope (R5) | reserve 15 mm diameter x 25 mm on the axis |
| Pendulum datum (R8) | one 3 mm hole at radius 50 mm, documented |

## Every part, as primitive shapes

Eight printed parts, built from **four shape types**: tube, disc, flat plate, cylinder.
Nothing here needs surfacing, lofting, or a single piece of organic geometry.

| # | Part | Base shape | Features to add |
|---|---|---|---|
| 1 | Casing shell | **tube** (cylinder minus cylinder) | camera aperture cutout; screw bosses at both ends |
| 2 | Casing end cap x2 | **disc** with centre bore | bearing counterbore; screw holes; one gets the pendulum datum hole (R8) |
| 3 | Chassis bearing disc x2 | **disc** with centre bore | bearing seat; standoff holes; wheel motor mount holes |
| 4 | Chassis drive band | **short tube**, 90 mm OD | plain flat band — the belt bonds onto it |
| 5 | Pitch motor bracket | **flat plate** | motor bore; screw holes; offset to set belt tension |
| 6 | IMU bridge | **flat strip** | two holes at the wall, a flat pad on the centreline (R2) |
| 7 | Camera mount | **flat plate** | four camera screw holes |
| 8 | Wheel x2 | **cylinder** with bore | O-ring groove for tread; flat or D for the shaft |

Optionally also: trim mass bosses (R4) are just threaded holes in part 1; the slip ring
envelope (R5) is just a bore in part 3. Neither is a separate part.

### Buy, do not model

Axle rod, bearings, standoffs, GT2 timing belt, 10 mm drive pulley, O-rings for tread, capsule
slip ring, N20 motor brackets, screws.

### Two tricks that delete the hard geometry

- **Bond a length of GT2 timing belt around the chassis drive band, teeth facing outward.**
  That belt *is* the toothed rack. Gear teeth are the only genuinely difficult shape in this
  whole design, and this removes them entirely — part 4 becomes a plain smooth band.
- **Cut a groove in each wheel and drop in an O-ring** for traction. No tread pattern to model,
  and the rubber is replaceable when it wears.

### If you build the bench rig first

Three parts, two shape types:

| Part | Base shape |
|---|---|
| Base plate | **rectangle** with holes — or a scrap of plywood or aluminium, zero CAD |
| Upright bearing block x2 | **rectangular block** with a bore |
| Dummy inertia disc | **disc** with a bore and bolt holes at a known radius |

## Modeling notes

The geometry here is deliberately simple, and it is worth keeping it that way.

- **The casing is a revolve.** Sketch the cross-section once — an annulus with end caps and a
  stepped band — and revolve it. This is the easiest class of shape in any CAD package. There
  is no surfacing, no lofting, and no organic geometry anywhere in this design.
- **Drive the sketch from named parameters**, not typed literals. Every number in the table
  above is likely to move at least once; if `casing_od` is a variable, that costs nothing, and
  if it is typed into forty places it costs an afternoon.
- **Model separate simple parts and assemble them.** One monolithic part that tries to be the
  shell, the belt track, the IMU bridge and the camera mount at once is where CAD gets
  genuinely hard. Four boring parts are easier than one clever one.
- **Skip fillets, chamfers and cosmetics** until the thing works. They make every later edit
  more expensive and they are not load-bearing on any question this project is asking.
- **Print a 20 mm axial slice first.** A short section through the belt band and the clearance
  gap tests the two fits that actually matter — belt engagement and rotational clearance — for
  a few minutes of printing instead of several hours. Iterate on the slice, then print the full
  120 mm once.
- **Do not model the electronics.** Bounding boxes with mounting hole positions are enough, and
  detailed component models mostly slow the assembly down.

## What to measure once parts exist

Feeds back into `tools/pitch_inertia_budget.py`:

1. Weigh the printed shell — 85% of the inertia estimate.
2. Weigh each casing component.
3. Find the CoM offset: hang the casing, see where it balances.
4. Time the pendulum period with R8's datum, and invert for the true `I`.
5. Confirm the camera's actual horizontal FOV and resolution.
6. Measure the transmission's actual backlash in degrees at the casing.

Record results in `docs/bringup/`.
