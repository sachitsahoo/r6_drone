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

**Requirement:** a 10–15 mm axial band at one end of the casing where the **chassis does not
extend**, carrying an internal toothed track (or a smooth capstan surface) on the casing's
inner wall.

**Why:** ADR 0006 flagged belt routing as its main risk, because the casing's outer surface is
the impact surface and the 2–3 mm rotational clearance looked too tight for a belt. Giving the
belt its own axial band dissolves that: with no chassis in the band, the track can project
inward as far as it likes and radial clearance stops being the constraint. **Axial length is
the constraint instead, and it fits comfortably.**

**Axial budget** (165 mm wheel-to-wheel):

| Wheels | Available | Casing | Belt band | Chassis | Spare |
|---|---|---|---|---|---|
| 15 mm | 135 mm | 125 mm | 15 mm | 106 mm | 10 mm |
| 18 mm | 129 mm | 115 mm | 15 mm | 96 mm | 14 mm |
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

**The constraint is radial only, so axial displacement is free.** The axis is crowded with the
axle and the slip ring, but the IMU can sit on the centreline at a *different axial station* —
beside the hub rather than inside it — and still see `r ≈ 0`. This is the cheap way out of what
looks like an unresolvable packaging conflict.

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

## A concrete starting point

Ranges are right for analysis and wrong for sketching. This is one consistent set that
satisfies every constraint above, so the first CAD session is typing numbers rather than
re-deciding them. Reproduce with the derivation at the bottom of
`tools/pitch_inertia_budget.py`'s companion analysis.

| Dimension | Value | Note |
|---|---|---|
| Casing OD | **135 mm** | bottom of the 135–145 range; smaller is less inertia |
| Casing wall | **3.0 mm** | printable and robust; thin it later with ribs (R6) |
| Casing inner dia | **129 mm** | derived |
| Rotational clearance | **2.5 mm** | mid-range |
| Chassis OD | **124 mm** | derived; sits inside the stated 115–125 |
| Wheel width | **18 mm** each | |
| Casing length | **120 mm** | |
| Side clearance | **4.5 mm** each | |
| Belt band | **15 mm** | at one end, chassis absent here (R1) |
| Chassis length | **101 mm** | derived |
| **Total width** | **165 mm** | matches the target exactly |
| Belt band pitch dia | **129 mm** | the casing's inner wall in the band |
| Drive pulley | **10 mm** | |
| **Reduction** | **12.9:1** | the torque budget assumed ~13:1 |

Suggested fixtures, not derived from anything — adjust freely:

| Feature | Suggestion |
|---|---|
| Trim mass bosses (R4) | 6 x M3, at radius 55 mm, every 60 degrees |
| IMU boss (R2) | on the centreline, radial offset target under 3 mm |
| Slip ring envelope (R5) | reserve 15 mm diameter x 25 mm on the axis |
| Pendulum datum (R8) | one 3 mm hole at radius 50 mm, documented |

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
