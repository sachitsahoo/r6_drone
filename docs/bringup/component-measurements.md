# Component measurements

The CAD currently runs on **26 dimensions tagged ASSUMPTION** and a handful tagged LISTING.
This is the checklist for replacing them with calipers, and where each number goes.

Workflow: measure, change the number in `cad/parameters.py` or `cad/bought_parts.py`,
re-run `python3 cad/assembly.py`. Nothing else needs touching — every dimension is derived
from those two files, and the interference sweep covers all pairs.

## Why this ordering

Some measurements unblock design decisions; most just improve accuracy. The ones marked
**GATING** are holding up something specific.

---

## GATING — these unblock decisions

| Measure | Goes to | Why it gates |
|---|---|---|
| **Pitch motor hollow bore + OD** | `PITCH_MOTOR_HOLLOW_BORE`, `PITCH_MOTOR_OD` | The bore must pass wheel A's 3 mm shaft with clearance (ADR 0010) -- a solid-shaft motor makes the layout impossible. OD sets the cup, and the cup's OD is the end-A bearing's bore. A 2804-class part is 35 mm. Single riskiest part to buy. |
| **Printed shell mass** | reported by `cad/build.py` | The shell is ~73% of the rotating inertia. Every torque figure in `docs/theory/` is an estimate until this is weighed. |
| **Casing CoM offset + pendulum period** | `docs/theory/` | Inverting `f = sqrt(mgd/I)/2pi` converts the whole torque budget from estimate to measurement. Use the R8 datum hole. **ADR 0006 cannot be Accepted without this.** |
| **Camera horizontal FOV + resolution** | ADR 0006 finding 3 | The backlash-to-pixels figure (1 deg ~ 21 px) assumes ~90 deg across 1920 px. If the Wide module is meaningfully different, the argument that disqualifies a geared actuator changes size. |

---

## Mechanical

- [ ] Pitch motor: OD, length, mass, hollow bore, **both bolt circles (rotor bell and
      stator base) and their counts** -> `PITCH_MOTOR_OD`, `PITCH_MOTOR_LENGTH`,
      `PITCH_MOTOR_HOLLOW_BORE`, `PITCH_MOTOR_ROTOR_BOLT_RADIUS`,
      `PITCH_MOTOR_STATOR_BOLT_RADIUS`, `PITCH_MOTOR_BOLT_COUNT`. Both patterns are guesses.
- [ ] N20 gearmotors: body diameter, **total length including gearbox**, shaft diameter and
      length, mass -> `WHEEL_MOTOR_OD`, `WHEEL_MOTOR_LENGTH`, `SPINE_BORE`. Gearbox length
      varies enormously with ratio and sets every axial station at both ends of the spine.
- [ ] Bearings: confirm 6704ZZ (20 x 27 x 4) at end B and 61807 (35 x 44 x 5) at end A
      -> `BEARING_*`, `BEARING_A_*`. The cup wall and boss follow.
- [ ] Spine waist rod: OD and material -> `SPINE_WAIST_OD`. It sets the IMU's radial
      offset directly (`IMU_RADIAL_OFFSET`), so thinner is better until it bends.
- [ ] Wheel A shaft extension and coupler: diameter, length (~55 mm needed)
- [ ] Slip ring, only if ADR 0009's wire loop is rejected: OD, length, circuit count, and
      **friction torque** -> `SLIP_RING_ENVELOPE_*`. Datasheet drag exceeds the budget.

## Electronics

- [ ] Pi Zero 2 W: confirm 65 x 30, **height with connectors fitted**
- [ ] Camera Module 3 Wide: board outline, **lens barrel height**, mounting hole spacing
      -> `CAMERA_MOUNT_HOLE_SPACING_*`, `CAMERA_APERTURE_*`
- [ ] MCU board: outline and height with headers. Currently modelled as a Nucleo-G474RE at
      70 x 82 x 20, which fits with **1.2 mm** to spare at the casing wall. If you end up
      with a bare module instead, this gets much easier.
- [ ] ICM-42688-P breakout: outline, hole spacing -> `IMU_PAD_SIZE`, `IMU_MOUNT_HOLE_DIA`
- [ ] AS5600 breakouts x2: outline, and the **magnet working distance** from the datasheet.
      That sets how close the sensor must sit to the magnet on the chassis boss.
- [ ] FOC driver: outline and height
- [ ] INA226 breakout: outline
- [ ] Battery: dimensions and mass. Its **placement is still undecided** — in the casing it
      adds the heaviest single item to the rotating inertia; in the chassis it puts high
      current across the slip ring (ADR 0005).

## Masses

Weigh everything. The budget is 700–900 g and the current estimate is ~651 g (395 g plastic
plus ~256 g of bought parts), but most of the component masses are guesses.

---

## After measuring

1. Update `cad/parameters.py` and `cad/bought_parts.py`, changing each `GUESS` or `LISTING`
   tag to `MEASURED`.
2. Run `python3 cad/assembly.py` — the sweep covers every pair, including the bought-part
   envelopes.
3. Run `pytest tests/python/test_cad.py` — several invariants are asserted there, including
   that the motor fits radially and axially and that the chassis OD stays inside the
   owner's stated 115–125 range.
4. Place the component envelopes in `cad/assembly.py` at their intended positions. The fit
   check in this repo confirms each board fits the pocket *individually*; it does not check
   they fit **together**, or that they clear the pitch motor at 150 deg and the drive band.
   That is what the sweep is for once they are placed.
