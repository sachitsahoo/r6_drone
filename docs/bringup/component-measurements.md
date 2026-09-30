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
| **Pitch motor outer diameter** | `PITCH_MOTOR_BORE` | Must be under **31.7 mm** or the drive band and chassis frame both shrink. A mislabelled 2804-class part is 35 mm. This is the single riskiest number. |
| **Printed shell mass** | reported by `cad/build.py` | The shell is ~73% of the rotating inertia. Every torque figure in `docs/theory/` is an estimate until this is weighed. |
| **Casing CoM offset + pendulum period** | `docs/theory/` | Inverting `f = sqrt(mgd/I)/2pi` converts the whole torque budget from estimate to measurement. Use the R8 datum hole. **ADR 0006 cannot be Accepted without this.** |
| **Camera horizontal FOV + resolution** | ADR 0006 finding 3 | The backlash-to-pixels figure (1 deg ~ 21 px) assumes ~90 deg across 1920 px. If the Wide module is meaningfully different, the argument that disqualifies a geared actuator changes size. |

---

## Mechanical

- [ ] Pitch motor: OD, length, mass, **mounting bolt circle radius and count**
      -> `PITCH_MOTOR_BORE`, `PITCH_MOTOR_LENGTH`, `PITCH_MOTOR_BOLT_RADIUS`,
      `PITCH_MOTOR_BOLT_COUNT`. The bolt pattern is a pure guess right now (4 at r=14.5).
- [ ] Pitch motor shaft: diameter and usable length -> sets how the pulley mounts
- [ ] N20 gearmotors: body diameter, **total length including gearbox**, shaft diameter and
      length, mass -> `AXIS_BOSS_BORE`, `bought_parts.wheel_motor_*`. Gearbox length varies
      enormously with ratio and drives the whole axis stack.
- [ ] Bearings: confirm OD / ID / width against 6704ZZ (20 x 27 x 4)
      -> `BEARING_OD`, `BEARING_ID`, `BEARING_WIDTH`, and `AXIS_BOSS_OD` follows
- [ ] GT2 belt: pitch, width, total thickness, tooth height
      -> `BELT_WIDTH`, `BELT_BACK_THICKNESS`, `BELT_TOOTH_HEIGHT`. These feed
      `DRIVE_PULLEY_CENTER_RADIUS`, so they move the motor position.
- [ ] Drive pulley: OD (pitch diameter, not flange), bore -> `DRIVE_PULLEY_OD`, and the
      ratio follows
- [ ] Slip ring: OD, length, circuit count, per-circuit current rating
      -> `SLIP_RING_ENVELOPE_*`. Current rating matters if the battery ends up in the chassis.

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
