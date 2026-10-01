# Component measurements

The CAD runs on dimensions tagged ASSUMPTION, LISTING or GUESS in `cad/parameters.py` and
`cad/bought_parts.py`. This is the checklist for replacing them with calipers, and where each
number goes.

Workflow: measure, change the number, re-run `python3 cad/assembly.py` and
`pytest tests/python/test_cad.py`. Every axial station is derived from part lengths, so one
change moves everything downstream, and the sweep checks every pair at every reachable angle.

---

## GATING — these unblock decisions

| Measure | Goes to | Why it gates |
|---|---|---|
| **DM3505 through-bore** | `PITCH_MOTOR_HOLLOW_BORE` | The datasheet gives ⌀8.5 on the rotor face; confirm it runs through the base, since wheel A's shaft must pass (ADR 0011). Also confirm both bolt circles (⌀15 rotor, ⌀33.5 base). |
| **Encoder fit** | ADR 0011 | The MA732-class sensor and ring magnet have to fit beside the motor. If the bundled AS5048A variant turns out to leave the bore open, that is simpler. |
| **Casing centre of mass, both directions** | `docs/theory/` | Trim the sideways offset, ideally to ≤ 2 mm (R4); it sets the holding heat. Use the R8 datum. |
| **Pendulum period** | `tools/pitch_inertia_budget.py` | Inverting `f = sqrt(mgd/I)/2pi` turns the torque budget into a measurement. |
| **Wire loop: turns tolerated + spring torque per turn** | ADR 0009 | Sets the unwind policy's limits and how much of the torque budget the loop takes. |

---

## Mechanical

- [ ] DM3505 (no encoder case): OD, length, mass, through-bore, rotor and stator bolt circles
      -> `PITCH_MOTOR_*`.
- [ ] N20 gearmotors (12 V, with encoders): body diameter, **total length including
      gearbox**, shaft diameter and length, mass -> `WHEEL_MOTOR_OD`, `WHEEL_MOTOR_LENGTH`,
      `SPINE_BORE`. Length varies with gear ratio and sets every axial station.
- [ ] Bearings: confirm 6709ZZ (45 × 55 × 6) at end A and 6704ZZ (20 × 27 × 4) at end B
      -> `BEARING_A_*`, `BEARING_*`.
- [ ] Spine waist rod (M5 threaded rod or 5 mm tube): OD -> `SPINE_WAIST_OD`. It sets the IMU's
      radial offset directly.
- [ ] Wheel A shaft extension and coupler: diameter, length (~48 mm needed), coupler OD
      (must pass the cup floor's 10 mm bore -> `CUP_FLOOR_BORE`).
- [ ] O-rings: cross-section and inside diameter against the wheel groove.

## Electronics

- [ ] Pi Zero 2 W: confirm 65 × 30, **height with connectors fitted**.
- [ ] Camera Module 3 Wide: board outline, **lens barrel height**, mounting hole spacing
      -> `CAMERA_MOUNT_HOLE_SPACING_*`, `CAMERA_APERTURE_*`. Also its true horizontal FOV.
- [ ] Small G474 board (not yet chosen): outline and height -> `bought_parts.mcu_board`. The
      Nucleo-G474RE is bench-only; it is wider than the bore.
- [ ] ICM-42688-P breakout: outline, hole spacing, and **the tallest part on the chip side**
      -> `IMU_BOARD_*`, `IMU_CHIP_SIDE_HEIGHT`, `IMU_MOUNT_HOLE_INSET`. The chip-side height
      sets the IMU offset.
- [ ] Encoder board and ring magnet: outline, magnet OD/ID, working distance.
- [ ] SimpleFOCMini, TB6612FNG, INA226, 5 V buck: outlines.
- [ ] Battery (3S 300 mAh): dimensions and mass. Listed as 60.5 × 16 × 11.5, 24.8 g.

## Masses

Weigh everything. The rotating budget assumes 267 g (140 g plastic, 127 g motor stator,
bearing races and electronics), and most of the component masses are guesses.

---

## After measuring

1. Update `cad/parameters.py` and `cad/bought_parts.py`, changing each tag to `MEASURED`.
2. Re-copy the CAD's rotating mass and inertia into `tools/pitch_inertia_budget.py`. A test
   fails if they drift apart.
3. Run `python3 cad/assembly.py` and `pytest tests/python/test_cad.py`.
4. Place the electronics envelopes in `cad/assembly.py`. They are not placed yet, so nothing
   checks that they fit **together** around the spine, the IMU and the wire loop.
