# 0011 — Pitch motor and encoder: DM3505, one off-axis encoder

- **Status:** **PROPOSED** (2026-09-30). The owner chose the motor in conversation; the CAD
  and budget implement it.
- **Amends:** [0008](0008-reduced-scale-direct-drive.md) (which assumed a 2208 motor)
- **Related:** [0007](0007-foc-implementation.md) (FOC), [0010](0010-direct-drive-axial-layout.md)
  (the motor's position), [0012](0012-power-and-electronics-placement.md) (supply voltage)
- **Budget:** `tools/pitch_inertia_budget.py`, written up in
  [`docs/theory/pitch-axis-inertia-and-torque.md`](../theory/pitch-axis-inertia-and-torque.md)

## Context

ADR 0008 chose direct drive using a 2208-class motor (28 mm, solid shaft). ADR 0010 then found
that wheel A's shaft has to pass **through** the pitch motor, so the motor must be hollow. ADR
0008's torque figure also left out the electronics.

## Decision 1: Mercury Motor DM3505, sold by SparkFun as ROB-27477

From the manufacturer datasheet (SparkFun CDN, `27477_27478_Datasheet.pdf`):

| | |
|---|---|
| Outer diameter × height | 40 × 16 mm (without encoder case); **40 × 20 mm with encoder and case** (manufacturer website, found by the owner 2026-10-02) |
| Mass | 58.3 g |
| Hollow bore | 8.5 mm on the rotor face |
| Pole pairs | 11 |
| Nominal | 12 V, 1.1 A, **0.09 N m** |
| Stall | 0.15 N m at 1.9 A |
| Torque constant | 0.08 N m/A (stated) |
| Resistance | 6.34 Ω phase to phase |
| Mounting | rotor: 8 × M2.5 on ⌀12 / ⌀15; base: 4 × M2 + 4 × M2.5 on ⌀33.5 |
| Price | $34.95 at SparkFun, in stock |

**Buy the version without the encoder case.** Its bundled AS5048A/AS5600 reads on-axis, where
wheel A's shaft runs. The encoder version is also 4 mm longer (40 × 20 mm), length the axial
stack doesn't have to give, and its case covers the back face, which may close the bore.

### Candidates considered

| Motor | Rated torque | Bore | Why not |
|---|---|---|---|
| **DM3505 (chosen)** | **90 mN m** | 8.5 mm | — |
| iPower GM2804H | 34 mN m | 6.5 mm | At its limit: 106% of rated untrimmed. Overseas sourcing |
| Oncetop EM3215 (SparkFun ROB-20441) | 31 mN m *at stall* | 5.9 mm? | 6–8 V only, below the 3S supply; too little torque; backordered |
| 2208 class | ~30 mN m | none | Solid shaft: the layout is impossible |

The 40 mm body needs a 42 mm cup, so the end-A bearing is a **6709ZZ (45 × 55 × 6)**. The CAD is
updated and every clearance and bearing check passes.

## Decision 2: one encoder, mounted off-axis

**With direct drive, the rotor angle *is* the casing-to-chassis angle.** One absolute encoder
therefore does both jobs: commutation for FOC (ADR 0007) and the relative angle `phi` that ADR
0005 derives chassis pitch from. The two-encoder plan, and the AS5600 fixed-address clash that
came with it, both disappear.

**But it cannot sit on the axis.** A standard magnetic encoder reads a small magnet on the end
of a shaft, and here the axis is taken by wheel A's shaft at every station. Proposed: an **MPS
MagAlpha MA732-class sensor in side-shaft position** beside a diametrically magnetized **ring
magnet** on the rotor. That is the arrangement the SimpleFOC community recommends for
hollow-shaft motors.

The sensor sits on the casing side, with the MCU; the magnet is on the rotor, which is on the
chassis side.

## Decision 3: orientation — stator on the casing

The stator (windings) bolts to end cap A and the rotor to the cup floor. That keeps the three
phase leads inside the casing with the power stage (R3), so they never cross the wire loop.

## The budget, and what it says about balance

At the design point: a 10° correction in 100 ms, 3 m/s² chassis acceleration. The rotating
assembly is 267 g at I = 0.180e-3 kg m²: plastic from the CAD, plus 127 g of motor stator,
bearing races and electronics as point masses.

| Case | Torque | % of rated | Current | Heat while holding |
|---|---|---|---|---|
| Untrimmed, 10 mm sideways | 38.8 mN m | 43% | 0.48 A | 0.51 W |
| Untrimmed, plus bottom-heavy 10 mm | 46.8 mN m | 52% | 0.59 A | 0.51 W |
| Trimmed to 2 mm, balanced | 17.8 mN m | 20% | 0.22 A | 0.02 W |
| Trimmed to 2 mm, bottom-heavy 10 mm | 25.8 mN m | 29% | 0.32 A | 0.02 W |

**Holding gravity is still the largest single term**, but this motor has room for it. Even the
worst case stays near half of rated. **Trimming the sideways offset is therefore recommended,
not required:** it cuts holding heat from 0.5 W to almost nothing and leaves margin for the
wire loop's spring torque and bearing drag, but nothing breaks without it. With the GM2804H
the untrimmed case ran at 106% of rated, which is the main reason this motor replaced it.

**Follow-up, 2026-10-03: printed in PLA, not ABS.** The makerspace supplies only PLA, so the
plastic is 19% heavier than the figures above assumed: rotating assembly 294 g at
I = 0.206e-3 kg m². The cases become 43.3 / 52.1 / 20.2 / 29.0 mN m (48 / 58 / 22 / 32% of
rated). Every conclusion above still holds; current numbers live in
`docs/theory/pitch-axis-inertia-and-torque.md` and `tools/pitch_inertia_budget.py`.

## Consequences

- **ADR 0008's 22.5 mN m is replaced** by the budget above. Its Kt (0.106, from 90 KV) is
  replaced by the DM3505's stated 0.08 N m/A.
- **Thermal is not "evaporated"** as ADR 0008 said, but it is modest: at most ~0.5 W held, and
  far less once trimmed.
- 11 pole pairs: the electrical frequency at a 3.5 rad/s slew is 6.1 Hz, trivial for FOC.
- `firmware/hal/` gets **one** pitch encoder interface, used by both FOC and estimation.
- The heavier motor and its larger bearing add ~17 g to the rotating budget; included above.

## What remains open after acceptance

1. **That the 8.5 mm bore runs through the base as well as the rotor face.** The product photo
   shows a hollow shaft, but the drawing doesn't state it. Check the part on arrival.
2. The encoder hardware: an MA732 board (or equivalent) and a diametric ring magnet that fit
   beside the motor, and the side-shaft accuracy after calibration.
3. Which of the base's two bolt patterns (M2 or M2.5 on ⌀33.5) to use. The CAD uses M2.5.
