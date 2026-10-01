# 0011 — Pitch motor and encoder: GM2804H, one off-axis encoder, and a balance requirement

- **Status:** **PROPOSED** (2026-09-30)
- **Amends:** [0008](0008-reduced-scale-direct-drive.md) (which assumed a 2208 motor)
- **Related:** [0007](0007-foc-implementation.md) (FOC), [0010](0010-direct-drive-axial-layout.md)
  (the motor's position), [0012](0012-power-and-electronics-placement.md) (supply voltage)
- **Budget:** `tools/pitch_inertia_budget.py`, written up in
  [`docs/theory/pitch-axis-inertia-and-torque.md`](../theory/pitch-axis-inertia-and-torque.md)

## Context

ADR 0008 chose direct drive using a 2208-class motor (28 mm, solid shaft). ADR 0010 then found
that wheel A's shaft has to pass **through** the pitch motor, so the motor must be hollow. A
28 mm hollow-shaft gimbal motor with a usable bore could not be found. ADR 0008's torque figure
also turned out to leave out the electronics, and its torque-per-amp came from a different KV
than `cad/parameters.py`.

## Decision 1: iPower GM2804H-100T

From the iFlight listing (2026-09-30):

| | |
|---|---|
| Outer diameter | 35 mm |
| Height | 25 mm |
| Mass | 51 g |
| Hollow shaft | 8 mm OD, **6.5 mm ID** |
| Configuration | 12N14P (7 pole pairs) |
| Resistance | 9 Ω |
| Rated load | 0.35 kg·cm (34.3 mN m) at 0.8 A |
| Voltage | 2–3S |
| Price | ~$28 bare, ~$39 with encoder |

**Buy the GM2804H, not the GBM2804H**: the GBM variant's bore is listed as 5 mm.

The 6.5 mm bore passes wheel A's 3 mm shaft with 1.75 mm of radial clearance. The 35 mm body
needs a 37 mm cup, so the end-A bearing grows to a **6708ZZ (40 × 50 × 6)**. The CAD is updated
and every clearance and bearing check passes.

## Decision 2: one encoder, mounted off-axis

**With direct drive, the rotor angle *is* the casing-to-chassis angle.** One absolute encoder
therefore does both jobs: commutation for FOC (ADR 0007) and the relative angle `phi` that ADR
0005 derives chassis pitch from. The two-encoder plan, and the AS5600 fixed-address clash that
came with it, both disappear.

**But it cannot sit on the axis.** A standard magnetic encoder (AS5048A, AS5600) reads a small
magnet on the end of a shaft, and here the axis is taken by wheel A's shaft at every station.
Proposed: an **MPS MagAlpha MA732-class sensor in side-shaft position** beside a diametrically
magnetized **ring magnet** on the rotor. That is the arrangement the SimpleFOC community
recommends for hollow-shaft motors.

| Option | Verdict |
|---|---|
| MA732 side-shaft + ring magnet (proposed) | Works with the shaft through the bore. Side-shaft accuracy is lower and needs calibration |
| GM2804 encoder variant (bundled AS5048A) | **Only if its bore stays open through the encoder**, which no listing states. An on-axis magnet would block it. Check before buying |
| Second encoder at end B | Same problem: wheel B's shaft is on the axis there |

## Decision 3: horizontal CoM offset trimmed to 2 mm or less, as a requirement

Budget at the design point: a 10° correction in 100 ms, 3 m/s² chassis acceleration, 240 g
rotating, I = 0.171e-3 kg m². That is the plastic from the CAD plus about 109 g of motor stator and electronics
as point masses.

| Case | Torque | % of rated | Current | Heat while holding |
|---|---|---|---|---|
| Untrimmed, 10 mm sideways (ADR 0008's assumption) | 36.4 mN m | **106%** | 0.85 A | 2.2 W |
| Trimmed to 2 mm, balanced | 16.8 mN m | 49% | 0.39 A | 0.09 W |
| Trimmed to 2 mm, bottom-heavy 10 mm | 24.3 mN m | 71% | 0.57 A | 0.09 W |

**Holding gravity is the largest term**, larger than the correction itself. An untrimmed
casing would run the motor past its rating and put about 2.2 W of heat into a closed plastic
shell for as long as the camera holds level. Trimming the *sideways* offset to 2 mm with the R4
masses removes almost all of it.

The choice between balanced and bottom-heavy is a *separate* question about the vertical
offset. It stays open: it is owner-reviewed control design, and both are within rating.

## Consequences

- **ADR 0008's 22.5 mN m is replaced** by the budget above. Its Kt (from 90 KV) is replaced by
  the motor's rated point, 43 mN m/A, which is the more pessimistic figure.
- **Trim masses become a requirement, not an option.** The pendulum-period measurement R8 was
  designed for now also has to find the sideways offset.
- **Thermal is not "evaporated"** as ADR 0008 said. Gimbal motors are high-resistance, so I²R
  at modest current matters. It is acceptable only once trimmed.
- **The stator bolts to the casing and the rotor to the chassis**, so the windings and their
  three phase leads stay in the casing with the power stage (R3) and never cross the wire
  loop. The stator (~30 g at r ≈ 10 mm) is in the rotating budget. The encoder's sensor sits
  on the casing side, reading a ring magnet on the chassis-side rotor.
- `firmware/hal/` gets **one** pitch encoder interface, used by both FOC and estimation.

## What remains open after acceptance

1. The encoder geometry: an MA732 board (or equivalent) and a diametric ring magnet that fits
   the rotor or the hollow shaft, and the side-shaft accuracy after calibration.
2. The GM2804H's bolt circles. `cad/parameters.py` guesses both.
3. The rated point (0.35 kg·cm at 0.8 A) on a real unit. The budget assumes it.
