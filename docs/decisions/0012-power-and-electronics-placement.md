# 0012 — Power: a 3S battery, and every electronic part in the casing

- **Status:** **PROPOSED** (2026-09-30)
- **Amends:** [0005](0005-imu-placement.md) (the battery trade-off it left open) and
  [0008](0008-reduced-scale-direct-drive.md) (its 45 × 25 × 12 battery guess)
- **Related:** [0007](0007-foc-implementation.md), [0009](0009-wire-crossing.md),
  [0010](0010-direct-drive-axial-layout.md), [0011](0011-pitch-motor-and-encoder.md)

## Context

No ADR had fixed the supply voltage. Writing out the parts list showed that the parts
already chosen constrain it tightly, and that ADR 0008's battery guess didn't satisfy them.

## Decision 1: 3S LiPo

| Part | Supply range | Source |
|---|---|---|
| SimpleFOCMini v1 (DRV8313) | **8 V minimum**, 30 V maximum (board) | ADR 0007 |
| TB6612FNG | **13.5 V maximum** recommended (15 V absolute) | datasheet; confirm |
| DM3505 | 12 V nominal | ADR 0011 |

| Pack | Range | |
|---|---|---|
| 2S | 6.0–8.4 V | falls below the DRV8313's 8 V as it drains |
| **3S** | **9.0–12.6 V** | inside every range |
| 4S | 12.0–16.8 V | above the TB6612's limit when charged |

**3S is the only option that fits every part.** The N20 wheel motors therefore have to be the
**12 V variant**: 6 V N20s are common and would be over-driven.

## Decision 2: about 300 mAh, slim format

The proposed pack is a BetaFPV 3S 300 mAh 45C: **60.5 × 16 × 11.5 mm, 24.8 g**. Its shape
matters more than its capacity. Laid along the axis as a chord in the 65 mm bore, it fits
beside the spine waist. The squarer packs common at this capacity (e.g. 42 × 19 × 18) are
harder to fit. ADR 0008's 45 × 25 × 12 was closer to a 1S size.

Rough run time. Every load except the pitch motor is an estimate to measure:

| Load | Estimate |
|---|---|
| Pi Zero 2 W + camera streaming | 1.5–2.5 W |
| MCU, IMU, encoder, INA226 | ~0.3 W |
| Pitch motor, trimmed (ADR 0011) | 0.1–0.5 W |
| Wheel motors, driving | ~1 W average |
| Regulator losses | ~10% |
| **Total** | **3–4.5 W** |

300 mAh × 11.1 V = 3.3 Wh, about 2.7 Wh usable: **roughly 35–55 minutes**. Enough for testing
and for the research measurements, which are runs of minutes.

## Decision 3: the battery and every board ride in the casing

- **The spine has no room.** It is a 16 mm tube (ADR 0010), fully occupied by the wheel
  motors.
- **Nothing high-current crosses the joint.** With the battery and the TB6612 in the casing,
  the wire loop (ADR 0009) carries only the wheel motor leads and their encoder lines: about
  **10 conductors** (two per motor, two encoder signals each, shared power and ground). All of
  them are low-current or low-rate.
- **ADR 0005's battery trade-off is settled:** the rotating mass grows by about 25 g at
  r ≈ 18 mm, which the budget in ADR 0011 already includes.

The chassis is then just the spine, two wheel motors and their wires.

## Consequences

- A **5 V buck regulator (~2 A)** supplies the Pi. The MCU runs from 5 V through its own
  regulator. Per ADR 0007, the DRV8313 module's onboard 3.3 V supply (10 mA) powers nothing
  else.
- The **INA226** sits on the battery line and measures total power, which the run-time
  estimate above needs.
- The **kill switch** (CLAUDE.md) has to be reachable on a rotating casing: a switch on the
  shell, plus the operator's software e-stop. Exact location open.
- Charging means removing the pack or adding a balance-lead port through the shell. Open.
- **Buy 12 V N20s** with magnetic encoders.

## What remains open after acceptance

1. The TB6612FNG's maximum VM from the datasheet, before the first 12.6 V power-up.
2. The pack's real dimensions on arrival, and that it fits beside the waist alongside the
   camera and the IMU. The electronics aren't placed in `cad/assembly.py` yet.
3. The actual power draw, measured with the INA226.
