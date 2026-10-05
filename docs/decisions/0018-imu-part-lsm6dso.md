# 0018 — IMU part: LSM6DSO on a Pololu carrier, replacing the ICM-42688-P

- **Status:** **ACCEPTED** by the owner, 2026-10-04 (chosen from three options in chat).
- **Amends:** [0005](0005-imu-placement.md) (which named the ICM-42688-P). Placement is
  unchanged: on the casing, chip side down beside the spine's waist ([0010](0010-direct-drive-axial-layout.md)).

## Context

The ICM-42688-P was named early and never designed around: no driver exists, and
`firmware/hal/hal/imu.hpp` is chip-independent. When it came time to buy (2026-10-04) it was in
a supply shortage: DigiKey showed the bare part out of stock until August 2027, and the
breakouts found were either sold out (Amazon Sysrox, Mikroe 6DOF IMU 14 Click, Elecrow,
SparkFun) or $47–98 (Mouser's TDK QCIOT eval board, a Tindie listing). A bare LGA part is not
an option anyway: mounting it needs a custom PCB, which is out of scope.

What the design actually needs from the part: SPI, at least 1 kHz output (CLAUDE.md loop
rates), a ±2000 °/s gyro range, and a breakout small enough to lie beside the waist with
nothing tall on the chip side.

## Options considered

| Option | Price | For | Against |
|---|---|---|---|
| **Pololu #2798, LSM6DSO carrier** | $21.95 | SPI, ±125 to ±2000 °/s, ODR well above 1 kHz; 12.7 × 22.9 mm; in stock in the US | voltage regulator on board, height unknown |
| Adafruit #4438, LSM6DSOX | $11.95 | cheapest | 25.6 × 17.8 mm; STEMMA connectors on the chip side make it 4.6 mm thick, breaking ADR 0010's chip-side rule unless removed |
| Keep the ICM-42688-P | $47+ | the part the early docs name | price and stock, for no capability the design uses |

## Decision

**Pololu #2798 (ST LSM6DSO).**

## Datasheet check (ST LSM6DSO, DocID rev. via SparkFun CDN, read 2026-10-04)

| | LSM6DSO | Needed |
|---|---|---|
| Gyro full scale | ±125 to ±2000 dps | ±2000 dps |
| Output data rate | 12.5 Hz to 6.66 kHz | ≥ 1 kHz |
| SPI | 3- and 4-wire, 10 MHz | SPI |
| Supply | 1.71–3.6 V | 3.3 V |
| Package | 2.5 × 3.0 × 0.83 mm LGA | — |
| Gyro rate noise density | 3.8 mdps/√Hz (high-performance) | — |
| Gyro zero-rate level / drift | ±1 dps / ±0.010 dps/°C | the estimator tracks bias |
| Mechanical shock | 20,000 g for 0.2 ms | ADR 0005's open check |

## Consequences

- **Offset from the axis.** The board rests on its tallest chip-side part, so
  `r = 3.25 + tallest - 0.415` mm (package 2.5 × 3 × 0.83 mm, ST datasheet). If the chip is
  tallest: 3.67 mm. If the regulator is up to 1.45 mm: about 4.3 mm. Both under R2's 5 mm. The
  CAD keeps `IMU_CHIP_SIDE_HEIGHT = 1.0` (3.8 mm) as a guess until the board is measured.
- **Mount it chip side down whatever the regulator height.** Flipping it chip side out puts the
  chip at about 5.3 mm, which is worse than any taller-regulator case.
- The tangential error at 200 rad/s² scales with r: 4.4° at 3.8 mm, about 4.9° at 4.3 mm.
  It is correctable from the gyro's derivative (ADR 0010).
- The IMU driver, when written, targets the LSM6DSO's register map. Nothing written so far
  changes.
- Shock survival (ADR 0005's open check) now applies to the LSM6DSO's rating.
