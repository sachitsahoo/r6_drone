# Bill of materials

Everything to buy, in three waves tied to when the work needs it. The owner starts with only an
Xbox controller and a laptop. Soldering iron, multimeter, bench power supply and 3D printing come
from the makerspace, so they are not listed.

**Status** says how far each item can be trusted:
**verified** = checked against a listing or datasheet, with the source below ·
**decided** = chosen in an ADR, not yet checked against a listing ·
**open** = a choice still to make.

Nothing in Phase 1 needs hardware. Wave 1 starts when firmware first touches real parts.

## Wave 1 — bench kit

| Item | Qty | Status | Notes |
|---|---|---|---|
| Nucleo-G474RE + USB cable | 1 | decided | Development board, never goes in the robot. Its built-in ST-LINK can likely also flash the small robot board later (to confirm) |
| ICM-42688-P breakout, SPI | 1 | decided | Pick one with **nothing taller than ~1 mm on the chip side** and room for headers on the back (ADR 0010) |
| INA226 breakout | 1 | decided | |
| TB6612FNG breakout | 1 | decided | |
| N20 gearmotor, **12 V**, magnetic encoder | 2 | decided | 12 V because the supply is 3S (ADR 0012). Gear ratio still to pick |
| Breadboard + jumper wires | 1 set | — | |

## Wave 2 — after the open items close

| Item | Qty | Status | Notes |
|---|---|---|---|
| **Mercury DM3505 gimbal motor, without encoder** — SparkFun ROB-27477 | 1 | verified | 40 × 16 mm, 8.5 mm bore, 0.09 N m nominal, 12 V, $34.95 SparkFun / $41.88 DigiKey. ADR 0011. Confirm the bore runs through |
| Pitch encoder: MA732-class board + diametric ring magnet | 1 | **open** | Has to read off-axis (ADR 0011). Sizes to match on purchase |
| SimpleFOCMini v1 (DRV8313) | 1 | decided | ADR 0007. v2.3 adds current sensing, worth considering for research reasons |
| 3S LiPo, 300 mAh, slim | 1 + spare | verified | BetaFPV 45C: 60.5 × 16 × 11.5 mm, 24.8 g. ADR 0012 |
| LiPo charger (3S balance) | 1 | — | Check whether the makerspace has one |
| 5 V buck regulator, ~2 A | 1 | decided | Powers the Pi from 3S |
| Raspberry Pi Zero 2 W | 1 | decided | |
| Camera Module 3 **Wide** | 1 | decided | |
| Pi Zero camera cable | 1 | decided | The Zero's connector is smaller than a full-size Pi's |
| microSD card | 1 | — | |

## Wave 3 — robot build

| Item | Qty | Status | Notes |
|---|---|---|---|
| Small STM32G474 board | 1 | **open** | Must fit a 65 mm bore; the Nucleo cannot |
| 6709ZZ bearing, 45 × 55 × 6 | 1 | verified | End A, on the cup wall |
| 6704ZZ bearing, 20 × 27 × 4 | 1 | decided | End B, on the spine boss |
| M5 threaded rod, or 5 mm brass/aluminium tube | ~100 mm | decided | Spine waist. Tube if wheel motor A's wires run through it |
| 3 mm rod + 3-to-3 mm coupler | ~50 mm | decided | Wheel A's shaft extension (~48 mm). Coupler must pass a 10 mm bore |
| O-rings, 3 mm cross-section, ~100 mm ID | 2 + spares | decided | Wheel treads; size from the groove |
| M3 screws for the end caps | 12 | decided | |
| M2.5 screws (motor, 8) and M2 screws (camera 4, IMU 2) | 14 | decided | Motor patterns from the DM3505 drawing |
| M3 heat-set inserts | ~20 | decided | Not yet in the CAD, which taps plastic directly |
| Trim masses: M3 screws, nuts, brass | assorted | decided | Recommended: trimming the sideways CoM to ≤ 2 mm cuts holding heat (ADR 0011) |
| Silicone-insulated stranded wire, 28–30 AWG | 1 spool | decided | For the wire loop (ADR 0009) |
| XT30 or similar battery connector, JST-PH leads | a few | — | |
| Power / kill switch | 1 | decided | Required by CLAUDE.md before motors run |
| Filament, ABS or PETG | 1 roll | — | The mass report assumes ABS. Check whether the makerspace supplies it |

## Borrow before buying

Calipers and a 0.1 g scale are needed for the measurement checklist
(`docs/bringup/component-measurements.md`). Buy only if the makerspace has neither.

## Not buying

- **A slip ring.** It has nowhere to fit, and its drag exceeds the budget (ADR 0009).
- **A second encoder.** One serves both FOC and estimation under direct drive (ADR 0011).
- **A USB-to-serial adapter.** The Nucleo's USB carries debug output.

## Sources for "verified"

- DM3505: [SparkFun ROB-27477](https://www.sparkfun.com/products/27477),
  [DigiKey](https://www.digikey.com/en/products/detail/sparkfun-electronics/ROB-27477/26523949),
  [datasheet](https://cdn.sparkfun.com/assets/3/3/d/4/5/27477_27478_Datasheet.pdf).
- Hollow-shaft encoder approach: [SimpleFOC community thread](https://community.simplefoc.com/t/recommended-absolute-encoder-mounting-for-hollow-shaft-motor/1677).
- 6709ZZ: [VXB](https://vxb.com/products/6709zz-thin-section-shielded-ball-bearing-45x55x6).
- 3S 300 mAh: [BetaFPV](https://betafpv.com/products/300mah-3s-45c-lipo-battery-s-version-2pcs).
- ICM-42688-P package, 2.5 × 3 × 0.91 mm LGA-14: [LCSC](https://support.lcsc.com/product-detail/Attitude-Sensor-Gyroscope_TDK-InvenSense-ICM-42688-P_C1850418.html).
