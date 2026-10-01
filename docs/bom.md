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
| iPower **GM2804H-100T** gimbal motor | 1 | verified | 35 mm, hollow 6.5 mm ID, 12N14P, 9 Ω, ~$28 bare. Not the GBM2804H (5 mm bore). ADR 0011 |
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
| 6708ZZ bearing, 40 × 50 × 6 | 1 | verified | End A, on the cup wall |
| 6704ZZ bearing, 20 × 27 × 4 | 1 | decided | End B, on the spine boss |
| M5 threaded rod, or 5 mm brass/aluminium tube | ~100 mm | decided | Spine waist. Tube if wheel motor A's wires run through it |
| 3 mm rod + 3-to-3 mm coupler | ~60 mm | decided | Wheel A's shaft extension (~57 mm) |
| O-rings, 3 mm cross-section, ~100 mm ID | 2 + spares | decided | Wheel treads; size from the groove |
| M3 screws for the end caps | 12 | decided | |
| M2 screws (motor 8, camera 4, IMU 2) | 14 | decided | Bolt circles still guesses |
| M3 heat-set inserts | ~20 | decided | Not yet in the CAD, which taps plastic directly |
| Trim masses: M3 screws, nuts, brass | assorted | decided | **Required**: the sideways CoM must be trimmed to ≤ 2 mm (ADR 0011) |
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

- GM2804H: [iFlight GM2804 with encoder](https://shop.iflight.com/ipower-gm2804-gimbal-motor-with-as5048a-encoder-pro288)
  (35 mm OD, 25 mm height, 51 g, 6.5 mm ID, 12N14P, 9 Ω, 0.35 kg·cm), and the
  [rcdrone gimbal motor list](https://rcdrone.top/collections/gimbal-motor) (bare-motor price).
- Hollow-shaft encoder approach: [SimpleFOC community thread](https://community.simplefoc.com/t/recommended-absolute-encoder-mounting-for-hollow-shaft-motor/1677).
- 6708ZZ: [Simply Bearings](https://www.simplybearings.co.uk/products/6708-zz-eu). Also confirms
  that 61807 is 35 × 47 × 7, not the 35 × 44 × 5 an earlier draft used.
- 3S 300 mAh: [BetaFPV](https://betafpv.com/products/300mah-3s-45c-lipo-battery-s-version-2pcs).
- ICM-42688-P package, 2.5 × 3 × 0.91 mm LGA-14: [LCSC](https://support.lcsc.com/product-detail/Attitude-Sensor-Gyroscope_TDK-InvenSense-ICM-42688-P_C1850418.html).
