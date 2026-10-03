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

**Buy the Nucleo first, on its own.** The first firmware image (`firmware/stm32/`, 2026-10-01)
needs nothing else. It runs the safety machine, the UART link and the watchdog on a bare board,
and every unverified register value in it is settled by `docs/bringup/stm32-first-image.md`.
The rest of wave 1 is needed when the motor drivers are written.

| Item | Qty | Status | Notes |
|---|---|---|---|
| Nucleo-G474RE + USB cable | 1 | decided | Development board, never goes in the robot. Its built-in ST-LINK can likely also flash the small robot board later (to confirm) |
| ICM-42688-P breakout, SPI | 1 | decided | Pick one with **nothing taller than ~1 mm on the chip side** and room for headers on the back (ADR 0010) |
| INA226 breakout | 1 | decided | |
| TB6612FNG breakout | 1 | decided | |
| N20 gearmotor, **12 V**, magnetic encoder | 2 | verified | 12 V because the supply is 3S (ADR 0012). **100:1** (owner, 2026-10-01): Pololu #5216, see the DigiKey order |
| Breadboard + jumper wires | 1 set | — | |
| 10 kΩ resistors (pull-downs) | ~10 | decided | TB6612 STBY and PWM inputs, so a chip in reset drives nothing (ADR 0014; owner, ADR 0015 Q5: fitted before the first motor is powered). Value is an initial guess; check the breakout doesn't already fit them |

## DigiKey order — checked against listings 2026-10-01

Prices and stock as shown on digikey.com that day; they move. "Marketplace" items ship from
Pololu, possibly with their own shipping fee.

**Bench kit: ORDERED by the owner 2026-10-01.**

| Item | DigiKey # | Qty | Each | Notes |
|---|---|---|---|---|
| NUCLEO-G474RE | 497-19491-ND | 1 | $20.13 | 2,570 in stock. The "99 weeks" shown is ST's factory lead time, irrelevant while stocked |
| SparkFun TB6612FNG breakout with headers (14450) | 1568-14450-ND | 1 | $14.94 | VM max 15 V per SparkFun, so 3S (12.6 V full) fits: closes the "TB6612 max VM" item |
| Pololu #5216: 100:1 HPCB 12 V micro metal gearmotor, 12 CPR encoder, back connector | 2183-5216-ND | 2 | $32.45 | **Gear ratio 100:1: owner confirmed 2026-10-01.** 330 rpm, 0.75 A stall at 12 V (under the TB6612's 1.2 A); 12 × 100.37 = 1204 counts/rev. Closest to the sim plant (300 rpm, 1400 counts/rev), so ADR 0013's gains carry over best. Marketplace |
| Pololu #4763: JST SH 6-pin cable, 30 cm, one end bare | 2183-4763-ND | 2 | $3.00 | Encoder lead to breadboard. Marketplace |
| Stackpole CF14JT10K0: 10 kΩ 1/4 W | CF14JT10K0CT-ND | 10 | ~$0.02 | Pull-downs (ADR 0014/0015 Q5) |
| Carling 111-16-73 toggle, SPST 6 A 125 V AC/DC | 432-1086-ND | 1 | $12.69 | Kill switch inline with the motor supply; CLAUDE.md requires one before motors run |

Breadboard, Dupont jumpers and spare resistors: **from the makerspace** (owner, 2026-10-02).

**Pull-downs, settled for the pitch stage (2026-10-02):** the DRV8313's EN1-3 inputs have an
internal pulldown (TI datasheet SLVSBA5D, pin functions: "Channel enable. Logic high enables
the 1/2-H bridge channel; internal pulldown"), so a floating MCU pin after an IWDG reset leaves
the SimpleFOCMini's outputs off. Its R1-R3 are pull-ups on nFAULT/nSLEEP/nRESET to the chip's
own 3.3 V (SimpleFOCMini v1.1 schematic). The 10 kΩ parts above are for the **TB6612** only.
Don't load the SimpleFOCMini's 3.3 V pin: it is the DRV8313's 10 mA internal regulator.

**Wave 2 items DigiKey has (decided, can wait):**

| Item | DigiKey # | Each | Notes |
|---|---|---|---|
| SparkFun ROB-27477 DM3505 | 1568-ROB-27477-ND | $41.88 | 17 in stock. **ADR 0011 is still proposed**: accept it before buying |
| Pololu #2858 D24V22F5, 5 V 2.2 A buck | 2183-2858-ND | $18.95 | Pi power from 3S. Marketplace |
| Raspberry Pi Camera Module 3 Wide (SC1224) | 2648-SC1224-ND | $35.00 | 7,410 in stock |
| Adafruit 5211 Pi Zero camera cable, 30 cm | 1528-5211-ND | $3.95 | From a search result, not the product page; length may be long for the casing |

**Not from DigiKey:** Pi Zero 2 W (out of stock there), ICM-42688-P (only a $55.51 Pmod board),
INA226 (only TI's $58.80 EVM), SimpleFOCMini, 3S LiPo, microSD. Cheaper generic breakouts
exist elsewhere, and none of these are needed until their designs exist.

## Wave 2 — after the open items close

| Item | Qty | Status | Notes |
|---|---|---|---|
| **Mercury DM3505 gimbal motor, without encoder** — SparkFun ROB-27477 | 1 | verified | 40 × 16 mm, 8.5 mm bore, 0.09 N m nominal, 12 V, $34.95 SparkFun / $41.88 DigiKey. ADR 0011. Confirm the bore runs through |
| Pitch encoder: MA732-class board + diametric ring magnet | 1 | **open** | Has to read off-axis (ADR 0011). Sizes to match on purchase |
| SimpleFOCMini v1 (DRV8313) | 1 | decided | ADR 0007. v2.3 adds current sensing, worth considering for research reasons |
| 3S LiPo, 300 mAh, slim | 1 + spare | verified | BetaFPV 45C: 60.5 × 16 × 11.5 mm, 24.8 g. ADR 0012 |
| LiPo charger (3S balance) | 1 | — | Check whether the makerspace has one |
| 5 V buck regulator, ~2 A | 1 | decided | Powers the Pi from 3S |
| Raspberry Pi Zero 2 W | 1 | **bought** | Owner, 2026-10-02: Amazon kit (board + loose header, heatsink, OTG cable, mini-HDMI adapter), $99.99; approved resellers and DigiKey were out of stock. No microSD in the kit. Leave the header unsoldered: it adds ~8 mm the casing doesn't have |
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
| M3 screws for the end caps | 12 | decided | |
| M2.5 screws (motor, 8) and M2 screws (camera 4, IMU 2) | 14 | decided | Motor patterns from the DM3505 drawing |
| M3 heat-set inserts | ~20 | decided | Not yet in the CAD, which taps plastic directly |
| Trim masses: M3 screws, nuts, brass | assorted | decided | Recommended: trimming the sideways CoM to ≤ 2 mm cuts holding heat (ADR 0011) |
| Silicone-insulated stranded wire, 28–30 AWG | 1 spool | decided | For the wire loop (ADR 0009) |
| XT30 or similar battery connector, JST-PH leads | a few | — | |
| Power / kill switch | 1 | decided | Required by CLAUDE.md before motors run |
| Filament, ABS or PETG | 1 roll | — | The mass report assumes ABS. Check whether the makerspace supplies it |
| Filament, TPU 95A | ~50 g | decided | Two tires (ADR 0017), ~21 g each. Check whether the makerspace has it and a printer that can feed it |

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
