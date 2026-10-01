"""Envelopes for every bought component, so the interference sweep can see them.

These are NOT printed parts. Each is a plain box or cylinder standing in for something
ordered, sized from a datasheet or a vendor listing. The point is that the assembly check
covers the real contents of the casing, not just the plastic.

That matters more than it sounds. Until the pitch motor got an envelope it was invisible
to every check, and the bracket had been positioned inside where its body sits.

EVERY dimension here carries a tag:

  MEASURED   taken off the real part with calipers -- trustworthy
  LISTING    from a vendor listing or datasheet -- probably right, confirm on arrival
  GUESS      invented, because the sweep needs a number -- replace this one first

The workflow is: parts arrive, measure, change the number, re-run
`python3 cad/assembly.py`. Nothing else needs touching. docs/bringup/component-measurements.md
is the checklist.
"""

from __future__ import annotations

from dataclasses import dataclass

import cadquery as cq

import parameters as P


@dataclass(frozen=True)
class Component:
    """A bought part reduced to a bounding envelope.

    Boxes are given as (x, y, z); cylinders as (diameter, length) with the axis along z
    before placement. `tag` records how much the numbers can be trusted.
    """

    name: str
    shape: str          # "box" or "cylinder"
    dims: tuple
    mass_g: float
    tag: str
    note: str = ""

    def solid(self) -> cq.Workplane:
        if self.shape == "box":
            x, y, z = self.dims
            return cq.Workplane("XY").box(x, y, z, centered=(True, True, False))
        diameter, length = self.dims
        return cq.Workplane("XY").circle(diameter / 2).extrude(length)


# --------------------------------------------------------------- inside the casing

CASING_COMPONENTS: list[Component] = [
    Component("pitch_motor", "cylinder", (P.PITCH_MOTOR_OD, P.PITCH_MOTOR_LENGTH),
              P.PITCH_MOTOR_MASS_G, "LISTING",
              "iPower GM2804H-100T, hollow shaft 6.5 mm ID, 12N14P, 9 ohm (ADR 0011). Not "
              "the GBM2804H variant, whose bore is 5 mm."),
    Component("pi_zero_2w", "box", (65.0, 30.0, 5.0), 11.0, "LISTING",
              "Pi Zero 2 W board outline is well documented; 5 mm is with nothing stacked."),
    Component("camera_module_3_wide", "box", (25.0, 24.0, 12.5), 5.0, "LISTING",
              "Lens barrel height dominates and varies; measure it."),
    Component("mcu_board", "box", (50.0, 25.0, 8.0), 8.0, "GUESS",
              "Small STM32G474 board, NOT YET CHOSEN. The Nucleo-G474RE (70 x 82) is "
              "bench-only: it is wider than the 65 mm bore. Same chip, same firmware."),
    Component("imu_breakout", "box", (P.IMU_BOARD_LENGTH, P.IMU_BOARD_WIDTH,
                                      P.IMU_BOARD_THICKNESS + P.IMU_CHIP_SIDE_HEIGHT),
              2.0, "GUESS",
              "ICM-42688-P breakout, SPI. Needs NOTHING taller than the chip on the chip side "
              "and headers on the back: it lies chip-down beside the spine waist (ADR 0010)."),
    Component("pitch_encoder", "box", (12.0, 12.0, 3.0), 3.0, "GUESS",
              "ONE encoder: with direct drive the rotor angle is the casing angle, so it does "
              "both commutation and casing pitch (ADR 0011). Must work off-axis -- wheel A's "
              "shaft occupies the axis. MA732-class sensor beside a diametric ring magnet."),
    Component("foc_driver", "box", (25.0, 20.0, 8.0), 5.0, "GUESS",
              "SimpleFOCMini (DRV8313), ADR 0007. Needs >= 8 V, hence 3S (ADR 0012)."),
    Component("buck_5v", "box", (22.0, 17.0, 4.0), 3.0, "GUESS",
              "5 V regulator for the Pi from the 3S pack, ~2 A (ADR 0012)."),
    Component("power_monitor", "box", (20.0, 16.0, 3.0), 2.0, "GUESS", "INA226 breakout."),
    Component("battery", "box", (60.5, 16.0, 11.5), 24.8, "LISTING",
              "3S 300 mAh, BetaFPV 45C listing: 60.5 x 16 x 11.5 mm, 24.8 g. Slim enough to "
              "lie along the axis in the casing (ADR 0012)."),
    Component("motor_driver", "box", (20.0, 20.0, 3.0), 3.0, "GUESS",
              "TB6612FNG breakout. In the casing: the spine has no platform (ADR 0012)."),
]

# ------------------------------------------------------------- inside the chassis
#
# Only the wheel motors (ADR 0012). Everything electronic rides in the casing, so the
# wire loop (ADR 0009) carries just the wheel motor leads and their encoder lines.

CHASSIS_COMPONENTS: list[Component] = [
    Component("wheel_motor_left", "cylinder", (P.WHEEL_MOTOR_OD, P.WHEEL_MOTOR_LENGTH), 10.0,
              "GUESS", "N20 gearmotor with magnetic encoder, inside the spine. Length "
                       "depends on gear ratio -- measure, it sets every axial station."),
    Component("wheel_motor_right", "cylinder", (P.WHEEL_MOTOR_OD, P.WHEEL_MOTOR_LENGTH), 10.0,
              "GUESS", "as above"),
]

ALL_COMPONENTS = CASING_COMPONENTS + CHASSIS_COMPONENTS


def total_mass_g() -> float:
    return sum(c.mass_g for c in ALL_COMPONENTS)


def by_tag() -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for c in ALL_COMPONENTS:
        out.setdefault(c.tag, []).append(c.name)
    return out
