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
    Component("pitch_motor", "cylinder", (P.PITCH_MOTOR_BORE, P.PITCH_MOTOR_LENGTH),
              39.0, "LISTING", "2208 gimbal, 28 x 26 mm, 39 g. Confirm OD on arrival: a "
                               "mislabelled 2804-class part is 35 mm and will not fit."),
    Component("pi_zero_2w", "box", (65.0, 30.0, 5.0), 11.0, "LISTING",
              "Pi Zero 2 W board outline is well documented; 5 mm is with nothing stacked."),
    Component("camera_module_3_wide", "box", (25.0, 24.0, 12.5), 5.0, "LISTING",
              "Lens barrel height dominates and varies; measure it."),
    Component("mcu_board", "box", (70.0, 82.0, 20.0), 60.0, "GUESS",
              "Nucleo-G474RE with headers. This is the biggest thing going in the casing "
              "and the most likely not to fit -- see the sweep."),
    Component("imu_breakout", "box", (20.0, 16.0, 3.0), 2.0, "GUESS",
              "ICM-42688-P breakout. Board outline varies by vendor."),
    Component("encoder_casing", "box", (20.0, 15.0, 3.0), 2.0, "GUESS",
              "AS5600 breakout reading the casing angle (ADR 0005)."),
    Component("encoder_motor", "box", (20.0, 15.0, 3.0), 2.0, "GUESS",
              "AS5600 breakout for FOC commutation. Note: AS5600's I2C address is fixed at "
              "0x36, so this and encoder_casing cannot share a bus."),
    Component("foc_driver", "box", (25.0, 20.0, 8.0), 5.0, "GUESS",
              "SimpleFOC Mini, or an eventual DRV8313-class stage."),
    Component("power_monitor", "box", (20.0, 16.0, 3.0), 2.0, "GUESS", "INA226 breakout."),
    Component("battery", "box", (60.0, 32.0, 18.0), 85.0, "GUESS",
              "3S LiPo, ~1000 mAh. Placement is UNDECIDED: in the casing it adds the "
              "heaviest single item to the rotating inertia; in the chassis it puts high "
              "current across the slip ring. See ADR 0005."),
]

# ------------------------------------------------------------- inside the chassis

CHASSIS_COMPONENTS: list[Component] = [
    Component("wheel_motor_left", "cylinder", (12.0, 40.0), 10.0, "GUESS",
              "N20 gearmotor. Length depends entirely on gear ratio -- measure."),
    Component("wheel_motor_right", "cylinder", (12.0, 40.0), 10.0, "GUESS", "as above"),
    Component("motor_driver", "box", (20.0, 20.0, 3.0), 3.0, "GUESS", "TB6612FNG breakout."),
    Component("slip_ring", "cylinder", (P.SLIP_RING_ENVELOPE_DIA,
                                        P.SLIP_RING_ENVELOPE_LENGTH),
              20.0, "GUESS", "Capsule slip ring, 8-12 circuits. Not yet selected."),
]

ALL_COMPONENTS = CASING_COMPONENTS + CHASSIS_COMPONENTS


def total_mass_g() -> float:
    return sum(c.mass_g for c in ALL_COMPONENTS)


def by_tag() -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for c in ALL_COMPONENTS:
        out.setdefault(c.tag, []).append(c.name)
    return out
