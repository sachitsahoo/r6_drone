#!/usr/bin/env python3
"""Inertia and torque budget for the pitch axis, at the current design point.

Feeds docs/theory/pitch-axis-inertia-and-torque.md and ADR 0011 (pitch motor, DM3505). Offline
analysis only -- nothing here runs on the robot.

The design point is ADR 0008's 70 x 182 mm casing, direct drive, with ADR 0010's spine
layout. The printed parts' mass and inertia come from the CAD solids (cad/build.py); the
electronics are point masses at the radii the layout puts them, which are estimates. Every
input is tagged with where it came from, and nothing here is a measurement yet.

This script deliberately does not import cad/: CadQuery is not a dependency of the tools
or of CI. The two CAD figures are copied in with their source, and the test suite checks
them against the CAD when CadQuery is installed.

Usage:
    python3 tools/pitch_inertia_budget.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.81  # m/s^2

# ------------------------------------------------------------------- printed plastic

#: From `python3 cad/build.py --report`, 2026-09-30: shell, both end caps, IMU bridge and
#: camera mount, in ABS. Re-copy these whenever the CAD changes.
CAD_ROTATING_PLASTIC_KG = 0.1402
CAD_ROTATING_PLASTIC_INERTIA_KG_M2 = 0.136e-3
#: The shell alone, same source. It is the reason mass at the wall is expensive.
CAD_SHELL_INERTIA_KG_M2 = 0.121e-3

# ------------------------------------------------------------------ the pitch motor
#
# Mercury DM3505, SparkFun ROB-27477 (ADR 0011). Datasheet 27477_27478, 2026-09-30.
#: "Nominal torque 0.09 N.m" at "Nominal current 1.1 A". Taken as the rated continuous point.
MOTOR_RATED_TORQUE_N_M = 0.09
MOTOR_RATED_CURRENT_A = 1.1
#: "Torque constant 0.08 Nm/A" -- stated directly, so no KV conversion is needed. The rated
#: point implies 0.082, which agrees.
MOTOR_KT_N_M_PER_A = 0.08
#: "Phase to phase resistance 6.34 ohm", so 3.17 ohm per phase in a wye.
MOTOR_RESISTANCE_PHASE_TO_PHASE_OHM = 6.34
#: Power stage limit, ADR 0007 (DRV8313 on SimpleFOCMini v1).
DRIVER_CURRENT_LIMIT_A = 2.5


@dataclass(frozen=True)
class PointMass:
    """A component treated as a point mass at a radius from the rotation axis."""

    name: str
    mass_kg: float
    radius_m: float
    source: str = ""

    @property
    def inertia_kg_m2(self) -> float:
        return self.mass_kg * self.radius_m ** 2


#: What the casing carries besides plastic. Radii are where ADR 0010's layout puts each
#: part: boards lie as chords in the 65 mm bore, so their centroids sit around r = 20 mm.
#: All radii are ESTIMATES until the electronics are placed in cad/assembly.py.
CASING_CONTENTS: list[PointMass] = [
    PointMass("pitch motor stator + windings", 0.033, 0.012,
              "~55% of the 58 g DM3505; GUESS. Stator on the casing, ADR 0011"),
    PointMass("bearing outer races (6709, 6704)", 0.015, 0.024,
              "about half of 25 g + 5 g; GUESS"),
    PointMass("battery, 3S 300 mAh", 0.025, 0.018, "BetaFPV listing 24.8 g; ADR 0012"),
    PointMass("Pi Zero 2 W", 0.011, 0.022, "listing"),
    PointMass("MCU board, small G474", 0.008, 0.022, "GUESS; board not chosen"),
    PointMass("camera module", 0.005, 0.020, "listing"),
    PointMass("power stage (SimpleFOCMini)", 0.005, 0.022, "GUESS"),
    PointMass("wheel driver (TB6612)", 0.003, 0.022, "GUESS; in the casing per ADR 0012"),
    PointMass("power monitor (INA226)", 0.002, 0.022, "GUESS"),
    PointMass("encoder + ring magnet", 0.003, 0.015, "GUESS; ADR 0011"),
    PointMass("IMU breakout", 0.002, 0.004, "beside the waist, ADR 0010"),
    PointMass("wiring, fasteners, 5 V regulator", 0.015, 0.020, "GUESS"),
]


@dataclass
class CasingModel:
    plastic_mass_kg: float
    plastic_inertia_kg_m2: float
    shell_inertia_kg_m2: float
    contents: list[PointMass] = field(default_factory=list)

    @property
    def contents_mass_kg(self) -> float:
        return sum(c.mass_kg for c in self.contents)

    @property
    def contents_inertia_kg_m2(self) -> float:
        return sum(c.inertia_kg_m2 for c in self.contents)

    @property
    def total_mass_kg(self) -> float:
        return self.plastic_mass_kg + self.contents_mass_kg

    @property
    def total_inertia_kg_m2(self) -> float:
        return self.plastic_inertia_kg_m2 + self.contents_inertia_kg_m2


def build_model() -> CasingModel:
    return CasingModel(plastic_mass_kg=CAD_ROTATING_PLASTIC_KG,
                       plastic_inertia_kg_m2=CAD_ROTATING_PLASTIC_INERTIA_KG_M2,
                       shell_inertia_kg_m2=CAD_SHELL_INERTIA_KG_M2,
                       contents=list(CASING_CONTENTS))


# --------------------------------------------------------------------------- loads

def gravity_torque_N_m(mass_kg: float, com_offset_m: float) -> float:
    """Torque from gravity on a centre of mass offset `com_offset_m` horizontally.

    With the camera held level the casing's world attitude is constant, so this is a
    *holding* load, paid continuously. Only the horizontal component of the offset (in the
    level pose) counts; a centre of mass directly below the axis costs nothing to hold.
    """
    return mass_kg * G * com_offset_m


def slew_torque_N_m(inertia_kg_m2: float, angle_rad: float, time_s: float) -> float:
    """Torque to correct `angle_rad` in `time_s` under constant-magnitude acceleration.

    Accelerate for half the interval, decelerate for the other half, so the peak angular
    acceleration is alpha = 4 * theta / t^2. A real controller does not run bang-bang, but
    this bounds what the actuator must be able to produce.
    """
    alpha = 4.0 * angle_rad / (time_s ** 2)
    return inertia_kg_m2 * alpha


def acceleration_torque_N_m(mass_kg: float, vertical_offset_m: float,
                            accel_m_s2: float) -> float:
    """Torque the stabilizer must reject when the chassis accelerates.

    A casing whose centre of mass sits `vertical_offset_m` below the axis is a pendulum; a
    horizontal acceleration of the axle swings it with torque m a d. This is the disturbance
    the research question is about (ADR 0004, coupling direction 2).
    """
    return mass_kg * accel_m_s2 * vertical_offset_m


def pendulum_frequency_Hz(mass_kg: float, com_offset_m: float,
                          inertia_kg_m2: float) -> float:
    """Natural frequency of the casing swinging as a pendulum about the wheel axis.

    omega = sqrt(m g d / I). The resonance sits inside the stabilization loop. A
    bottom-heavy casing passively keeps the camera roughly upright, but the restoring
    torque and the resonance have to be handled by the controller.
    """
    if com_offset_m <= 0.0:
        return 0.0
    return math.sqrt(mass_kg * G * com_offset_m / inertia_kg_m2) / (2.0 * math.pi)


def motor_current_A(torque_N_m: float) -> float:
    return torque_N_m / MOTOR_KT_N_M_PER_A


def copper_loss_W(current_peak_A: float) -> float:
    """I^2 R heat for sinusoidal three-phase drive at peak phase current `current_peak_A`.

    P = 3 * (I_peak / sqrt 2)^2 * R_phase = 1.5 * I_peak^2 * R_phase. This heat is generated
    inside a closed printed casing, which is why the holding load matters.
    """
    r_phase = MOTOR_RESISTANCE_PHASE_TO_PHASE_OHM / 2.0
    return 1.5 * current_peak_A ** 2 * r_phase


def backlash_pixels(backlash_deg: float, horizontal_fov_deg: float,
                    horizontal_pixels: int) -> float:
    """Image displacement from transmission backlash, in pixels.

    Direct drive has none, which is part of why ADR 0008 chose it. Kept because it is the
    argument against ever reintroducing a gearbox.
    """
    return backlash_deg * horizontal_pixels / horizontal_fov_deg


# --------------------------------------------------------------- the design point

#: The stabilizer specification the budget is built on. Initial guesses -- to be tuned.
SLEW_ANGLE_RAD = math.radians(10.0)   # correct 10 degrees ...
SLEW_TIME_S = 0.100                   # ... in 100 ms
CHASSIS_ACCEL_M_S2 = 3.0              # hard launch or braking; initial guess
#: Horizontal CoM offset after trimming with the R4 masses. A requirement, not a guess:
#: see design_point_report for why.
TRIMMED_HORIZONTAL_OFFSET_M = 0.002
#: Vertical offset if the owner chooses bottom-heavy. Open (balance is owner-reviewed).
BOTTOM_HEAVY_VERTICAL_OFFSET_M = 0.010


@dataclass(frozen=True)
class Budget:
    slew_N_m: float
    holding_N_m: float
    accel_N_m: float

    @property
    def total_N_m(self) -> float:
        return self.slew_N_m + self.holding_N_m + self.accel_N_m


def budget(model: CasingModel, horizontal_offset_m: float,
           vertical_offset_m: float) -> Budget:
    return Budget(
        slew_N_m=slew_torque_N_m(model.total_inertia_kg_m2, SLEW_ANGLE_RAD, SLEW_TIME_S),
        holding_N_m=gravity_torque_N_m(model.total_mass_kg, horizontal_offset_m),
        accel_N_m=acceleration_torque_N_m(model.total_mass_kg, vertical_offset_m,
                                          CHASSIS_ACCEL_M_S2),
    )


def report() -> None:
    m = build_model()
    print("=== ROTATING ASSEMBLY ===")
    print(f"  printed plastic (CAD): {m.plastic_mass_kg * 1000:5.0f} g   "
          f"I = {m.plastic_inertia_kg_m2 * 1e3:.3f}e-3")
    for c in m.contents:
        print(f"    {c.name:<34}{c.mass_kg * 1000:5.0f} g at r = {c.radius_m * 1000:4.0f} mm"
              f"   I = {c.inertia_kg_m2 * 1e6:5.2f}e-6   ({c.source})")
    print(f"  contents:              {m.contents_mass_kg * 1000:5.0f} g   "
          f"I = {m.contents_inertia_kg_m2 * 1e3:.3f}e-3")
    print(f"  TOTAL:                 {m.total_mass_kg * 1000:5.0f} g   "
          f"I = {m.total_inertia_kg_m2 * 1e3:.3f}e-3 kg m^2   "
          f"(shell {100 * m.shell_inertia_kg_m2 / m.total_inertia_kg_m2:.0f}%)")
    print()

    print("=== HOLDING TORQUE vs horizontal CoM offset (the dominant term) ===")
    for d_mm in (0, 2, 5, 10):
        tau = gravity_torque_N_m(m.total_mass_kg, d_mm / 1000)
        i = motor_current_A(tau)
        print(f"  {d_mm:>2} mm -> {tau * 1000:5.1f} mN m, {i:4.2f} A, "
              f"{copper_loss_W(i):4.2f} W of heat, continuously")
    print()

    print("=== SLEW TORQUE (I alpha) ===")
    for angle_deg, t_ms in ((10, 200), (10, 100), (10, 50), (30, 100)):
        tau = slew_torque_N_m(m.total_inertia_kg_m2, math.radians(angle_deg), t_ms / 1000)
        alpha = 4.0 * math.radians(angle_deg) / ((t_ms / 1000.0) ** 2)
        print(f"  {angle_deg:>2} deg in {t_ms:>3} ms (alpha {alpha:5.0f} rad/s^2) -> "
              f"{tau * 1000:5.1f} mN m")
    print()

    print("=== DESIGN POINT: 10 deg in 100 ms, 3 m/s^2 chassis acceleration ===")
    print(f"  motor rated {MOTOR_RATED_TORQUE_N_M * 1000:.1f} mN m at "
          f"{MOTOR_RATED_CURRENT_A} A (Kt {MOTOR_KT_N_M_PER_A * 1000:.0f} mN m/A)")
    cases = (("untrimmed, 10 mm sideways", 0.010, 0.0),
             ("trimmed to 2 mm, balanced", TRIMMED_HORIZONTAL_OFFSET_M, 0.0),
             ("trimmed to 2 mm, bottom-heavy 10 mm",
              TRIMMED_HORIZONTAL_OFFSET_M, BOTTOM_HEAVY_VERTICAL_OFFSET_M))
    for label, d_h, d_v in cases:
        b = budget(m, d_h, d_v)
        i = motor_current_A(b.total_N_m)
        print(f"  {label}")
        print(f"    slew {b.slew_N_m * 1000:4.1f} + holding {b.holding_N_m * 1000:4.1f} + "
              f"accel {b.accel_N_m * 1000:4.1f} = {b.total_N_m * 1000:4.1f} mN m "
              f"({100 * b.total_N_m / MOTOR_RATED_TORQUE_N_M:3.0f}% of rated), "
              f"{i:4.2f} A, {DRIVER_CURRENT_LIMIT_A / i:3.1f}x driver margin")
    print()

    print("=== PENDULUM RESONANCE (vertical offset, bottom-heavy option) ===")
    rows = [f"{d:>2} mm -> {pendulum_frequency_Hz(m.total_mass_kg, d / 1000, m.total_inertia_kg_m2):4.2f} Hz"
            for d in (2, 5, 10, 20)]
    print("  " + ",  ".join(rows))


if __name__ == "__main__":
    report()
