#!/usr/bin/env python3
"""Inertia and torque budget for the pitch axis.

Feeds the pitch actuator selection (docs/decisions/0006) and the theory note in
docs/theory/pitch-axis-inertia-and-torque.md. Offline analysis only -- nothing here runs on
the robot.

Every input is an estimate from the mechanical envelope, not a measurement. The point of
having this as a script rather than arithmetic in a document is that it can be re-run
against weighed parts, and the conclusions either survive or visibly do not.

Geometry from the owner, 2026-09-30:
    outer diameter          135-145 mm
    internal chassis dia    115-125 mm
    shell thickness         2.5-3 mm, printed
    rotational clearance    2-3 mm
    wheel-to-wheel          ~165 mm
    total mass target       700-900 g

Usage:
    python3 tools/pitch_inertia_budget.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.81  # m/s^2


@dataclass(frozen=True)
class Geometry:
    """Casing geometry in metres. The casing is a thin cylindrical shell."""

    outer_diameter_m: float
    shell_thickness_m: float
    axial_length_m: float

    @property
    def outer_radius_m(self) -> float:
        return self.outer_diameter_m / 2.0

    @property
    def inner_radius_m(self) -> float:
        return self.outer_radius_m - self.shell_thickness_m

    @property
    def wall_volume_m3(self) -> float:
        """Volume of the cylindrical wall, excluding end caps."""
        return math.pi * (self.outer_radius_m ** 2 - self.inner_radius_m ** 2) \
            * self.axial_length_m


@dataclass(frozen=True)
class PointMass:
    """A component treated as a point mass at a radius from the rotation axis."""

    name: str
    mass_kg: float
    radius_m: float

    @property
    def inertia_kg_m2(self) -> float:
        return self.mass_kg * self.radius_m ** 2


@dataclass
class CasingModel:
    geometry: Geometry
    material_density_kg_m3: float
    wall_fill_fraction: float          # 1.0 = solid walls; FDM 3 mm walls are near-solid
    end_cap_mass_kg: float
    contents: list[PointMass] = field(default_factory=list)

    @property
    def shell_mass_kg(self) -> float:
        return (self.geometry.wall_volume_m3 * self.material_density_kg_m3
                * self.wall_fill_fraction) + self.end_cap_mass_kg

    @property
    def shell_inertia_kg_m2(self) -> float:
        """Thick-walled cylinder about its own axis: I = m (r_i^2 + r_o^2) / 2.

        End cap mass is lumped in at the shell's mean radius, which overestimates slightly
        (caps are discs, I = m r^2 / 2 with mass spread inward). Erring high is the right
        direction for a torque budget.
        """
        g = self.geometry
        return 0.5 * self.shell_mass_kg * (g.inner_radius_m ** 2 + g.outer_radius_m ** 2)

    @property
    def contents_mass_kg(self) -> float:
        return sum(c.mass_kg for c in self.contents)

    @property
    def contents_inertia_kg_m2(self) -> float:
        return sum(c.inertia_kg_m2 for c in self.contents)

    @property
    def total_mass_kg(self) -> float:
        return self.shell_mass_kg + self.contents_mass_kg

    @property
    def total_inertia_kg_m2(self) -> float:
        return self.shell_inertia_kg_m2 + self.contents_inertia_kg_m2


def gravity_torque_N_m(mass_kg: float, com_offset_m: float) -> float:
    """Holding torque needed when the centre of mass is offset from the rotation axis.

    This is a *continuous* load, not a transient: the actuator holds it whenever the camera
    is pointed anywhere other than through the CoM. Balancing the casing drives it to zero.
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


def build_model(*, optimistic: bool) -> CasingModel:
    """Two corners of the envelope: light/small and heavy/large.

    `optimistic` takes the smallest diameter, thinnest wall and lightest parts.
    """
    if optimistic:
        geometry = Geometry(outer_diameter_m=0.135, shell_thickness_m=0.0025,
                            axial_length_m=0.115)
        density = 1040.0        # ABS, kg/m^3
        fill = 0.85
        caps = 0.020
        mcu_mass = 0.010        # bare STM32G474 module on a small custom-ish carrier
    else:
        geometry = Geometry(outer_diameter_m=0.145, shell_thickness_m=0.003,
                            axial_length_m=0.125)
        density = 1270.0        # PETG, kg/m^3
        fill = 1.0
        caps = 0.040
        mcu_mass = 0.060        # Nucleo-G474RE development board as-is

    # Radii are where the part sits relative to the rotation axis. Boards mounted against
    # the inner wall sit far out; anything on the axis contributes almost nothing.
    inner = geometry.inner_radius_m
    contents = [
        PointMass("Pi Zero 2 W", 0.011, inner * 0.75),
        PointMass("Camera Module 3 Wide", 0.005, inner * 0.90),
        PointMass("MCU board", mcu_mass, inner * 0.70),
        PointMass("IMU, near the axis per ADR 0005", 0.002, 0.005),
        PointMass("wiring, fasteners, brackets", 0.020 if optimistic else 0.040,
                  inner * 0.60),
    ]
    return CasingModel(geometry=geometry, material_density_kg_m3=density,
                       wall_fill_fraction=fill, end_cap_mass_kg=caps, contents=contents)


def report() -> None:
    for label, optimistic in (("LIGHT corner", True), ("HEAVY corner", False)):
        m = build_model(optimistic=optimistic)
        g = m.geometry
        print(f"=== {label} "
              f"(OD {g.outer_diameter_m * 1000:.0f} mm, wall {g.shell_thickness_m * 1000:.1f} mm, "
              f"L {g.axial_length_m * 1000:.0f} mm) ===")
        print(f"  shell: {m.shell_mass_kg * 1000:6.0f} g   "
              f"I = {m.shell_inertia_kg_m2 * 1e3:6.3f} e-3 kg m^2   "
              f"({100 * m.shell_inertia_kg_m2 / m.total_inertia_kg_m2:.0f}% of total)")
        print(f"  contents: {m.contents_mass_kg * 1000:3.0f} g   "
              f"I = {m.contents_inertia_kg_m2 * 1e3:6.3f} e-3 kg m^2")
        print(f"  TOTAL rotating: {m.total_mass_kg * 1000:.0f} g, "
              f"I = {m.total_inertia_kg_m2 * 1e3:.3f} e-3 kg m^2")
        print()
        print("  holding torque against gravity, by CoM offset from the axis:")
        for offset_mm in (0, 2, 5, 10, 20):
            tau = gravity_torque_N_m(m.total_mass_kg, offset_mm / 1000.0)
            print(f"    {offset_mm:>2} mm -> {tau * 1000:6.1f} mN m")
        print()
        print("  peak torque to correct a pitch error, by aggressiveness:")
        for angle_deg, t_ms in ((10, 200), (10, 100), (10, 50), (30, 100)):
            tau = slew_torque_N_m(m.total_inertia_kg_m2, math.radians(angle_deg),
                                  t_ms / 1000.0)
            alpha = 4.0 * math.radians(angle_deg) / ((t_ms / 1000.0) ** 2)
            print(f"    {angle_deg:>2} deg in {t_ms:>3} ms "
                  f"(alpha {alpha:6.0f} rad/s^2) -> {tau * 1000:6.1f} mN m")
        print()
        print("  what a reduction ratio does to the motor-side requirement")
        print("  (10 deg in 100 ms, plus 10 mm CoM offset):")
        tau_axis = (slew_torque_N_m(m.total_inertia_kg_m2, math.radians(10), 0.100)
                    + gravity_torque_N_m(m.total_mass_kg, 0.010))
        for ratio in (1, 4, 13, 30):
            reflected = m.total_inertia_kg_m2 / (ratio ** 2)
            print(f"    {ratio:>2}:1 -> motor torque {tau_axis * 1000 / ratio:6.1f} mN m, "
                  f"reflected inertia {reflected * 1e6:7.2f} e-6 kg m^2")
        print()


def pendulum_frequency_Hz(mass_kg: float, com_offset_m: float,
                          inertia_kg_m2: float) -> float:
    """Natural frequency of the casing swinging as a pendulum about the wheel axis.

    An unbalanced casing is a pendulum: omega = sqrt(m g d / I). This matters because the
    resonance sits inside the stabilization loop. A bottom-heavy casing passively keeps the
    camera roughly upright -- which is how throwable two-wheeled robots usually work -- but
    the same restoring torque fights the actuator whenever it points off-level, and the
    resonance has to be handled by the controller rather than wished away.
    """
    if com_offset_m <= 0.0:
        return 0.0
    return math.sqrt(mass_kg * G * com_offset_m / inertia_kg_m2) / (2.0 * math.pi)


def pendulum_report() -> None:
    print("=== pendulum resonance from an unbalanced casing ===")
    for label, optimistic in (("LIGHT", True), ("HEAVY", False)):
        m = build_model(optimistic=optimistic)
        rows = []
        for offset_mm in (2, 5, 10, 20):
            f = pendulum_frequency_Hz(m.total_mass_kg, offset_mm / 1000.0,
                                      m.total_inertia_kg_m2)
            rows.append(f"{offset_mm:>2} mm -> {f:4.2f} Hz")
        print(f"  {label}: " + ",  ".join(rows))
    print()


def backlash_pixels(backlash_deg: float, horizontal_fov_deg: float,
                    horizontal_pixels: int) -> float:
    """Image displacement from transmission backlash, in pixels.

    The research question is how much active stabilization improves *visual* stability, so
    backlash is not a side effect -- it lands directly in the measurement.
    """
    return backlash_deg * horizontal_pixels / horizontal_fov_deg


def backlash_report() -> None:
    # FOV is an assumption to confirm against the Camera Module 3 Wide datasheet.
    fov, px = 90.0, 1920
    print(f"=== backlash as image jitter (assumes {fov:.0f} deg horizontal FOV, "
          f"{px} px wide) ===")
    for deg in (0.1, 0.5, 1.0, 2.0):
        print(f"  {deg:>4.1f} deg backlash -> {backlash_pixels(deg, fov, px):5.0f} px of jitter")


if __name__ == "__main__":
    report()
    pendulum_report()
    backlash_report()
