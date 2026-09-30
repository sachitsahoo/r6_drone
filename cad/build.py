#!/usr/bin/env python3
"""Export the Recon UGV parts and report mass and inertia.

Two jobs:

1. Write STEP (for editing in any CAD package) and STL (for printing) to cad/out/.
2. Report mass and rotating inertia computed from the actual solids, which replaces the
   hand estimates in docs/theory/pitch-axis-inertia-and-torque.md.

That second job is the point. The torque budget was built on an estimated shell mass and a
guess at the contents; the geometry now exists, so the numbers can come from it. The first
build came out at 855 g of plastic against a 700-900 g vehicle budget, which no estimate
had caught.

Usage:
    python3 cad/build.py            # export everything and report
    python3 cad/build.py --report   # report only, no files written
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt
from OCP.GProp import GProp_GProps

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parameters as P
import parts

# Which parts rotate with the casing. This is what the pitch actuator has to accelerate,
# and therefore what ADR 0006's torque budget is about.
ROTATING = {
    "01_casing_shell": 1,
    "02_casing_end_cap": 1,
    "02b_casing_end_cap_with_datum": 1,
    "05_pitch_motor_bracket": 1,
    "06_imu_bridge": 1,
    "07_camera_mount": 1,
}

# Everything printed, with quantities, for the vehicle mass budget.
QUANTITIES = {
    "01_casing_shell": 1,
    "02_casing_end_cap": 1,
    "02b_casing_end_cap_with_datum": 1,
    "03_chassis_disc": 2,
    "04_chassis_drive_band": 1,
    "05_pitch_motor_bracket": 1,
    "06_imu_bridge": 1,
    "07_camera_mount": 1,
    "08_wheel": 2,
}


def volume_mm3(shape: cq.Shape) -> float:
    return shape.Volume()


def moment_about_z_mm5(shape: cq.Shape) -> float:
    """Volumetric second moment about the Z axis, in mm^5.

    Multiply by density to get a mass moment of inertia. Uses OCP's exact volume
    properties rather than a mesh approximation.
    """
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    axis = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0))
    return props.MomentOfInertia(axis)


def inertia_kg_m2(shape: cq.Shape, density_g_mm3: float) -> float:
    """Mass moment of inertia about Z, in kg m^2.

    g mm^2 -> kg m^2 is a factor of 1e-9: grams to kg is 1e-3, mm^2 to m^2 is 1e-6.
    """
    return moment_about_z_mm5(shape) * density_g_mm3 * 1e-9


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="store_true", help="report only, write nothing")
    parser.add_argument("--out-dir", type=Path,
                        default=Path(__file__).resolve().parent / "out")
    parser.add_argument("--density", type=float, default=P.PRINT_DENSITY,
                        help="g/mm^3; default is the material in parameters.py")
    args = parser.parse_args(argv[1:])

    built = {name: fn() for name, fn in parts.ALL_PARTS.items()}

    if not args.report:
        args.out_dir.mkdir(parents=True, exist_ok=True)
        for name, wp in built.items():
            cq.exporters.export(wp, str(args.out_dir / f"{name}.step"))
            cq.exporters.export(wp, str(args.out_dir / f"{name}.stl"))
        print(f"exported {2 * len(built)} files to {args.out_dir}\n")

    rho = args.density
    print(f"=== MASS BUDGET (density {rho * 1000:.2f} g/cm^3) ===")
    print(f"{'part':<34}{'qty':>4}{'each g':>9}{'total g':>9}")
    total_mass = 0.0
    for name, wp in built.items():
        qty = QUANTITIES.get(name, 0)
        if qty == 0:
            continue
        mass = volume_mm3(wp.val()) * rho
        total_mass += mass * qty
        print(f"{name:<34}{qty:>4}{mass:>9.1f}{mass * qty:>9.1f}")
    print(f"{'PRINTED PLASTIC TOTAL':<34}{'':>4}{'':>9}{total_mass:>9.1f}")
    print(f"  vehicle target 700-900 g -> leaves {700 - total_mass:.0f} to "
          f"{900 - total_mass:.0f} g for motors, battery, electronics, bearings, axle")
    print()

    print("=== ROTATING ASSEMBLY (what the pitch actuator accelerates) ===")
    rot_mass = 0.0
    rot_inertia = 0.0
    for name, qty in ROTATING.items():
        shape = built[name].val()
        mass = volume_mm3(shape) * rho * qty
        inertia = inertia_kg_m2(shape, rho) * qty
        rot_mass += mass
        rot_inertia += inertia
        print(f"  {name:<34}{mass:>8.1f} g   I = {inertia * 1e3:>7.3f} e-3 kg m^2")
    print(f"  {'TOTAL':<34}{rot_mass:>8.1f} g   I = {rot_inertia * 1e3:>7.3f} e-3 kg m^2")
    print()

    print("=== versus the estimate in docs/theory/pitch-axis-inertia-and-torque.md ===")
    print(f"  estimated: 174 g / 0.647e-3 (light) to 370 g / 1.534e-3 (heavy) kg m^2")
    print(f"  measured from geometry: {rot_mass:.0f} g / {rot_inertia * 1e3:.3f}e-3 kg m^2")
    print()
    print("  Note: the pitch motor itself is not modelled and rides in the casing (R3).")
    print("  Add its contribution before trusting the torque budget.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
