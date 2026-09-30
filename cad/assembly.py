#!/usr/bin/env python3
"""Assemble the parts at their intended positions and check clearances numerically.

Two reasons this exists beyond a picture:

1. The individual part exports all sit at the origin, so there is nothing for a CAD
   package's interference detection to chew on. This writes one positioned STEP.
2. Minimum distance between solids can be computed exactly, which is better than eyeballing
   a section view. Interference shows up as a distance of zero with overlapping volume.

Note what this does NOT check: whether a printed part achieves its nominal dimensions. FDM
runs holes undersized and outer walls oversized, so a 2.5 mm designed gap can land anywhere
from roughly 1.5 to 3.5 mm. That is a manufacturing question, not a geometry one.

The axial layout below is a PROPOSAL. Nothing in the analysis so far fixes it.

Usage:
    python3 cad/assembly.py            # export assembly STEP + report clearances
    python3 cad/assembly.py --report   # report only
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parameters as P
import parts

# --------------------------------------------------------------------- axial layout
#
# z = 0 at the outboard face of one casing end. PROPOSAL, not derived.
Z_CASING_START = 0.0
Z_END_CAP_A = 0.0
Z_END_CAP_B = P.CASING_LENGTH - P.END_CAP_THICKNESS          # 116
Z_CHASSIS_DISC_A = 12.0        # placed so the mirrored boss tip lands on z = 0
Z_DRIVE_BAND = 88.0
Z_CHASSIS_DISC_B = 108.0       # boss points outward from here toward end cap B
Z_IMU_BRIDGE = 40.0
Z_CAMERA_MOUNT = P.CASING_LENGTH / 2
Z_MOTOR_BRACKET = 70.0
Z_WHEEL_A = -P.WHEEL_WIDTH - 2.0
Z_WHEEL_B = P.CASING_LENGTH + 2.0


def min_distance_mm(a: cq.Shape, b: cq.Shape) -> float:
    """Exact minimum distance between two solids, in mm. Zero means touching or overlapping."""
    calc = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    calc.Perform()
    return calc.Value()


def overlaps(a: cq.Shape, b: cq.Shape) -> bool:
    """True if the solids share volume, which min_distance alone cannot distinguish."""
    try:
        return cq.Workplane(obj=a).intersect(cq.Workplane(obj=b)).val().Volume() > 1e-6
    except Exception:
        return False


def build_assembly() -> dict[str, cq.Workplane]:
    """Every part translated to its assembled position."""
    shell = parts.casing_shell()
    cap_a = parts.casing_end_cap(with_datum=True).translate((0, 0, Z_END_CAP_A))
    cap_b = parts.casing_end_cap().translate((0, 0, Z_END_CAP_B))
    # Mirrored, like disc B: the boss has to point OUTWARD toward its end cap's bearing.
    # An earlier version left disc A unmirrored, so its boss pointed inboard and the casing
    # had no bearing support at that end -- 14 mm of air where the bearing should be.
    disc_a = (parts.chassis_disc()
              .rotate((0, 0, 0), (1, 0, 0), 180)
              .translate((0, 0, Z_CHASSIS_DISC_A + P.CHASSIS_DISC_THICKNESS)))
    # NOT mirrored: disc A and disc B need OPPOSITE orientations, since each boss points
    # outward toward its own end cap. Mirroring both left disc B's boss pointing inboard,
    # 8 mm short of the bearing it is supposed to carry.
    disc_b = parts.chassis_disc().translate((0, 0, Z_CHASSIS_DISC_B))
    band = parts.chassis_drive_band().translate((0, 0, Z_DRIVE_BAND))
    bridge = parts.imu_bridge().translate((0, 0, Z_IMU_BRIDGE))
    camera = (parts.camera_mount()
              .rotate((0, 0, 0), (0, 1, 0), 90)
              .translate((P.CASING_ID / 2 - 6.0, 0, Z_CAMERA_MOUNT)))
    bracket = (parts.pitch_motor_bracket()
               .translate((P.DRIVE_PULLEY_CENTER_RADIUS, 0, Z_MOTOR_BRACKET)))
    wheel_a = parts.wheel().translate((0, 0, Z_WHEEL_A))
    wheel_b = parts.wheel().translate((0, 0, Z_WHEEL_B))

    return {
        "casing_shell": shell,
        "end_cap_a": cap_a,
        "end_cap_b": cap_b,
        "chassis_disc_a": disc_a,
        "chassis_disc_b": disc_b,
        "drive_band": band,
        "imu_bridge": bridge,
        "camera_mount": camera,
        "pitch_motor_bracket": bracket,
        "wheel_a": wheel_a,
        "wheel_b": wheel_b,
    }


# Pairs worth checking, with the clearance the design intends. None means "must not touch,
# no specific target". A target of 0.0 means contact is expected by design.
CHECKS: list[tuple[str, str, float | None, str]] = [
    ("casing_shell", "chassis_disc_a", P.ROTATIONAL_CLEARANCE, "rotational clearance"),
    ("casing_shell", "chassis_disc_b", P.ROTATIONAL_CLEARANCE, "rotational clearance"),
    ("casing_shell", "drive_band", None, "band sits well inside the shell"),
    ("casing_shell", "wheel_a", None, "wheel must clear the rotating casing"),
    ("casing_shell", "wheel_b", None, "wheel must clear the rotating casing"),
    ("chassis_disc_a", "drive_band", None, "both are chassis; contact is fine"),
    ("end_cap_a", "chassis_disc_a", None, "bearing sits between; overlap checked separately"),
    ("end_cap_b", "chassis_disc_b", None, "bearing sits between; overlap checked separately"),
    ("imu_bridge", "chassis_disc_a", None, "bridge must not foul the chassis"),
    ("imu_bridge", "drive_band", None, "bridge must not foul the band"),
    ("pitch_motor_bracket", "drive_band", None, "pulley engages here; bracket must clear"),
    ("pitch_motor_bracket", "casing_shell", None, "bracket mounts to the shell"),
    ("camera_mount", "chassis_disc_a", None, "camera must not foul the chassis"),
]


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--out-dir", type=Path,
                        default=Path(__file__).resolve().parent / "out")
    args = parser.parse_args(argv[1:])

    assembly = build_assembly()

    if not args.report:
        args.out_dir.mkdir(parents=True, exist_ok=True)
        combined = cq.Workplane("XY")
        for wp in assembly.values():
            combined = combined.add(wp)
        target = args.out_dir / "00_assembly.step"
        cq.exporters.export(combined, str(target))
        print(f"wrote {target}\n")

    print("=== CLEARANCE REPORT (exact minimum distance between solids) ===")
    print(f"{'pair':<44}{'gap mm':>9}{'target':>9}  note")
    problems: list[str] = []
    for a_name, b_name, target, note in CHECKS:
        a, b = assembly[a_name].val(), assembly[b_name].val()
        gap = min_distance_mm(a, b)
        clash = gap < 1e-6 and overlaps(a, b)
        flag = "INTERFERENCE" if clash else ""
        target_text = f"{target:.1f}" if target is not None else "-"
        print(f"{a_name + ' / ' + b_name:<44}{gap:>9.2f}{target_text:>9}  {note} {flag}")
        if clash:
            problems.append(f"{a_name} / {b_name}")
        elif target is not None and abs(gap - target) > 0.05:
            problems.append(f"{a_name} / {b_name}: gap {gap:.2f} mm, expected {target:.2f} mm")

    # Axial overlap of each boss with its end cap's bearing seat. A minimum-distance check
    # cannot see this: the boss can be concentric with the cap and still miss it axially,
    # which is exactly the failure the first two assembly attempts had.
    print()
    print("=== BEARING SEAT ENGAGEMENT (axial overlap, not distance) ===")
    for cap_name, disc_name in (("end_cap_a", "chassis_disc_a"),
                                ("end_cap_b", "chassis_disc_b")):
        cap_bb = assembly[cap_name].val().BoundingBox()
        disc_bb = assembly[disc_name].val().BoundingBox()
        overlap = min(cap_bb.zmax, disc_bb.zmax) - max(cap_bb.zmin, disc_bb.zmin)
        ok = overlap >= P.BEARING_WIDTH
        print(f"  {cap_name} / {disc_name}: {overlap:.1f} mm of axial overlap "
              f"(need {P.BEARING_WIDTH:.1f} for the bearing) {'OK' if ok else 'TOO SHORT'}")
        if not ok:
            problems.append(f"{cap_name} / {disc_name}: only {overlap:.1f} mm of "
                            f"bearing engagement")

    print()
    if problems:
        print(f"{len(problems)} problem(s):")
        for p in problems:
            print(f"  - {p}")
    else:
        print("no interference, and every targeted clearance is as designed")

    print()
    print("NOT checked here: printed dimensional accuracy. FDM runs holes undersized and")
    print("walls oversized, so a 2.5 mm designed gap can land near 1.5-3.5 mm in practice.")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
