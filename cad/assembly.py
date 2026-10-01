#!/usr/bin/env python3
"""Assemble the parts at their intended positions and check clearances numerically.

Three reasons this exists beyond a picture:

1. The individual part exports all sit at the origin, so there is nothing for a CAD
   package's interference detection to chew on. This writes one positioned STEP.
2. Minimum distance between solids can be computed exactly, which is better than eyeballing
   a section view. Interference shows up as a distance of zero with overlapping volume.
3. **The casing turns relative to the chassis, and the wheels turn relative to both.** A
   check at one pose says nothing about the next one. The first version of this file swept
   a single pose and passed an IMU bridge that went straight through a chassis standoff --
   hidden only because the standoffs were holes, never modelled as rods. Parts in different
   rotation groups are now checked over a full relative turn.

Note what this does NOT check: whether a printed part achieves its nominal dimensions. FDM
runs holes undersized and outer walls oversized, so a 0.5 mm designed gap can close entirely.
That is a manufacturing question, not a geometry one.

The axial layout comes from the derived stations in parameters.py, not from numbers here.

Usage:
    python3 cad/assembly.py            # export assembly STEP + report clearances
    python3 cad/assembly.py --report   # report only
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parameters as P
import parts

#: Angular step for the swept check. A feature narrower than r * step could slip between
#: samples: at the 32.5 mm bore, 5 degrees is 2.8 mm. Every printed feature that crosses a
#: rotation boundary is wider than that, and pairs that cannot meet at any angle are proven
#: clear exactly by the radial test below rather than sampled.
SWEEP_STEP_DEG = 5.0

# Which body each part moves with. Parts in different groups rotate relative to each other
# about the Z axis, so their clearance has to hold at every relative angle.
ROTATION_GROUP = {
    "casing_shell": "casing",
    "end_cap_a": "casing",
    "end_cap_b": "casing",
    "imu_bridge": "casing",
    "camera_mount": "casing",
    "chassis_spine": "chassis",
    "wheel_motor_a": "chassis",
    "wheel_motor_b": "chassis",
    # Stator on the chassis, rotor on the casing. As a plain envelope it is a cylinder, so
    # which group it belongs to does not change any result.
    "pitch_motor_envelope": "motor",
    "wheel_a": "wheel_a",
    "wheel_shaft_a": "wheel_a",
    "wheel_b": "wheel_b",
}

#: Bodies of revolution: rotating them changes nothing, so one pose is an exact check.
AXISYMMETRIC = {"pitch_motor_envelope", "wheel_motor_a", "wheel_motor_b", "wheel_shaft_a"}


def min_distance_mm(a: cq.Shape, b: cq.Shape) -> float:
    """Exact minimum distance between two solids, in mm. Zero means touching or overlapping."""
    calc = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    calc.Perform()
    return calc.Value()


def overlaps(a: cq.Shape, b: cq.Shape) -> bool:
    """True if the solids share volume, which min_distance alone cannot distinguish."""
    return shared_volume_mm3(a, b) > 1e-6


def shared_volume_mm3(a: cq.Shape, b: cq.Shape) -> float:
    try:
        return cq.Workplane(obj=a).intersect(cq.Workplane(obj=b)).val().Volume()
    except Exception:
        return 0.0


def _cylinder(diameter: float, z_lo: float, z_hi: float, bore: float = 0.0) -> cq.Workplane:
    wp = cq.Workplane("XY", origin=(0, 0, z_lo)).circle(diameter / 2)
    if bore > 0.0:
        wp = wp.circle(bore / 2)
    return wp.extrude(z_hi - z_lo)


def build_assembly() -> dict[str, cq.Workplane]:
    """Every part translated to its assembled position, plus envelopes for bought parts
    that sit on the axis."""
    shell = parts.casing_shell()
    cap_a = parts.casing_end_cap_a().translate((0, 0, P.Z_END_CAP_A))
    # Flipped: the cap's bearing seat opens toward +Z in the part, and at this end of the
    # casing "inboard" is -Z.
    cap_b = (parts.casing_end_cap_b()
             .rotate((0, 0, 0), (1, 0, 0), 180)
             .translate((0, 0, P.Z_END_CAP_B + P.END_CAP_THICKNESS)))
    spine = parts.chassis_spine()                       # already built in casing coordinates
    # Arm pointing away from the camera, which is on +X.
    bridge = (parts.imu_bridge()
              .rotate((0, 0, 0), (0, 0, 1), 180)
              .translate((0, 0, P.Z_IMU_BRIDGE)))
    camera = (parts.camera_mount()
              .rotate((0, 0, 0), (0, 1, 0), 90)
              .translate((P.CAMERA_MOUNT_INNER_RADIUS, 0, P.Z_CAMERA)))

    # Envelopes, not printed parts: plain cylinders standing in for bought parts so the
    # sweep can see them. The pitch motor was invisible to every check in the first design
    # until it got one, which is how a bracket ended up inside its body.
    motor = _cylinder(P.PITCH_MOTOR_OD, P.Z_MOTOR_LO, P.Z_MOTOR_HI, P.PITCH_MOTOR_HOLLOW_BORE)
    wheel_motor_a = _cylinder(P.WHEEL_MOTOR_OD, P.Z_WHEEL_MOTOR_A_LO, P.Z_WHEEL_MOTOR_A_HI)
    wheel_motor_b = _cylinder(P.WHEEL_MOTOR_OD, P.Z_WHEEL_MOTOR_B_LO, P.Z_WHEEL_MOTOR_B_HI)
    # Wheel A's shaft runs from wheel motor A's face, through the pitch motor's hollow bore
    # and cap A, to the wheel. This is the part that forces a hollow-shaft motor.
    shaft_a = _cylinder(P.WHEEL_SHAFT_DIA, P.Z_WHEEL_A, P.Z_WHEEL_MOTOR_A_LO)

    wheel_a = parts.wheel().translate((0, 0, P.Z_WHEEL_A))
    wheel_b = parts.wheel().translate((0, 0, P.Z_WHEEL_B))

    return {
        "casing_shell": shell,
        "end_cap_a": cap_a,
        "end_cap_b": cap_b,
        "chassis_spine": spine,
        "imu_bridge": bridge,
        "camera_mount": camera,
        "pitch_motor_envelope": motor,
        "wheel_motor_a": wheel_motor_a,
        "wheel_motor_b": wheel_motor_b,
        "wheel_shaft_a": shaft_a,
        "wheel_a": wheel_a,
        "wheel_b": wheel_b,
    }


# ------------------------------------------------------------------ the sweep itself

@dataclass(frozen=True)
class PairResult:
    """Outcome of checking one pair. `gap_mm` is the minimum over every relative angle the
    pair can reach (or a proven lower bound, when `method` is "radial")."""

    a: str
    b: str
    method: str          # "radial", "static" or "swept"
    gap_mm: float
    shared_mm3: float    # > 0 means interference
    worst_angle_deg: float = 0.0


def _radial_extent(shape: cq.Shape) -> tuple[float, float]:
    """(r_min, r_max) about the Z axis. r_min is exact; r_max is a safe upper bound."""
    axis = cq.Edge.makeLine(cq.Vector(0, 0, -1e4), cq.Vector(0, 0, 1e4))
    calc = BRepExtrema_DistShapeShape(shape.wrapped, axis.wrapped)
    calc.Perform()
    r_min = calc.Value()
    bb = shape.BoundingBox()
    r_max = max(math.hypot(x, y) for x in (bb.xmin, bb.xmax) for y in (bb.ymin, bb.ymax))
    return r_min, r_max


def check_pair(a_name: str, b_name: str, a: cq.Shape, b: cq.Shape,
               prune: bool = True) -> PairResult:
    """Clearance between two parts over every relative pose they can reach.

    Cheapest exact test first. Two parts whose (r, z) extents are disjoint cannot meet at
    any angle, because rotation about Z preserves both r and z. Only pairs that fail that
    test, rotate relative to each other, and are not bodies of revolution get sampled.

    The radial test proves "no contact" but its gap is only a lower bound (r_max comes
    from the bounding box). Pass prune=False to measure the actual worst-case gap.
    """
    if prune:
        a_bb, b_bb = a.BoundingBox(), b.BoundingBox()
        z_gap = max(a_bb.zmin - b_bb.zmax, b_bb.zmin - a_bb.zmax)
        a_r, b_r = _radial_extent(a), _radial_extent(b)
        r_gap = max(a_r[0] - b_r[1], b_r[0] - a_r[1])
        if z_gap > 0.0 or r_gap > 0.0:
            return PairResult(a_name, b_name, "radial", max(z_gap, r_gap), 0.0)

    same_body = ROTATION_GROUP[a_name] == ROTATION_GROUP[b_name]
    if same_body or a_name in AXISYMMETRIC or b_name in AXISYMMETRIC:
        gap = min_distance_mm(a, b)
        shared = shared_volume_mm3(a, b) if gap < 1e-6 else 0.0
        return PairResult(a_name, b_name, "static", gap, shared)

    worst_gap, worst_angle, worst_shared = math.inf, 0.0, 0.0
    steps = int(round(360.0 / SWEEP_STEP_DEG))
    for i in range(steps):
        angle = i * SWEEP_STEP_DEG
        a_rot = a.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), angle)
        gap = min_distance_mm(a_rot, b)
        shared = shared_volume_mm3(a_rot, b) if gap < 1e-6 else 0.0
        if shared > worst_shared or (worst_shared == 0.0 and gap < worst_gap):
            worst_gap, worst_angle, worst_shared = gap, angle, shared
    return PairResult(a_name, b_name, "swept", worst_gap, worst_shared, worst_angle)


def sweep(assembly: dict[str, cq.Workplane]) -> dict[tuple[str, str], PairResult]:
    """EVERY pair, deliberately. An earlier version checked a hand-picked 13 of 55 pairs and
    missed four real interferences -- a curated check reads as a clean bill of health."""
    return {(a, b): check_pair(a, b, assembly[a].val(), assembly[b].val())
            for a, b in itertools.combinations(assembly, 2)}


def bearing_engagement(race_holder: cq.Shape, r_lo: float, r_hi: float,
                       z_lo: float, z_hi: float) -> float:
    """Fraction of a bearing's width that is backed by material, from 0 to 1.

    A minimum-distance check cannot see this: a boss can be concentric with its cap and
    still miss it axially, which both of the first two assembly attempts did. So probe a
    thin ring just inside (or outside) the race and measure how much of it is solid.
    """
    probe = _cylinder(2 * r_hi, z_lo, z_hi, 2 * r_lo).val()
    return shared_volume_mm3(race_holder, probe) / probe.Volume()


def bearing_checks(assembly: dict[str, cq.Workplane]) -> list[tuple[str, float]]:
    """(description, engaged fraction) for both races of both casing bearings."""
    spine = assembly["chassis_spine"].val()
    cap_a = assembly["end_cap_a"].val()
    cap_b = assembly["end_cap_b"].val()
    probe = 0.4   # mm: radial depth of the probe ring on each side of the race

    a_lo, a_hi = P.Z_MOTOR_LO - P.BEARING_A_WIDTH, P.Z_MOTOR_LO
    b_lo, b_hi = P.Z_END_CAP_B, P.Z_END_CAP_B + P.BEARING_WIDTH
    return [
        ("A inner race on the cup wall",
         bearing_engagement(spine, P.BEARING_A_ID / 2 - probe, P.BEARING_A_ID / 2, a_lo, a_hi)),
        ("A outer race in cap A",
         bearing_engagement(cap_a, P.BEARING_A_OD / 2, P.BEARING_A_OD / 2 + probe, a_lo, a_hi)),
        ("B inner race on the boss",
         bearing_engagement(spine, P.BEARING_ID / 2 - probe, P.BEARING_ID / 2, b_lo, b_hi)),
        ("B outer race in cap B",
         bearing_engagement(cap_b, P.BEARING_OD / 2, P.BEARING_OD / 2 + probe, b_lo, b_hi)),
    ]


# Pairs with a designed clearance. A target of 0.0 means contact is expected by design
# (bolted faces). The gap compared is the worst over every reachable relative angle.
CHECKS: list[tuple[str, str, float, str]] = [
    ("end_cap_a", "chassis_spine", P.RUNNING_CLEARANCE, "cap A hub inside the cup, and cup tip"),
    ("end_cap_b", "chassis_spine", P.RUNNING_CLEARANCE, "cap B bore around the boss"),
    ("imu_bridge", "chassis_spine", P.IMU_WAIST_CLEARANCE, "IMU pad hole around the waist"),
    ("pitch_motor_envelope", "end_cap_a", 0.0, "rotor bell bolted to cap A"),
    ("pitch_motor_envelope", "chassis_spine", 0.0, "stator bolted to the cup floor"),
    ("wheel_shaft_a", "pitch_motor_envelope",
     (P.PITCH_MOTOR_HOLLOW_BORE - P.WHEEL_SHAFT_DIA) / 2, "shaft through the hollow bore"),
    ("wheel_shaft_a", "end_cap_a",
     (P.END_CAP_A_SHAFT_BORE - P.WHEEL_SHAFT_DIA) / 2, "shaft through cap A"),
    ("wheel_motor_a", "chassis_spine", 0.0, "seated against pocket A's end wall"),
    ("wheel_motor_b", "chassis_spine", 0.0, "seated against pocket B's end wall"),
]


def measured_gap(assembly: dict[str, cq.Workplane], a: str, b: str) -> PairResult:
    """The actual worst-case gap for one pair, never the radial lower bound."""
    return check_pair(a, b, assembly[a].val(), assembly[b].val(), prune=False)


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

    problems: list[str] = []

    results = sweep(assembly)
    methods = {m: sum(r.method == m for r in results.values())
               for m in ("radial", "static", "swept")}
    print(f"=== INTERFERENCE SWEEP ({len(assembly)} parts, {len(results)} pairs: "
          f"{methods['radial']} clear by radius/axial extent, {methods['static']} one pose, "
          f"{methods['swept']} swept over a full turn at {SWEEP_STEP_DEG:g} deg) ===")
    clashes = sorted((r for r in results.values() if r.shared_mm3 > 1e-6),
                     key=lambda r: -r.shared_mm3)
    for r in clashes:
        where = f" at {r.worst_angle_deg:g} deg" if r.method == "swept" else ""
        print(f"  INTERFERENCE  {r.a} / {r.b}: {r.shared_mm3:.0f} mm3 shared{where}")
        problems.append(f"{r.a} / {r.b}")
    if not clashes:
        print("  no interference in any pair, at any reachable angle")
    print()

    print("=== TARGETED CLEARANCES (worst case over rotation) ===")
    print(f"{'pair':<46}{'gap mm':>8}{'target':>8}  {'method':<7} note")
    for a_name, b_name, target, note in CHECKS:
        r = measured_gap(assembly, a_name, b_name)
        print(f"{a_name + ' / ' + b_name:<46}{r.gap_mm:>8.2f}{target:>8.2f}  {r.method:<7} {note}")
        if abs(r.gap_mm - target) > 0.05:
            problems.append(f"{a_name} / {b_name}: gap {r.gap_mm:.2f} mm, expected {target:.2f}")

    print()
    print("=== BEARING SEAT ENGAGEMENT (material behind each race, not distance) ===")
    for description, fraction in bearing_checks(assembly):
        ok = fraction > 0.99
        print(f"  {description:<32}{100 * fraction:>6.1f}% {'OK' if ok else 'NOT SUPPORTED'}")
        if not ok:
            problems.append(f"bearing {description}: only {100 * fraction:.0f}% supported")

    print()
    print(f"IMU radial offset {P.IMU_RADIAL_OFFSET:.2f} mm (R2 as amended: under 5 mm)")
    if P.IMU_RADIAL_OFFSET >= 5.0:
        problems.append(f"IMU offset {P.IMU_RADIAL_OFFSET:.2f} mm")

    print()
    if problems:
        print(f"{len(problems)} problem(s):")
        for p in problems:
            print(f"  - {p}")
    else:
        print("no interference, and every targeted clearance is as designed")

    print()
    print("NOT checked here: printed dimensional accuracy. FDM runs holes undersized and")
    print("walls oversized, so a 0.5 mm designed running gap can close entirely in practice.")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
