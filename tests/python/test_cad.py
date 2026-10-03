"""Tests for the parametric CAD model.

Skipped unless cadquery is installed, which CI deliberately does not do -- it pulls VTK,
numba, scipy and matplotlib, none of which the firmware or protocol tests need. Run
locally with `pip install -r requirements-cad.txt`.

The mass budget assertion is the one that earns its keep. The first build of these parts
came out at 855 g of printed plastic against a 700-900 g vehicle target, which no estimate
had caught. A parameter change that reintroduces that should fail a test, not be discovered
after printing.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

cq = pytest.importorskip("cadquery", reason="pip install -r requirements-cad.txt")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "cad"))

import build  # noqa: E402
import parameters as P  # noqa: E402
import parts  # noqa: E402


@pytest.fixture(scope="module")
def built():
    return {name: fn() for name, fn in parts.ALL_PARTS.items()}


def test_every_part_builds_as_a_solid(built) -> None:
    assert len(built) == 8, "seven printed parts once direct drive deleted the belt train, " \
        "plus the TPU tire (ADR 0017)"
    for name, wp in built.items():
        solid = wp.val()
        assert solid.Volume() > 0.0, f"{name} has no volume"
        assert solid.isValid(), f"{name} is not a valid solid"


def test_inertia_routine_matches_the_closed_form() -> None:
    """A tube has I = m (ri^2 + ro^2) / 2. If this drifts, every inertia figure is wrong."""
    r_o, r_i, length, rho = 67.5, 64.5, 120.0, P.ABS_DENSITY
    tube = cq.Workplane("XY").circle(r_o).circle(r_i).extrude(length).val()
    mass_g = tube.Volume() * rho
    analytic = 0.5 * (mass_g * 1e-3) * ((r_i * 1e-3) ** 2 + (r_o * 1e-3) ** 2)
    assert math.isclose(build.inertia_kg_m2(tube, rho), analytic, rel_tol=1e-6)


def test_casing_shell_outer_diameter_matches_parameters(built) -> None:
    bb = built["01_casing_shell"].val().BoundingBox()
    assert math.isclose(bb.xlen, P.CASING_OD, abs_tol=0.01)
    assert math.isclose(bb.zlen, P.CASING_LENGTH, abs_tol=0.01)


def test_overall_width_matches_adr_0008() -> None:
    """214 mm wheel face to wheel face, the number ADR 0008's proportions were chosen at."""
    overall = (P.Z_WHEEL_B + P.WHEEL_WIDTH) - P.Z_WHEEL_A
    assert math.isclose(overall, P.OVERALL_WIDTH, abs_tol=1e-9)


def test_wheel_shaft_passes_through_the_pitch_motor() -> None:
    """The topology constraint ADR 0010 found: with the motor coaxial at end A, wheel A's
    drive has nowhere to go except through the motor's centre. A solid-shaft motor makes
    the layout impossible, so the bore must clear the shaft."""
    assert P.PITCH_MOTOR_HOLLOW_BORE > P.WHEEL_SHAFT_DIA
    assert P.END_CAP_A_SHAFT_BORE > P.WHEEL_SHAFT_DIA


def test_pitch_motor_fits_in_the_cup() -> None:
    """Radially inside the cup wall with clearance, and axially between cap A and the cup
    floor. The first design checked the motor's diameter for months and its 26 mm length
    never, until the real part was looked up."""
    assert P.CUP_ID >= P.PITCH_MOTOR_OD + 2 * P.MOTOR_CUP_RADIAL_CLEARANCE - 1e-9
    assert math.isclose(P.Z_MOTOR_HI - P.Z_MOTOR_LO, P.PITCH_MOTOR_LENGTH)
    assert P.Z_MOTOR_LO >= P.Z_END_CAP_A + P.END_CAP_A_THICKNESS
    assert P.CUP_OD < P.CASING_ID, "the cup must sit inside the casing"


def test_motor_bolt_circles_land_in_material() -> None:
    """The rotor bolts go into the cup floor around its coupler bore; the stator bolts go into
    cap A around the shaft bore, inside the bearing groove. A bolt circle that falls inside a
    bore, or into the groove, has nothing to thread into."""
    hole = P.M2_5_CLEARANCE / 2
    assert P.PITCH_MOTOR_ROTOR_BOLT_RADIUS - hole > P.CUP_FLOOR_BORE / 2
    assert P.PITCH_MOTOR_ROTOR_BOLT_RADIUS + hole < P.CUP_ID / 2
    assert P.PITCH_MOTOR_STATOR_BOLT_RADIUS - hole > P.END_CAP_A_SHAFT_BORE / 2
    assert P.PITCH_MOTOR_STATOR_BOLT_RADIUS + hole < P.END_CAP_A_GROOVE_INNER_R


def test_cap_a_screws_clear_the_bearing_groove() -> None:
    """At 40 mm the motor pushes the 6709's groove out to r = 27.5, close to the cap screw
    circle at r = 30.25. A screw hole breaking into the groove would leave the bearing's
    outer race unsupported on that side."""
    assert P.END_CAP_SCREW_RADIUS - P.M3_CLEARANCE / 2 > P.END_CAP_A_GROOVE_OUTER_R + 0.5


def test_trim_bosses_are_clear_of_the_camera_station() -> None:
    """One boss shares the 0-degree angle with the camera. In the first design it was
    only clear because it happened to pass through the lens hole."""
    camera_lo = P.Z_CAMERA - P.CAMERA_MOUNT_PLATE_AXIAL / 2
    camera_hi = P.Z_CAMERA + P.CAMERA_MOUNT_PLATE_AXIAL / 2
    boss_lo = P.Z_TRIM_BOSSES - P.TRIM_BOSS_OD / 2
    boss_hi = P.Z_TRIM_BOSSES + P.TRIM_BOSS_OD / 2
    assert boss_hi < camera_lo or boss_lo > camera_hi


def test_camera_mount_corners_stay_inside_the_bore() -> None:
    """At a 65 mm bore a 26 mm-wide plate's corners reach the wall long before its face
    does, so its radius is solved from the corner. This guards that derivation."""
    corner = math.hypot(P.CAMERA_MOUNT_INNER_RADIUS + P.CAMERA_MOUNT_PLATE_THICKNESS,
                        P.CAMERA_MOUNT_PLATE_TANGENTIAL / 2)
    assert corner <= P.CASING_ID / 2 - P.RUNNING_CLEARANCE + 1e-9


def test_end_cap_rim_is_thinner_than_its_hub() -> None:
    """Cap B's 6 mm hub thickness exists only to give the bearing a 4 mm seat plus a
    shoulder. Mass at the rim is the most expensive mass in the rotating assembly."""
    assert P.END_CAP_RIM_THICKNESS < P.END_CAP_THICKNESS
    assert P.END_CAP_THICKNESS - P.BEARING_WIDTH >= 1.5, "no shoulder left under the bearing"
    assert P.END_CAP_A_THICKNESS - P.END_CAP_A_GROOVE_DEPTH >= 1.5, \
        "no shoulder left under cap A's bearing"


def test_end_cap_is_thicker_than_its_bearing_seat() -> None:
    """A 4 mm cap with a 4 mm seat was bored straight through, leaving no shoulder."""
    assert P.END_CAP_THICKNESS > P.BEARING_WIDTH
    assert P.END_CAP_A_THICKNESS > P.END_CAP_A_GROOVE_DEPTH
    assert P.END_CAP_BOSS_CLEARANCE_BORE > P.AXIS_BOSS_OD, "cap would rub on the boss"


def test_wheel_is_larger_than_the_casing(built) -> None:
    """Otherwise the casing drags on the ground."""
    assert P.WHEEL_OD > P.CASING_OD
    clearance = (P.WHEEL_OD - P.CASING_OD) / 2
    assert clearance >= 5.0, f"only {clearance} mm of ground clearance under the casing"


def test_printed_mass_leaves_room_for_the_rest_of_the_robot(built) -> None:
    """The regression that matters: the first build was 855 g of plastic alone."""
    total = sum(built[name].val().Volume() * build.density_g_mm3(name, P.PRINT_DENSITY) * qty
                for name, qty in build.QUANTITIES.items())
    assert total < 500.0, (
        f"printed plastic is {total:.0f} g; the vehicle target is 700-900 g total, so this "
        f"leaves only {700 - total:.0f} g for motors, battery, electronics and bearings")


def test_rotating_inertia_matches_the_direct_drive_budget(built) -> None:
    """ADR 0011's budget (17.8-25.8 mN m trimmed, against the DM3505's 90 rated) is built
    on this plastic inertia plus the electronics. The slew term scales linearly with I, so a
    drift past 0.20e-3 moves the design point noticeably and should be caught here first."""
    inertia = sum(build.inertia_kg_m2(built[name].val(), P.PRINT_DENSITY) * qty
                  for name, qty in build.ROTATING.items())
    assert 0.10e-3 < inertia < 0.20e-3, f"I = {inertia * 1e3:.3f}e-3 kg m^2"


def test_the_budget_tool_uses_the_current_cad_figures(built) -> None:
    """tools/pitch_inertia_budget.py copies two CAD results because it must not depend on
    CadQuery. A stale copy would make the torque budget silently describe an old machine,
    which is exactly how docs/theory/ ended up describing the belt drive for a day."""
    sys.path.insert(0, str(REPO_ROOT / "tools"))
    import pitch_inertia_budget as budget  # noqa: PLC0415
    sys.path.remove(str(REPO_ROOT / "tools"))

    mass = sum(built[n].val().Volume() * P.PRINT_DENSITY * q
               for n, q in build.ROTATING.items()) * 1e-3
    inertia = sum(build.inertia_kg_m2(built[n].val(), P.PRINT_DENSITY) * q
                  for n, q in build.ROTATING.items())
    shell = build.inertia_kg_m2(built["01_casing_shell"].val(), P.PRINT_DENSITY)
    assert math.isclose(budget.CAD_ROTATING_PLASTIC_KG, mass, rel_tol=0.01)
    assert math.isclose(budget.CAD_ROTATING_PLASTIC_INERTIA_KG_M2, inertia, rel_tol=0.01)
    assert math.isclose(budget.CAD_SHELL_INERTIA_KG_M2, shell, rel_tol=0.01)


def test_the_shell_still_dominates_rotating_inertia(built) -> None:
    """The finding that drives R6. If it stops holding, the mass strategy changes."""
    shell = build.inertia_kg_m2(built["01_casing_shell"].val(), P.PRINT_DENSITY)
    total = sum(build.inertia_kg_m2(built[name].val(), P.PRINT_DENSITY) * qty
                for name, qty in build.ROTATING.items())
    assert shell / total > 0.70, f"shell is only {100 * shell / total:.0f}% of rotating inertia"


def test_imu_sits_within_5_mm_of_the_axis() -> None:
    """R2 as amended by ADR 0010. On-axis is impossible with a continuous chassis; the
    breakout lies beside the spine's waist, chip side facing it, so the offset is the
    stack-up from the axis: waist radius + running gap + chip-side height - half the chip."""
    assert P.IMU_RADIAL_OFFSET < 5.0, f"IMU at r = {P.IMU_RADIAL_OFFSET:.2f} mm"
    assert P.IMU_CHIP_SIDE_HEIGHT >= P.IMU_PACKAGE_THICKNESS, \
        "the chip itself is on the chip side, so nothing there can be shorter than it"


def test_imu_station_is_on_the_waist_and_clear_of_the_camera() -> None:
    """The board only clears the waist, not the full-diameter spine either side, so its
    whole carrier must sit within the waist's axial span."""
    half = P.IMU_BOARD_LENGTH / 2 + P.IMU_CARRIER_MARGIN
    lo, hi = P.Z_IMU_BRIDGE - half, P.Z_IMU_BRIDGE + half
    assert P.Z_WAIST_LO < lo and hi < P.Z_WAIST_HI
    assert lo > P.Z_CAMERA + P.CAMERA_MOUNT_PLATE_AXIAL / 2


def test_end_cap_screw_circle_fits_inside_the_shell(built) -> None:
    assert P.END_CAP_SCREW_RADIUS < P.CASING_ID / 2, "screws would miss the shell flange"
    assert P.END_CAP_SCREW_RADIUS - P.M3_TAP / 2 > P.CASING_ID / 2 - P.SHELL_FLANGE_RADIAL, \
        "screw tap runs off the flange's inner edge"


# ----------------------------------------------------------------- assembly clearances

@pytest.fixture(scope="module")
def assembly():
    import assembly as asm  # noqa: PLC0415
    return asm, asm.build_assembly()


def test_assembly_has_no_interference_at_any_reachable_angle(assembly) -> None:
    """EXHAUSTIVE over pairs, and over relative rotation for parts that move.

    Two earlier failures this guards against: a hand-picked 13 of 55 pairs that passed
    while four real interferences sat in the other 42, and a single-pose sweep that passed
    an IMU bridge driven straight through a chassis standoff.
    """
    asm, parts_map = assembly
    clashes = [f"{r.a} / {r.b} ({r.shared_mm3:.0f} mm3)"
               for r in asm.sweep(parts_map).values() if r.shared_mm3 > 1e-6]
    assert not clashes, f"interference: {clashes}"


def test_rotation_sweep_catches_what_a_single_pose_misses(assembly) -> None:
    """Regression for the first design's hidden collision. A chassis rod off the axis, on
    the opposite side from the IMU bridge's arm, clears at the assembled pose and is hit
    half a turn later. The swept check must see it."""
    asm, parts_map = assembly
    bridge = parts_map["imu_bridge"].val()
    rod = (cq.Workplane("XY", origin=(20.0, 0.0, P.Z_IMU_BRIDGE - 5.0))
           .circle(1.5).extrude(10.0).val())
    assert asm.min_distance_mm(bridge, rod) > 1.0, "test setup: must clear at one pose"

    asm.ROTATION_GROUP["_rod"] = "chassis"
    try:
        result = asm.check_pair("imu_bridge", "_rod", bridge, rod)
    finally:
        del asm.ROTATION_GROUP["_rod"]
    assert result.method == "swept"
    assert result.shared_mm3 > 0.0


def test_every_targeted_pair_is_in_the_assembly(assembly) -> None:
    """Guards against CHECKS naming a part the assembly does not contain."""
    asm, parts_map = assembly
    for a_name, b_name, _target, _note in asm.CHECKS:
        assert a_name in parts_map and b_name in parts_map
    assert set(parts_map) == set(asm.ROTATION_GROUP), "every part needs a rotation group"


def test_targeted_clearances_hold_over_a_full_turn(assembly) -> None:
    asm, parts_map = assembly
    for a_name, b_name, target, note in asm.CHECKS:
        gap = asm.measured_gap(parts_map, a_name, b_name).gap_mm
        assert math.isclose(gap, target, abs_tol=0.05), \
            f"{a_name}/{b_name} ({note}): {gap:.2f} mm, expected {target:.2f}"


def test_every_bearing_race_is_supported(assembly) -> None:
    """A minimum-distance check cannot see this: a boss can be concentric with its cap and
    still miss it axially, which both of the first two assembly attempts did."""
    asm, parts_map = assembly
    for description, fraction in asm.bearing_checks(parts_map):
        assert fraction > 0.99, f"{description}: {100 * fraction:.0f}% supported"


def test_casing_shell_is_hollow_inside_the_trim_bosses(built) -> None:
    """The trim-mass bosses were once radial RODS from the axis, not pads on the wall.

    A YZ workplane extrudes along +X from wherever its origin sits. Starting at x=0 and
    extruding by the inner radius produced six solid rods spanning the full bore, with the
    tap drilled clean through the impact surface.
    """
    shell = built["01_casing_shell"].val()
    probe_r = P.CASING_ID / 2 - P.TRIM_BOSS_HEIGHT - 0.5
    probe = (cq.Workplane("XY", origin=(0, 0, -10))
             .circle(probe_r).extrude(P.CASING_LENGTH + 20).val())
    shared = cq.Workplane(obj=shell).intersect(cq.Workplane(obj=probe)).val().Volume()
    assert shared < 1.0, f"{shared:.0f} mm3 of shell material inside r={probe_r} mm"


def test_trim_bosses_do_not_pierce_the_impact_surface(built) -> None:
    """Their tapped holes must stay blind: the casing's outer face takes the hits."""
    shell = built["01_casing_shell"].val()
    assert P.TRIM_BOSS_HEIGHT - 1.0 < P.TRIM_BOSS_HEIGHT, "tap must be shallower than the boss"
    bb = shell.BoundingBox()
    assert math.isclose(bb.xlen, P.CASING_OD, abs_tol=0.01), \
        "a boss projecting outward would grow the bounding box"


def test_wheels_clear_the_rotating_casing(assembly) -> None:
    asm, parts_map = assembly
    for wheel in ("wheel_a", "wheel_b", "tire_a", "tire_b"):
        gap = asm.min_distance_mm(parts_map["casing_shell"].val(), parts_map[wheel].val())
        assert gap > 1.0, f"{wheel} is {gap:.2f} mm from the casing"


# ------------------------------------------------------------------- tire (ADR 0017)

def test_tire_sets_the_wheel_diameter(built) -> None:
    """The tire, not the hub, touches the ground, so ADR 0008's 105 mm is the tire's OD."""
    bb = built["08_tire"].val().BoundingBox()
    assert math.isclose(bb.xlen, P.WHEEL_OD, abs_tol=0.01)
    assert math.isclose(bb.zlen, P.WHEEL_WIDTH, abs_tol=0.01)


def test_hub_sits_inside_the_tire_with_no_ground_contact(built) -> None:
    """Only TPU reaches the ground; the hub's outer surface is a tire thickness inboard."""
    bb = built["07_wheel"].val().BoundingBox()
    assert bb.xlen < P.WHEEL_OD - 2 * (P.TIRE_THICKNESS - P.TIRE_RIDGE_HEIGHT) + 0.01


def test_tire_keeps_material_under_tread_and_ridge() -> None:
    """Thinnest TPU is where a tread groove sits over the hub's retaining ridge."""
    thinnest = P.TIRE_THICKNESS - P.TREAD_DEPTH - P.TIRE_RIDGE_HEIGHT
    assert thinnest >= 1.5, f"only {thinnest} mm of TPU left over the ridge"


def test_tire_is_a_stretch_fit_on_the_hub() -> None:
    """Printed undersize so it grips the rim; the ridge stops it walking off sideways."""
    assert 0.0 < P.TIRE_FIT_INTERFERENCE < 1.5
    assert P.TIRE_RIDGE_WIDTH < P.WHEEL_WIDTH / 2
