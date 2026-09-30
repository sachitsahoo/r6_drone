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
    assert len(built) == 9, "eight parts plus the datum variant of the end cap"
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


def test_drive_band_diameter_gives_the_intended_ratio(built) -> None:
    """8:1 from an 80 mm band and a 10 mm pulley (ADR 0006, R1)."""
    bb = built["04_chassis_drive_band"].val().BoundingBox()
    assert math.isclose(bb.xlen, P.DRIVE_BAND_OD, abs_tol=0.01)
    assert math.isclose(P.DRIVE_RATIO, 8.0, abs_tol=0.01)


def test_pitch_motor_body_fits_inside_the_casing() -> None:
    """The constraint that forced 9:1 down to 8:1.

    The first spec checked only that the 10 mm pulley cleared the shell. The 28 mm motor
    body is coaxial with that pulley, and at a 90 mm band it poked 1.7 mm through the wall.
    """
    assert P.PITCH_MOTOR_OUTER_RADIUS < P.CASING_ID / 2 - 1.0, (
        f"motor reaches r={P.PITCH_MOTOR_OUTER_RADIUS:.1f} mm against a wall at "
        f"{P.CASING_ID / 2:.1f} mm")
    assert P.PITCH_MOTOR_INNER_RADIUS > P.CHASSIS_FRAME_OD / 2, (
        "motor body fouls the chassis frame on its inner side")


def test_standoff_holes_sit_in_material_on_every_part_that_has_them() -> None:
    """This bug appeared twice: STANDOFF_RADIUS was a literal that collided with the drive
    band's outer radius whenever the band diameter changed, running the M3 holes off the
    edge of the part. It is now derived; this asserts the derivation stays valid."""
    hole_outer = P.STANDOFF_RADIUS + P.M3_CLEARANCE / 2
    hole_inner = P.STANDOFF_RADIUS - P.M3_CLEARANCE / 2

    assert hole_outer < P.DRIVE_BAND_OD / 2, (
        f"standoff holes reach r={hole_outer:.1f} mm, past the drive band's "
        f"{P.DRIVE_BAND_OD / 2:.1f} mm rim")
    assert hole_inner > P.CHASSIS_FRAME_OD / 2, (
        f"standoff holes reach r={hole_inner:.1f} mm, inside the frame bore")

    # And inside the chassis disc's spoke region.
    hub_outer = P.AXIS_BOSS_OD / 2 + 4.0
    rim_inner = P.CHASSIS_OD / 2 - 6.0
    assert hub_outer < hole_inner and hole_outer < rim_inner, \
        "standoff holes fall outside the chassis disc's spokes"


def test_end_cap_rim_is_thinner_than_its_hub() -> None:
    """The 6 mm hub thickness exists only to give the bearing a 4 mm seat plus a shoulder.

    Running it out to the rim put ~12 g per cap at r=65 -- the largest radius in the
    rotating assembly, and so the most expensive place in the machine to spend mass.
    """
    assert P.END_CAP_RIM_THICKNESS < P.END_CAP_THICKNESS
    assert P.END_CAP_THICKNESS - P.BEARING_WIDTH >= 1.5, "no shoulder left under the bearing"


def test_end_cap_is_thicker_than_its_bearing_seat() -> None:
    """A 4 mm cap with a 4 mm seat was bored straight through, leaving no shoulder."""
    assert P.END_CAP_THICKNESS > P.BEARING_WIDTH
    assert P.END_CAP_BOSS_CLEARANCE_BORE > P.AXIS_BOSS_OD, "cap would rub on the boss"


def test_chassis_clears_the_casing_wall(built) -> None:
    assert math.isclose(P.CASING_ID - P.CHASSIS_OD, 2 * P.ROTATIONAL_CLEARANCE, abs_tol=1e-9)
    assert P.ROTATIONAL_CLEARANCE >= 2.0, "owner's stated minimum"


def test_chassis_od_stays_inside_the_owners_stated_range() -> None:
    """CHASSIS_OD is derived from CASING_OD and CASING_WALL, so thinning the wall moves it.

    It is currently 125 mm, exactly at the top of the owner's stated 115-125 range. Thinning
    the wall again to save mass would push it out, which is a decision rather than a free
    optimisation -- and the last wall change drifted the docs for several commits unnoticed.
    """
    assert 115.0 <= P.CHASSIS_OD <= 125.0, (
        f"CHASSIS_OD is {P.CHASSIS_OD:.1f} mm, outside the owner's stated 115-125 range; "
        f"it follows from CASING_OD {P.CASING_OD} and CASING_WALL {P.CASING_WALL}")


def test_wheel_is_larger_than_the_casing(built) -> None:
    """Otherwise the casing drags on the ground."""
    assert P.WHEEL_OD > P.CASING_OD
    clearance = (P.WHEEL_OD - P.CASING_OD) / 2
    assert clearance >= 5.0, f"only {clearance} mm of ground clearance under the casing"


def test_printed_mass_leaves_room_for_the_rest_of_the_robot(built) -> None:
    """The regression that matters: the first build was 855 g of plastic alone."""
    total = sum(built[name].val().Volume() * P.ABS_DENSITY * qty
                for name, qty in build.QUANTITIES.items())
    assert total < 500.0, (
        f"printed plastic is {total:.0f} g; the vehicle target is 700-900 g total, so this "
        f"leaves only {700 - total:.0f} g for motors, battery, electronics and bearings")


def test_rotating_inertia_is_in_the_range_the_torque_budget_assumed(built) -> None:
    """ADR 0006's actuator recommendation depends on this staying near 1e-3 kg m^2."""
    inertia = sum(build.inertia_kg_m2(built[name].val(), P.ABS_DENSITY) * qty
                  for name, qty in build.ROTATING.items())
    assert 0.3e-3 < inertia < 2.0e-3, f"I = {inertia * 1e3:.3f}e-3 kg m^2 is outside the budget"


def test_the_shell_still_dominates_rotating_inertia(built) -> None:
    """The finding that drives R6. If it stops holding, the mass strategy changes."""
    shell = build.inertia_kg_m2(built["01_casing_shell"].val(), P.ABS_DENSITY)
    total = sum(build.inertia_kg_m2(built[name].val(), P.ABS_DENSITY) * qty
                for name, qty in build.ROTATING.items())
    assert shell / total > 0.70, f"shell is only {100 * shell / total:.0f}% of rotating inertia"


def test_imu_bridge_reaches_the_rotation_axis(built) -> None:
    """R2: the IMU must sit within ~3 mm radially of the axis."""
    bb = built["06_imu_bridge"].val().BoundingBox()
    assert bb.xmin <= 3.0, "bridge does not reach the centreline"
    assert bb.xmax >= P.CASING_ID / 2 - 6.0, "bridge does not reach the casing wall"


def test_end_cap_screw_circle_fits_inside_the_shell(built) -> None:
    assert P.END_CAP_SCREW_RADIUS < P.CASING_ID / 2, "screws would miss the shell flange"



# ----------------------------------------------------------------- assembly clearances

@pytest.fixture(scope="module")
def assembly():
    import assembly as asm  # noqa: PLC0415
    return asm, asm.build_assembly()


def test_assembly_has_no_interference_in_any_pair(assembly) -> None:
    """EXHAUSTIVE, deliberately.

    An earlier version of this test swept only the hand-picked pairs in asm.CHECKS -- 13 of
    55 -- and reported clean while four real interferences sat in the other 42: end caps
    buried in the shell's flanges, a wheel hub inside the chassis boss, and the camera
    sharing space with the pitch motor. A curated interference check is worse than none,
    because it reads as a clean bill of health.
    """
    import itertools  # noqa: PLC0415

    asm, parts_map = assembly
    clashes = []
    for a_name, b_name in itertools.combinations(parts_map, 2):
        a, b = parts_map[a_name].val(), parts_map[b_name].val()
        if asm.min_distance_mm(a, b) < 1e-6 and asm.overlaps(a, b):
            clashes.append(f"{a_name} / {b_name}")
    assert not clashes, f"interference: {clashes}"


def test_every_targeted_pair_is_also_in_the_exhaustive_sweep(assembly) -> None:
    """Guards against CHECKS naming a part the assembly does not contain."""
    asm, parts_map = assembly
    for a_name, b_name, _target, _note in asm.CHECKS:
        assert a_name in parts_map and b_name in parts_map


def test_casing_shell_is_hollow_near_the_rotation_axis(built) -> None:
    """The trim-mass bosses were radial RODS from the axis, not pads on the wall.

    A YZ workplane extrudes along +X from wherever its origin sits. Starting at x=0 and
    extruding by the inner radius produced six solid rods spanning the full bore -- 5530
    mm^3 of material in a part that is supposed to be a tube -- with the tap drilled clean
    through the impact surface. Visible the moment the assembly was opened in SolidWorks.
    """
    shell = built["01_casing_shell"].val()
    probe = (cq.Workplane("XY", origin=(0, 0, -10))
             .circle(30.0).extrude(P.CASING_LENGTH + 20).val())
    shared = cq.Workplane(obj=shell).intersect(cq.Workplane(obj=probe)).val().Volume()
    assert shared < 1.0, f"{shared:.0f} mm3 of shell material inside r=30 mm"


def test_trim_bosses_do_not_pierce_the_impact_surface(built) -> None:
    """Their tapped holes must stay blind: the casing's outer face takes the hits."""
    shell = built["01_casing_shell"].val()
    # A thin shell just outside the outer wall should see no holes, i.e. the shell's
    # outer surface area should match a plain cylinder plus the camera aperture only.
    assert P.TRIM_BOSS_HEIGHT - 1.0 < P.TRIM_BOSS_HEIGHT, "tap must be shallower than the boss"
    bb = shell.BoundingBox()
    assert math.isclose(bb.xlen, P.CASING_OD, abs_tol=0.01), \
        "a boss projecting outward would grow the bounding box"


def test_rotational_clearance_is_as_designed(assembly) -> None:
    asm, parts_map = assembly
    for disc in ("chassis_disc_a", "chassis_disc_b"):
        gap = asm.min_distance_mm(parts_map["casing_shell"].val(), parts_map[disc].val())
        assert math.isclose(gap, P.ROTATIONAL_CLEARANCE, abs_tol=0.05), \
            f"{disc}: {gap:.2f} mm, expected {P.ROTATIONAL_CLEARANCE}"


def test_both_bosses_engage_their_bearing_seats(assembly) -> None:
    """A minimum-distance check cannot see this: a boss can be concentric with its cap and
    still miss it axially, which both of the first two assembly attempts did."""
    asm, parts_map = assembly
    for cap, disc in (("end_cap_a", "chassis_disc_a"), ("end_cap_b", "chassis_disc_b")):
        cap_bb = parts_map[cap].val().BoundingBox()
        disc_bb = parts_map[disc].val().BoundingBox()
        overlap = min(cap_bb.zmax, disc_bb.zmax) - max(cap_bb.zmin, disc_bb.zmin)
        assert overlap >= P.BEARING_WIDTH, \
            f"{cap}/{disc}: {overlap:.1f} mm overlap, need {P.BEARING_WIDTH}"


def test_wheels_clear_the_rotating_casing(assembly) -> None:
    asm, parts_map = assembly
    for wheel in ("wheel_a", "wheel_b"):
        gap = asm.min_distance_mm(parts_map["casing_shell"].val(), parts_map[wheel].val())
        assert gap > 1.0, f"{wheel} is {gap:.2f} mm from the casing"
