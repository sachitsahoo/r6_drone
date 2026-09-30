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
    """ADR 0006 and R1 depend on 9:1 from a 90 mm band and a 10 mm pulley."""
    bb = built["04_chassis_drive_band"].val().BoundingBox()
    assert math.isclose(bb.xlen, P.DRIVE_BAND_OD, abs_tol=0.01)
    ratio = P.DRIVE_BAND_OD / P.DRIVE_PULLEY_OD
    assert math.isclose(ratio, 9.0, abs_tol=0.01)


def test_chassis_clears_the_casing_wall(built) -> None:
    assert math.isclose(P.CASING_ID - P.CHASSIS_OD, 2 * P.ROTATIONAL_CLEARANCE, abs_tol=1e-9)
    assert P.ROTATIONAL_CLEARANCE >= 2.0, "owner's stated minimum"


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
