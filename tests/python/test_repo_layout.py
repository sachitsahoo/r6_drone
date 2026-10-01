"""Phase 1 placeholder: proves the pytest harness runs in CI.

Replaced by real protocol codec and bridge tests once protocol/ is designed.
Kept meaningful rather than `assert True` so a green run means something: it
checks the documentation invariants CLAUDE.md requires of every module.
"""

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]

MODULE_DIRS = [
    "firmware/core",
    "firmware/hal",
    "firmware/stm32",
    "sim",
    "protocol",
    "robot_bridge",
    "operator",
    "tools",
    "tests",
    "docs",
]


@pytest.mark.parametrize("module_dir", MODULE_DIRS)
def test_every_module_directory_exists(module_dir: str) -> None:
    assert (REPO_ROOT / module_dir).is_dir(), f"{module_dir} is missing"


@pytest.mark.parametrize("module_dir", MODULE_DIRS)
def test_every_module_directory_has_a_readme(module_dir: str) -> None:
    """CLAUDE.md: every module directory has a README.md. Stubs are fine; empty is not."""
    readme = REPO_ROOT / module_dir / "README.md"
    assert readme.is_file(), f"{module_dir}/README.md is missing"
    assert len(readme.read_text().strip()) > 200, f"{module_dir}/README.md is too thin to be useful"


# --------------------------------------------------- analysis tooling sanity checks

def test_pitch_inertia_budget_physics_is_self_consistent() -> None:
    """Sanity-checks tools/pitch_inertia_budget.py against closed-form results.

    ADR 0011's motor choice and its balance requirement rest on these numbers, so the
    script is worth a few assertions rather than trusting that it was right once.
    """
    import math
    import sys

    sys.path.insert(0, str(REPO_ROOT / "tools"))
    import pitch_inertia_budget as budget  # noqa: PLC0415

    # A point mass at radius r has I = m r^2, and doubling r quadruples it.
    near = budget.PointMass("near", 0.1, 0.033)
    far = budget.PointMass("far", 0.1, 0.066)
    assert math.isclose(far.inertia_kg_m2 / near.inertia_kg_m2, 4.0, rel_tol=1e-9), \
        "inertia must scale with the square of radius"

    # Gravity holding torque is zero exactly when the casing is balanced.
    assert budget.gravity_torque_N_m(0.37, 0.0) == 0.0
    assert budget.gravity_torque_N_m(0.37, 0.010) > 0.0

    # Slew torque: halving the time quadruples the required torque.
    slow = budget.slew_torque_N_m(1e-3, math.radians(10), 0.100)
    fast = budget.slew_torque_N_m(1e-3, math.radians(10), 0.050)
    assert math.isclose(fast / slow, 4.0, rel_tol=1e-9)

    # A balanced casing has no pendulum resonance; an offset one does, below 2 Hz for this
    # geometry, which is the finding that makes balance a control-design decision.
    assert budget.pendulum_frequency_Hz(0.37, 0.0, 1.5e-3) == 0.0
    f = budget.pendulum_frequency_Hz(0.37, 0.010, 1.534e-3)
    assert 0.5 < f < 1.5, f"expected a sub-2 Hz resonance, got {f} Hz"

    # Backlash to pixels is linear, and 1 deg is tens of pixels at 1080p -- the figure that
    # disqualifies a typical gearbox.
    assert math.isclose(budget.backlash_pixels(1.0, 90.0, 1920), 21.33, rel_tol=1e-3)
    assert math.isclose(budget.backlash_pixels(2.0, 90.0, 1920),
                        2 * budget.backlash_pixels(1.0, 90.0, 1920), rel_tol=1e-9)

    # The headline finding of the current budget (ADR 0011): an untrimmed 10 mm sideways
    # CoM offset puts the GM2804 at or past its rated torque, and trimming to 2 mm brings
    # it back under. If a change breaks either half, the balance requirement needs revisiting.
    model = budget.build_model()
    untrimmed = budget.budget(model, 0.010, 0.0).total_N_m
    trimmed = budget.budget(model, budget.TRIMMED_HORIZONTAL_OFFSET_M,
                            budget.BOTTOM_HEAVY_VERTICAL_OFFSET_M).total_N_m
    assert untrimmed > 0.95 * budget.MOTOR_RATED_TORQUE_N_M
    assert trimmed < 0.75 * budget.MOTOR_RATED_TORQUE_N_M

    # The shell is still the largest single contributor once electronics are included.
    shell_fraction = model.shell_inertia_kg_m2 / model.total_inertia_kg_m2
    assert shell_fraction > 0.65, f"shell is only {shell_fraction:.0%} of rotating inertia"

    # Copper loss is quadratic in current: double the current, four times the heat.
    assert math.isclose(budget.copper_loss_W(0.4) / budget.copper_loss_W(0.2), 4.0)

    sys.path.remove(str(REPO_ROOT / "tools"))
