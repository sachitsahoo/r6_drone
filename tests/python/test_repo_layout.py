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
