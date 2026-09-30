"""Tests for the firmware/core purity guard.

The guard is CI's only enforcement of CLAUDE.md hard rules 1 and 2, so a guard
that silently stopped catching violations would be worse than no guard at all --
it would read as green. These tests prove it still catches each violation class,
and that it does not fire on prose.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools"))

import check_core_purity as guard  # noqa: E402


def write_core_file(root: Path, name: str, contents: str) -> Path:
    core_dir = root / "firmware" / "core"
    core_dir.mkdir(parents=True, exist_ok=True)
    path = core_dir / name
    path.write_text(contents)
    return path


def test_clean_file_has_no_violations(tmp_path: Path) -> None:
    write_core_file(
        tmp_path,
        "clean.hpp",
        "#pragma once\n#include <cstdint>\nnamespace recon::core { uint32_t f(); }\n",
    )
    assert guard.check_tree(tmp_path / "firmware" / "core") == []


@pytest.mark.parametrize(
    ("snippet", "expected_reason_fragment"),
    [
        ('#include "stm32g4xx_hal.h"', "STM32 vendor header"),
        ("#include <stm32g474xx.h>", "STM32 vendor header"),
        ('#include "core_cm4.h"', "CMSIS core header"),
        ('#include "main.h"', "CubeMX-generated main.h"),
        ('#include "FreeRTOS.h"', "FreeRTOS header"),
    ],
)
def test_vendor_includes_are_rejected(
    tmp_path: Path, snippet: str, expected_reason_fragment: str
) -> None:
    write_core_file(tmp_path, "leaky.hpp", f"#pragma once\n{snippet}\n")
    violations = guard.check_tree(tmp_path / "firmware" / "core")
    assert len(violations) == 1
    assert expected_reason_fragment in violations[0]
    assert "leaky.hpp:2" in violations[0]


@pytest.mark.parametrize(
    ("snippet", "expected_reason_fragment"),
    [
        ("auto* p = new Widget();", "operator new"),
        ("void* p = malloc(16);", "C heap allocation"),
        ("std::vector<int> v;", "std::vector"),
        ("std::string name;", "std::string"),
        ("std::function<void()> cb;", "std::function"),
        ("throw MyError();", "throw"),
        ("auto* d = dynamic_cast<Derived*>(base);", "dynamic_cast"),
    ],
)
def test_forbidden_constructs_are_rejected(
    tmp_path: Path, snippet: str, expected_reason_fragment: str
) -> None:
    write_core_file(tmp_path, "allocating.cpp", f"void f() {{\n  {snippet}\n}}\n")
    violations = guard.check_tree(tmp_path / "firmware" / "core")
    assert len(violations) == 1
    assert expected_reason_fragment in violations[0]


def test_prose_in_comments_is_not_a_violation(tmp_path: Path) -> None:
    """Doc comments must be able to explain the very rules being enforced."""
    write_core_file(
        tmp_path,
        "documented.hpp",
        "#pragma once\n"
        "// This module deliberately avoids std::string and std::vector.\n"
        "/* It never calls malloc, never uses new, and cannot throw,\n"
        "   because it is built with -fno-exceptions. */\n"
        "#include <cstdint>\n",
    )
    assert guard.check_tree(tmp_path / "firmware" / "core") == []


def test_reported_line_numbers_survive_comment_stripping(tmp_path: Path) -> None:
    """Comments are blanked, not deleted, so line numbers stay usable."""
    write_core_file(
        tmp_path,
        "offset.cpp",
        "/* a\n   multi-line\n   comment */\nvoid f() { std::string s; }\n",
    )
    violations = guard.check_tree(tmp_path / "firmware" / "core")
    assert len(violations) == 1
    assert "offset.cpp:4" in violations[0]


def test_real_repository_core_is_clean() -> None:
    """The actual firmware/core/ in this repo must pass. This is the CI gate."""
    assert guard.check_tree(REPO_ROOT / "firmware" / "core") == []
