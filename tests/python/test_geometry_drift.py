"""firmware/core/control/geometry.hpp copies two numbers from cad/parameters.py.

A copied figure goes stale silently when the design changes (tasks/lessons.md, 2026-09-30),
so this fails if either side moves without the other. cad/parameters.py imports only
`math`, so this runs in CI without CadQuery.
"""

from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
GEOMETRY_HPP = REPO_ROOT / "firmware" / "core" / "control" / "geometry.hpp"


def _cad_parameters():
    spec = importlib.util.spec_from_file_location("cad_parameters", REPO_ROOT / "cad" / "parameters.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _cpp_constant(name: str) -> float:
    match = re.search(rf"inline constexpr float {name} = ([0-9.]+)F;", GEOMETRY_HPP.read_text())
    assert match, f"{name} not found in {GEOMETRY_HPP}"
    return float(match.group(1))


def test_wheel_radius_matches_the_cad() -> None:
    p = _cad_parameters()
    assert _cpp_constant("kWheelRadius_m") == pytest.approx(p.WHEEL_OD / 2 / 1000)


def test_track_width_matches_the_cad() -> None:
    p = _cad_parameters()
    centre_mm = p.CASING_LENGTH / 2 + p.WHEEL_STANDOFF + p.WHEEL_WIDTH / 2
    assert _cpp_constant("kTrackWidth_m") == pytest.approx(2 * centre_mm / 1000)
