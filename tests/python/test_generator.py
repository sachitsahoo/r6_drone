"""Tests for the protocol code generator.

Generated output is gitignored, so there is no committed golden file to diff against.
What matters instead is determinism: the same schema must always produce byte-identical
output, or a rebuild becomes a source of mysterious diffs and the --check mode used by CI
would be useless.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import generate  # noqa: E402
from schema_loader import default_schema_paths, load_schema  # noqa: E402


@pytest.fixture(scope="module")
def schema():
    return load_schema(*default_schema_paths())


def test_generation_is_deterministic(schema) -> None:
    first = generate.render(schema)
    second = generate.render(schema)
    assert first == second


def test_repeated_loads_produce_identical_output() -> None:
    """Catches any dependence on dict ordering or object identity across loads."""
    a = generate.render(load_schema(*default_schema_paths()))
    b = generate.render(load_schema(*default_schema_paths()))
    assert a == b


def test_generates_exactly_the_expected_files(schema) -> None:
    assert set(generate.render(schema)) == {"messages.hpp", "messages.py"}


@pytest.mark.parametrize("filename", ["messages.hpp", "messages.py"])
def test_output_carries_a_do_not_edit_banner(schema, filename: str) -> None:
    text = generate.render(schema)[filename]
    head = text[:600]
    assert "AUTO-GENERATED" in head
    assert "DO NOT EDIT" in head


def test_hpp_has_an_include_guard_and_no_vendor_headers(schema) -> None:
    text = generate.render(schema)["messages.hpp"]
    assert "#pragma once" in text
    # This header is included from firmware/core/, so it is bound by hard rule 1.
    for forbidden in ("stm32", "core_cm", "cmsis", "FreeRTOS"):
        assert forbidden not in text.lower()


def test_hpp_avoids_constructs_banned_in_firmware(schema) -> None:
    """Hard rule 2 applies to generated code too -- tools/check_core_purity.py only
    scans firmware/core/, and this file lives in protocol/generated/."""
    text = generate.render(schema)["messages.hpp"]
    for forbidden in ("std::vector", "std::string", "std::function", "malloc", "throw ",
                      "dynamic_cast", "new "):
        assert forbidden not in text, f"generated C++ must not use {forbidden!r}"


def test_no_f64_reaches_the_generated_code(schema) -> None:
    """The Cortex-M4 FPU is single precision; a double would be software-emulated."""
    text = generate.render(schema)["messages.hpp"]
    assert "double" not in text


def test_check_mode_passes_against_freshly_generated_files(tmp_path: Path) -> None:
    out = tmp_path / "generated"
    script = REPO_ROOT / "protocol" / "generate.py"
    assert subprocess.run([sys.executable, str(script), "--out-dir", str(out)],
                          capture_output=True, check=False).returncode == 0
    assert subprocess.run([sys.executable, str(script), "--out-dir", str(out), "--check"],
                          capture_output=True, check=False).returncode == 0


def test_check_mode_fails_when_output_is_stale(tmp_path: Path) -> None:
    out = tmp_path / "generated"
    script = REPO_ROOT / "protocol" / "generate.py"
    subprocess.run([sys.executable, str(script), "--out-dir", str(out)],
                   capture_output=True, check=True)
    (out / "messages.hpp").write_text("// someone edited the generated file\n")
    result = subprocess.run([sys.executable, str(script), "--out-dir", str(out), "--check"],
                            capture_output=True, text=True, check=False)
    assert result.returncode != 0
    assert "stale" in result.stderr


def test_check_mode_fails_when_output_is_missing(tmp_path: Path) -> None:
    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "protocol" / "generate.py"),
         "--out-dir", str(tmp_path / "nothing-here"), "--check"],
        capture_output=True, text=True, check=False)
    assert result.returncode != 0


def test_generated_python_is_importable_and_matches_the_schema(schema, tmp_path: Path) -> None:
    out = tmp_path / "generated"
    out.mkdir()
    (out / "messages.py").write_text(generate.render(schema)["messages.py"])
    sys.path.insert(0, str(out))
    try:
        for name in list(sys.modules):
            if name == "messages":
                del sys.modules[name]
        import messages as fresh  # noqa: PLC0415
        assert fresh.PROTOCOL_VERSION == schema.protocol_version
        assert len(fresh.PAYLOAD_BYTES_BY_ID) == len(schema.messages)
        for message in schema.messages:
            assert fresh.PAYLOAD_BYTES_BY_ID[message.id] == \
                message.payload_bytes(schema.enums)
    finally:
        sys.path.remove(str(out))
        sys.modules.pop("messages", None)
