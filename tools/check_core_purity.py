#!/usr/bin/env python3
"""Enforce the portability firewall and allocation ban on firmware/core/ and firmware/hal/.

CLAUDE.md states two hard rules that nothing in the compiler checks:

  1. firmware/core/ never includes vendor or STM32 headers. It must build and
     pass tests on a PC.
  2. No dynamic allocation after init: no new, malloc, std::vector growth, or
     std::string in firmware.

Both are conventions until something enforces them, and both are cheap to
violate by accident and expensive to unwind later -- vendor headers spread, and
a single std::string in a control path reintroduces the heap. CI runs this on
every push.

firmware/hal/ is held to the same rules: core includes its headers, so a vendor
header there would reach core one include away.

Usage:
    python3 tools/check_core_purity.py [ROOT]

Exits 0 when clean, 1 when a violation is found (with file:line for each).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

# Rule 1: headers that mean platform code has leaked into portable code.
FORBIDDEN_INCLUDE_PATTERNS: list[tuple[str, str]] = [
    (r'#\s*include\s*[<"]stm32', "STM32 vendor header"),
    (r'#\s*include\s*[<"]core_cm[0-9]', "CMSIS core header"),
    (r'#\s*include\s*[<"]cmsis', "CMSIS header"),
    (r'#\s*include\s*[<"]main\.h[>"]', "CubeMX-generated main.h"),
    (r'#\s*include\s*[<"]FreeRTOS', "FreeRTOS header"),
]

# Rule 2: constructs that allocate, throw, or otherwise break the firmware subset.
FORBIDDEN_CONSTRUCT_PATTERNS: list[tuple[str, str]] = [
    (r'\bnew\s+[A-Za-z_:]', "operator new (no dynamic allocation after init)"),
    (r'\b(?:std::)?(?:malloc|calloc|realloc)\s*\(', "C heap allocation"),
    (r'\bstd::vector\b', "std::vector (grows on the heap)"),
    (r'\bstd::string\b', "std::string (allocates)"),
    (r'\bstd::function\b', "std::function (may allocate)"),
    (r'\bthrow\b', "throw (built with -fno-exceptions)"),
    (r'\bdynamic_cast\b', "dynamic_cast (built with -fno-rtti)"),
]

SOURCE_SUFFIXES = {".hpp", ".cpp", ".h", ".cc", ".cxx"}


def strip_comments(text: str) -> str:
    """Blank out comment and string-literal content, preserving line numbering.

    Prevents false positives from prose -- a README-style comment mentioning
    std::string, or a doc comment explaining why throw is banned, is not a
    violation. Replaces content with spaces rather than deleting it so reported
    line numbers still match the original file.

    Limitation: this is a regex approximation, not a C++ lexer. Raw string
    literals and multi-line string continuations are not handled. That is
    acceptable for a CI guard whose failure mode is a false positive a human
    reads, not silently wrong firmware.
    """

    def blank(match: re.Match[str]) -> str:
        return re.sub(r"[^\n]", " ", match.group(0))

    # Order matters: block comments first, then line comments, then literals.
    text = re.sub(r"/\*.*?\*/", blank, text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", blank, text)

    # Blank string literals -- but NOT on preprocessor lines. An include such as
    #     #include "stm32g4xx_hal.h"
    # carries its header name inside quotes, so blanking literals there would
    # erase the exact text this guard exists to catch.
    processed: list[str] = []
    for line in text.split("\n"):
        if re.match(r"\s*#", line):
            processed.append(line)
        else:
            processed.append(re.sub(r'"(?:\\.|[^"\\\n])*"', blank, line))
    return "\n".join(processed)


def check_file(path: Path) -> list[str]:
    """Return a list of human-readable violation messages for one file."""
    violations: list[str] = []
    source = strip_comments(path.read_text(encoding="utf-8", errors="replace"))

    for line_number, line in enumerate(source.splitlines(), start=1):
        for pattern, reason in FORBIDDEN_INCLUDE_PATTERNS + FORBIDDEN_CONSTRUCT_PATTERNS:
            if re.search(pattern, line):
                violations.append(f"{path}:{line_number}: {reason}\n    {line.strip()}")
    return violations


def check_tree(core_dir: Path) -> list[str]:
    """Return every violation under `core_dir`, sorted by path."""
    violations: list[str] = []
    for path in sorted(core_dir.rglob("*")):
        if path.is_file() and path.suffix in SOURCE_SUFFIXES:
            violations.extend(check_file(path))
    return violations


#: Directories held to the firewall, relative to the repository root. core is required; hal
#: is checked whenever it exists.
CHECKED_DIRS = (("firmware", "core"), ("firmware", "hal"))


def main(argv: list[str]) -> int:
    root = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parents[1]
    core_dir = root / "firmware" / "core"

    if not core_dir.is_dir():
        print(f"error: {core_dir} does not exist", file=sys.stderr)
        return 1

    violations: list[str] = []
    for parts in CHECKED_DIRS:
        directory = root.joinpath(*parts)
        if directory.is_dir():
            violations.extend(check_tree(directory))

    if violations:
        print(f"firmware purity check FAILED: {len(violations)} violation(s)\n")
        for violation in violations:
            print(violation)
        print(
            "\nfirmware/core and firmware/hal must stay vendor-free and allocation-free"
            "\n(CLAUDE.md, 'Hard rules for firmware' 1 and 2). Platform code belongs in"
            "\nfirmware/stm32/ behind a firmware/hal/ interface."
        )
        return 1

    print("firmware/core and firmware/hal purity check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
