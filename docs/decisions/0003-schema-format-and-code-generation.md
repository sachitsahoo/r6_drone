# 0003 — Schema format and code generation

- **Status:** Accepted (2026-09-30)
- **Date:** 2026-09-30
- **Related:** [0002](0002-protocol-framing-and-codec.md), [`protocol/design-proposal.md`](../../protocol/design-proposal.md)

## Context

CLAUDE.md requires that the schema in `protocol/` be the single source of truth, with C++ and
Python code generated from it and never hand-edited. That leaves two independent choices: the
format of the file a human edits, and the language of the program that reads it.

These are genuinely separate, and conflating them caused confusion once already. The
generator runs at build time on a development machine or in CI. It is never deployed to the
robot, never runs on the MCU, and contributes no code size, no allocation, and no runtime
dependency. Its cost is toolchain surface, nothing else.

## Options considered

### The schema format

1. **YAML (chosen).** Readable diffs, comments, no build step to read it, and a standard
   parser in Python. Cost: YAML's type coercion is notoriously loose (`1.0`, `yes`, `0x10`),
   so the generator must validate aggressively rather than trusting the parse.
2. **TOML.** Stricter types and a stdlib parser in Python 3.11+ (`tomllib`), which would
   remove the PyYAML dependency. Rejected narrowly: nested lists of tables for messages and
   fields are noticeably uglier to edit than YAML's, and this file gets edited often.
3. **A custom DSL.** Best possible ergonomics, and a parser to write, document, debug, and
   explain. Rejected: the schema is not where this project should spend novelty.
4. **An existing IDL** — Protocol Buffers, nanopb, Cap'n Proto, FlatBuffers. These bring
   mature generators and wire formats. Rejected for three reasons: the framing is already
   specified by hand in CLAUDE.md, so their wire encoding would be redundant or fought;
   varint encoding produces variable-size messages, which conflicts with the fixed-size
   decision in ADR 0002 and with the no-allocation rule; and the owner must be able to defend
   the format in interviews and a research writeup. A readable generator whose output can be
   inspected end to end is defensible in a way that a third-party wire format is not.

### The generator's language

5. **Python (chosen).** Already a hard project dependency — pytest, `operator/`,
   `robot_bridge/`, `tools/`. Adds no new toolchain.
6. **C++.** Would mean CMake must build a host-side generator binary *before* it can build
   the firmware that consumes the generator's output, including during a cross-compile.
   That bootstrap ordering is solvable and unpleasant, and buys nothing.

### Templating

7. **Plain Python string building (chosen).** Two output files with regular structure. No
   dependency, and the generator stays readable by someone who does not know a template language.
8. **Jinja2.** Nicer for large templates, and a dependency plus a second syntax to explain.
   Revisit if the generator exceeds roughly 400 lines.

## Decision

- Schema: **YAML**, under `protocol/schema/`, with a generator that validates rather than trusts.
- Generator: **Python 3.11+**, plain string building, no templating library.
- Output: `protocol/generated/messages.hpp` and `protocol/generated/messages.py`, gitignored.
- The generator is **deterministic** and its output is pinned by a golden-file test, so a
  change to generated code shows up as a reviewable diff rather than appearing silently.

## The schema carries units, ranges, and rates

This is the part that earns the generator its keep. Each field declares its type, its SI
unit, and its valid range. From one declaration the generator emits:

- the C++ and Python struct definitions and codec
- a doc comment stating the unit and range, so units appear at the point of use
- **range validation in the decoder**, which is a real defense: a corrupt frame that passes
  CRC-16 still has to produce physically plausible values (see ADR 0002 consequences)
- **a lint that a field's name carries its unit suffix** — a field declared `unit: m/s` whose
  name does not end in `_m_s` fails the build

That last check turns CLAUDE.md's hard rule 5 from a convention into something enforced. The
same applies to rule 6: a range or default with no `source:` note fails the build, so every
constant must say whether it came from a datasheet, a measurement, a derivation, or is an
initial guess awaiting tuning.

## Consequences

- **The generator becomes a hard build dependency.** Generated output is gitignored, so every
  clone and every CI run must be able to run it. A fresh clone with no Python cannot build the
  firmware. Accepted deliberately: committing generated code invites someone editing it, which
  is exactly the two-sources-of-truth failure the schema exists to prevent.
- **CMake must run the generator before compiling** anything that includes `messages.hpp`, via
  a custom command with correct dependencies on the schema files. Getting that dependency wrong
  produces stale-generated-code bugs that look like impossible behavior, so it needs a test
  that touches the schema and asserts a rebuild.
- **YAML's loose typing is a real hazard.** `0x10` may parse as a string and `1.0` as a float
  where an integer was meant. The generator validates every value's type explicitly and fails
  loudly rather than coercing.
- **PyYAML is a new dependency** in `requirements-dev.txt`. TOML via stdlib `tomllib` would
  avoid it; if dependency minimalism later matters more than editing ergonomics, that is a
  cheap superseding decision.
- A schema change regenerates both languages at once, so C++ and Python cannot drift. Shared
  test vectors (ADR 0002, `protocol/design-proposal.md`) verify they actually agree rather
  than merely being generated together.
