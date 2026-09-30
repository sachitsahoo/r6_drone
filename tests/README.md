# tests — unit and software-in-the-loop tests

## What it does

Holds the C++ unit and SIL tests (`cpp/`) and the Python tests (`python/`) that CI runs on
every push.

## How it fits the architecture

The reason [`../firmware/core/`](../firmware/core/) is vendor-free. SIL tests run the real
control code against [`../sim/`](../sim/)'s simulated HAL, so a control law can be exercised
before any hardware exists. Per [`../CLAUDE.md`](../CLAUDE.md), tests are written from the
acceptance criteria *before* the implementation, and a failing test is never weakened,
skipped, or deleted to get to green.

## Key design decisions

- **Virtual time in SIL tests.** The simulated `Clock` is advanced explicitly by the test
  rather than tracking wall time, so results are deterministic and CI cannot flake on timing.
- **Wraparound and overflow get dedicated tests.** A `uint32_t` microsecond timestamp wraps
  every ~71 minutes and encoder counters wrap too; both are cheap to test and expensive to
  discover in the field.
- **The protocol decoder is fuzzed**, not just round-trip tested. Round-trip tests only prove
  it handles frames it produced itself.

## Layout

| Path | Contents |
|---|---|
| `cpp/test_timestamp.cpp` | `uint32_t` microsecond wraparound, including the watchdog interval. |
| `cpp/test_crc32.cpp` | Published known-answer value, streaming vs one-shot, every single-bit flip. |
| `cpp/test_cobs.cpp` | Paper vectors, block boundaries, trailing zeros, malformed input, garbage. |
| `cpp/test_frame_codec.cpp` | Round-trip, resynchronization, every rejection path, 200k-byte fuzz. |
| `cpp/test_generated_messages.cpp` | Byte order, NaN and range rejection, magic constants, enums. |
| `cpp/protocol_vector_tool.cpp` | Host-only CLI exposing the real codec for cross-language tests. |
| `python/test_schema.py` | Schema validity, documented payload sizes, every validator rule. |
| `python/test_generator.py` | Determinism, `--check` staleness detection, banner, no banned constructs. |
| `python/test_codec.py` | Python codec, `hypothesis` fuzzing, timestamp unwrapping. |
| `python/test_cross_language.py` | C++ and Python agree byte for byte, against committed vectors. |
| `python/test_check_core_purity.py` | The CI firewall guard's own tests. |
| `python/test_repo_layout.py` | Every module directory exists and has a non-trivial README. |

## Known limitations

- No tests of robot behavior, because no control law, estimator, or safety state machine
  exists yet. Current coverage is the protocol, the build system, and the CI guards.

## How to test

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
pytest
```
