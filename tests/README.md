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
| `cpp/test_hal_fakes.cpp`, `cpp/hal_fakes.hpp` | Test doubles for every HAL interface. |
| `cpp/test_sim.cpp` | Simulated HAL: clock, UART (rate, overflow, bit errors, cut), wheel plant, encoder freeze/reverse, `SimWatchdog`. |
| `cpp/test_sil_link.cpp` | SIL: frames across the simulated UART, under bit errors and a clock wrap. |
| `cpp/test_wheel_speed_estimator.cpp`, `cpp/test_wheel_velocity_controller.cpp`, `cpp/test_diff_drive.cpp`, `cpp/test_drive_loop.cpp` | ADR 0013 wheel loop units. |
| `cpp/test_sil_drive.cpp` | SIL: wheel loop on the plant (step, ramp, turn, carpet, slope, ±50% plant mismatch). |
| `cpp/test_wheel_stall_detector.cpp`, `cpp/test_check_in_monitor.cpp` | ADR 0014 WHEEL_STALL detector and IWDG check-in gate. |
| `cpp/test_safety_supervisor.cpp` | ADR 0014: the full state × event transition table (completeness enforced), interlocks, comms watchdog timing and wraps, escalation, clearing. |
| `cpp/test_sil_safety.cpp` | SIL: link cut, unplugged and reversed encoder, carpet full stick, watchdog-reset boot, e-stop latency, IWDG starvation. |
| `cpp/protocol_vector_tool.cpp` | Host-only CLI exposing the real codec for cross-language tests. |
| `python/test_schema.py` | Schema validity, documented payload sizes, every validator rule. |
| `python/test_generator.py` | Determinism, `--check` staleness detection, banner, no banned constructs. |
| `python/test_codec.py` | Python codec, `hypothesis` fuzzing, timestamp unwrapping. |
| `python/test_cross_language.py` | C++ and Python agree byte for byte, against committed vectors. |
| `python/test_check_core_purity.py` | The CI firewall guard's own tests. |
| `python/test_repo_layout.py` | Every module directory exists and has a non-trivial README. |

## Known limitations

- Behaviour tests cover the wheel loop (ADR 0013) and the safety machine (ADR 0014) against
  a **placeholder** plant. No estimator or pitch loop exists yet, and nothing is checked on
  hardware.
- The SIL tests' frame-to-supervisor glue is test code (`test_sil_safety.cpp`'s `Robot`), not
  firmware: the real glue waits on the STM32 timer design (ADR 0015, proposed).

## How to test

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
pytest
```
