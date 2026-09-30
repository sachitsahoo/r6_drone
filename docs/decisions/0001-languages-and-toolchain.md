# 0001 — Languages and toolchain

- **Status:** Accepted
- **Date:** 2026-09-30

## Context

The project spans three tiers with very different constraints: a bare-metal STM32 running
1 kHz control loops, a Linux SBC relaying video, and a laptop application with a UI. Phase 1
needs a language and build choice for each before any code is written, and the choice has
to support the central architectural bet — that portable control code in `firmware/core/`
compiles and is tested on a laptop as well as on the target.

## Options considered

**Firmware language**

1. **C++17** — Zero-cost abstractions, strong typing for units and interfaces, and abstract
   base classes for the HAL seam. Costs discipline: the language has many features
   (exceptions, RTTI, the allocating parts of the standard library) that are unsafe here and
   must be switched off explicitly.
2. **C11** — Simpler, universal in embedded, no accidental allocation. But the HAL seam
   becomes function-pointer structs, and there is no type system help for keeping units and
   frames straight — a category of bug this project is specifically trying to avoid.
3. **Rust** — Memory safety, excellent type-level unit encoding, real package management.
   Rejected for now on ecosystem and personal-fluency grounds: STM32G4 HAL support is less
   mature than the vendor C HAL, and the owner must be able to defend every line. A language
   being learned alongside control theory and hardware bring-up is one variable too many.

**Host-side language**

4. **Python 3.11+** for the operator app, bridge, and tools — fastest path to a UI, plots,
   and scripting; already the ecosystem for SDL2 bindings, numerical work, and analysis.
   Not real-time, which is acceptable: nothing time-critical runs off the MCU.

**Build system**

5. **CMake** — one build description for both the host and `arm-none-eabi` targets via a
   toolchain file, and the de facto standard for C++ tooling and IDE integration.
6. **Make** — fewer moving parts, but dual-target and dependency fetching become hand-rolled.
7. **PlatformIO / STM32CubeIDE** — fast to start, but both pull the project toward a
   vendor-shaped layout and make a clean host build of `core` awkward. That directly
   undermines the portability firewall.

## Decision

- Firmware and simulator: **C++17**, compiled `-fno-exceptions -fno-rtti`, no dynamic
  allocation after init. Target compiler `arm-none-eabi-gcc`.
- Operator app, bridge, tools: **Python 3.11+**.
- Build: **CMake** (>= 3.20) with a toolchain file for the target.
- Tests: **GoogleTest** for C++, **pytest** for Python.
- CI: **GitHub Actions** running host build, all tests, and a target cross-compile on every push.

## Consequences

- Two languages and two test frameworks to maintain, and a protocol code generator to keep
  their message definitions in sync. That generator cost is accepted deliberately — see
  `protocol/README.md`.
- Switching off exceptions and RTTI means errors are return values and explicit state. This
  has to be a habit, because the compiler only enforces it where the flags are applied.
- CMake's toolchain-file approach means the target build is configured in a separate build
  tree (`build-stm32/`), so host and target artifacts never mix.
- Choosing C++ over Rust is a bet that discipline plus CI enforcement substitutes for
  compiler-enforced safety. If that bet fails visibly — recurring memory or lifetime bugs in
  `core` — it should be revisited in a superseding ADR rather than patched over.
- The owner's fluency in Rust and in C++ will diverge further as this project grows. That is
  a real career-relevant cost of this decision, noted here so it is a choice and not an accident.
