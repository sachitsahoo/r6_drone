# 0016 — STM32 register access: CMSIS headers only

- **Status:** **ACCEPTED** by the owner, 2026-10-01 (chosen from four options in chat).
  Owner-reviewed area: STM32 configuration.

## Context

ADR 0015 decides what runs where on the G474, but not how `firmware/stm32/` reaches the
peripherals. Every option needs vendor files that this repository does not contain.

## Options considered

| Option | For | Against |
|---|---|---|
| **CMSIS only**: ST's device register header + Arm's core header; own small drivers | smallest; nothing hidden; every register write is in our code and can be explained | more reading of the reference manual (RM0440) |
| ST Low-Layer (LL) drivers | readable inline wrappers, still thin | another library whose behaviour must be trusted or read |
| ST HAL + CubeMX | fastest bring-up | heavy, callback-driven, hides timing; generated files are hand-edited around, against the project's "generated is never edited" rule |
| Write no STM32 code until a board exists | nothing unverified is written | the glue's compile-and-link problems are found later, with hardware on the bench |

## Decision

**CMSIS only.** Fetched at configure time, pinned by tag and SHA-256, the way GoogleTest is:

| Package | Version | Licence | Used for |
|---|---|---|---|
| `STMicroelectronics/cmsis-device-g4` | v1.2.6 | Apache-2.0 | `stm32g474xx.h` (register map, bit macros), `system_stm32g4xx.c`, GCC startup file |
| `ARM-software/CMSIS_6` | v6.3.0 | Apache-2.0 | `core_cm4.h` (NVIC, SysTick, SCB) |

The linker script is ours (ST ships none for GCC in this package). Register **names and bit
positions** come from the header, so the compiler checks them. Register **values** (key
sequences, DMAMUX request numbers, prescaler encodings) are named constants in our code, each
citing RM0440 and marked [UNCLEAR] until checked against the manual or on the board.

## Consequences

- A clean configure of the stm32 target downloads about 11 MB (the CMSIS_6 archive includes
  docs). The host build downloads nothing new.
- Vendor headers are confined to `firmware/stm32/`; the CI purity guard keeps them out of `core`.
- Nothing written against these headers is verified until it runs on a Nucleo. Each driver
  lists what to check in `docs/bringup/stm32-first-image.md`.
