# 0015 — MCU loop timing, IWDG driver and safety glue

- **Status:** **PROPOSED** 2026-10-01, awaiting owner review. Owner-reviewed area: STM32
  timer/interrupt/DMA configuration, watchdog. **Nothing here is implemented.**
- **Builds on:** ADR 0013 (DriveLoop, 1 kHz), ADR 0014 (SafetySupervisor, CheckInMonitor,
  `hal::Watchdog`, 50 ms IWDG), `docs/bringup/uart-link.md` (460 800 baud, circular DMA RX
  is mandatory), `protocol/design-proposal.md` (stale-command rejection).
- **Does not cover:** the pitch/FOC loop (no design yet), the param table and flash commit
  (next slice), telemetry scheduling beyond what the IWDG needs.

## Context

`firmware/core/` now has everything ADR 0013 and 0014 call for, as pure logic: DriveLoop,
SafetySupervisor, CheckInMonitor. None of it runs, because nothing calls it. The missing piece
is the layer ADR 0014 calls "the glue". It has to decide what runs in an interrupt and what in
the main loop, how a decoded frame reaches the 1 kHz loop, how the IWDG is configured and fed,
and how the boot reason is read. Every one of those is an owner-reviewed choice.

## Proposal

### 1. Two execution contexts, no RTOS

| Context | Rate | Runs | Checks in |
|---|---|---|---|
| **Motor-loop ISR**: a basic timer (TIM6) update interrupt | 1 kHz | build `SafetyInputs` from the mailbox and the last DriveLoop status → `SafetySupervisor::step` → `DriveLoop::step(armed)` → apply outputs (`WheelMotor::stop`, `PitchPowerStage::set_enabled`) → measure its own execution time | `kMotorLoop` |
| **Main loop**: superloop in `main()` | as fast as it runs; budget ≤ 1 ms per pass | drain the UART DMA ring → `FrameDecoder` → stale-timestamp check → write the mailbox → send queued Fault/Nack frames and telemetry → `CheckInMonitor::should_feed()` → `Watchdog::feed()` | `kMainLoop` |

- **Why the control loop is in an ISR.** Its period must not depend on how long the main
  loop's frame handling takes. A timer interrupt gives a fixed rate with jitter of a few
  cycles. Polling a tick flag from the main loop was the alternative; it ties loop jitter to
  the slowest thing the main loop does, such as a 64-byte telemetry encode.
- **Why no RTOS.** Two contexts and a mailbox are enough. An RTOS adds a scheduler, priority
  inversion, and stack sizing per task, none of which the design needs yet. It can be
  reconsidered when the pitch loop arrives.
- **The pitch loop** (later) gets its own, higher-priority timer interrupt and its own
  check-in bit. Nothing here blocks that.

### 2. The mailbox from main loop to ISR

The ISR must never see a half-written command. Proposed, single producer (main loop), single
consumer (ISR), lock-free:

- `estop`: an `std::atomic<bool>`, set by the main loop and cleared by the ISR on read. Never
  queued behind anything.
- `SafetyStateRequest`s: an SPSC ring of 4 entries. The ISR takes at most one per tick, as
  ADR 0014's inputs allow. A full ring Nacks the extra request (`UNSPECIFIED`) from the main
  loop, so it is visible rather than silently dropped.
- The latest `DriveCommand` and "other frame seen" flags: a **sequence-locked** struct (writer
  bumps a counter before and after; the reader retries on odd or changed counter). It is a
  double-buffer, so the ISR always reads a complete pair of floats.
- **Receive time** is stamped by the main loop with `Clock::now_us()` when the frame is
  decoded, and carried in the mailbox. The supervisor's `now_us` is the ISR's own clock read.
  The receipt-to-ISR delay is at most one main-loop pass plus one tick, the same bound ADR 0014
  gives for e-stop latency.

### 3. Stale-command rejection (protocol design, not yet implemented)

`protocol/design-proposal.md` says a command whose `timestamp_us` is not newer than the last
accepted command of its type is rejected. Two details the glue must get right:

- **"Newer" across the operator's clock wrap:** use `(int32_t)(t_new − t_last) > 0`, which is
  correct for gaps under 35.8 min.
- **Operator restart.** A restarted operator app's clock starts again near 0, so every command
  looks older than the last one, and the robot would reject them forever. Proposed: the "last
  accepted" timestamps are reset whenever the comms watchdog goes stale (link_fresh false).
  After 200 ms of silence, the first frame is accepted on its own. [Owner question 3]

Rejected frames are Nacked `STALE_TIMESTAMP` and never reach the supervisor, so they never feed
the comms watchdog (ADR 0014).

### 4. IWDG configuration

| Item | Proposed | Source / check |
|---|---|---|
| Clock | LSI, nominal 32 kHz | [UNCLEAR] confirm nominal and tolerance in the G474 datasheet (ADR 0014 assumes ±10%) |
| Prescaler | /4 → 8 kHz, 125 µs per count | [UNCLEAR] confirm the PR encoding in RM0440 |
| Reload | 400 counts − 1 = 399 → 50.0 ms | 12-bit RLR, so the range fits; [UNCLEAR] confirm the reload semantics (N or N+1 counts) |
| Start | software, in `main()` before the first loop pass; key sequence per RM0440 | the hardware-start option byte is not used, so a debug image can run without one |
| Window | disabled (no WWDG behaviour) | ADR 0014 rejected window watchdogs |
| Debug | IWDG frozen while the core is halted (DBGMCU freeze bit) | [UNCLEAR] confirm the register name for the G4 |
| Feed | only in the main loop, only when `CheckInMonitor::should_feed()` | ADR 0014 Q5 |

**Boot reason.** At the top of `main()`, read the RCC reset-flag register once, keep the IWDG
flag in a `bool`, then clear all reset flags so the next boot reads a fresh cause.
`StmWatchdog::reset_was_watchdog()` returns that `bool`. The other reset causes (brown-out, pin,
software) are logged once in a boot Fault frame context, but only the IWDG flag changes the
boot state. [Owner question 4]

### 5. Interrupt priorities (Cortex-M4 NVIC, lower number = more urgent)

| Priority | Source | Why |
|---|---|---|
| 1 | (reserved: pitch FOC timer) | the fastest, most timing-sensitive loop when it exists |
| 2 | TIM6 motor loop | fixed 1 kHz |
| 3 | UART RX DMA half/full and IDLE | only moves bytes; the ring absorbs any latency |
| lowest | everything else | |

The motor-loop ISR budget is set by ADR 0014's 5 ms LOOP_OVERRUN, but its *target* is
≤ 100 µs. It is measured and reported in LoopTiming (hard rule 4).

### 6. Hardware requirement carried from ADR 0014

Pull-downs on the TB6612 STBY and PWM inputs and the DRV8313 enable lines, so a chip in reset
(floating pins) drives nothing. [UNCLEAR] whether the SimpleFOCMini fits them. Check its
schematic before ordering.

## Alternatives considered

| Option | Rejected because |
|---|---|
| Motor loop polled from the main loop | loop jitter becomes the main loop's worst-case pass |
| Frames decoded in the UART interrupt | puts CRC and decode in an ISR; the mailbox problem moves, it doesn't go away |
| FreeRTOS tasks | more machinery than two contexts need; out of proportion before the pitch loop exists |
| Feeding the IWDG from the timer ISR | keeps a hung main loop alive (ADR 0014) |
| Mutex around the command struct | a mutex in an ISR either blocks or fails; sequence lock is wait-free for the reader |

## Test plan

- **Host:** the mailbox (SPSC ring, sequence lock, estop flag) as a pure `core` class with unit
  tests, including a writer interrupted mid-update. The stale-timestamp filter, including the
  operator-restart and wrap cases. A glue-level SIL that replaces `test_sil_safety.cpp`'s test
  `Robot` with the real glue running on sim HAL.
- **Target (bench, wheels off the ground, kill switch in reach):** LoopTiming exec times; a
  deliberate infinite loop in the main loop resets within 50 ms and boots into
  FAULT(WATCHDOG_RESET); a breakpoint does not reset; e-stop latency (GPIO toggle on decode
  and on brake, measured with a scope at the makerspace) ≤ 2 ms.

## Questions for the owner

1. **Motor loop in a timer ISR, frames in the main loop, no RTOS?** *Recommended: yes.*
2. **Mailbox: atomic e-stop flag, 4-deep request ring, sequence-locked command?**
   *Recommended: yes.*
3. **Reset the stale-timestamp baseline when the link goes stale**, so a restarted operator app
   is accepted? Without it, restarting the laptop app means power-cycling the robot.
   *Recommended: yes.*
4. **Report non-IWDG reset causes** (brown-out, pin, software) for information only, without
   booting into FAULT? A brown-out reset is worth seeing, but making it a fault would mean
   every battery swap needs a clear. *Recommended: report only.*
5. **Pull-downs:** confirm they go on the bench wiring now, before the first motor is powered.
   *Recommended: yes.* After an IWDG reset, a floating STBY can enable the TB6612.
