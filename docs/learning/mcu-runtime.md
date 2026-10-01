# Learning note — the MCU runtime (interrupts, mailbox, watchdog)

A plain-language walkthrough of [ADR 0015](../decisions/0015-mcu-loop-timing-and-safety-glue.md),
[ADR 0016](../decisions/0016-stm32-register-access.md), [`firmware/core/runtime/`](../../firmware/core/runtime/)
and [`firmware/stm32/`](../../firmware/stm32/).

## The job

`core` has the robot's logic: the wheel loop and the safety machine. Something has to *run*
it on the chip. The motor loop must tick exactly every millisecond. Frames from the laptop have
to reach it. Its decisions have to reach the motors. And if any of that ever stops, the chip
has to reset itself.

## Two things running at once

The G474 has one core, but two contexts share it:

- **The motor-loop interrupt (TIM6, 1 kHz).** A hardware timer interrupts whatever is running,
  every millisecond, on the dot. Its handler runs `MotorLoop::tick()`: take the latest frames,
  step the safety supervisor, step the wheel loop, stop or drive the motors. Then it returns.
- **The main loop (`main()`'s `while (true)`).** It runs whenever the interrupt isn't. It reads
  bytes off the UART, decodes frames, throws away replays, hands the rest to the motor loop,
  sends Fault and Nack frames back, and feeds the watchdog.

The motor loop is in an interrupt so its timing never depends on how long the main loop takes
to decode a frame. There's no RTOS, because two contexts don't need a scheduler. ADR 0015
lists when that would change.

## The mailbox: passing data without locks

The main loop writes, and the interrupt reads. The danger is the interrupt firing halfway
through a write and seeing half-new, half-old data: a new linear speed with an old turn rate.
A mutex can't fix that. The interrupt can't wait for the main loop to finish, because the main
loop can't run until the interrupt returns.

So each kind of data gets a structure that is safe by construction:

- **E-stop:** one atomic flag. Setting it is a single instruction.
- **State requests:** a small queue (`SpscRing`). The writer only moves the tail, the reader
  only moves the head, and each publishes its index only after the slot is complete.
- **The latest DriveCommand:** two slots. The writer fills the slot the reader is *not* using,
  then flips one atomic counter to say "this one is newest now". The reader always reads a slot
  that is either finished or untouched.

## The hardware layer, briefly

- **Clock:** TIM2 is a 32-bit timer counting microseconds. Its counter *is* the timestamp, so
  it wraps at exactly 2³² µs, the way the rest of the code already expects.
- **UART:** bytes arrive by DMA, which copies them into a ring buffer with no CPU involvement.
  The main loop reads them out of the ring. An interrupt counts each time the DMA laps the
  ring, so the driver always knows how many bytes have arrived. If the main loop falls more
  than a whole ring behind, it knows exactly how many bytes it lost, and counts them.
- **Watchdog:** the IWDG runs on its own oscillator and resets the chip unless fed every 50 ms.
  `MainLoop` feeds it only when the motor loop has also checked in.
- **Motors:** not yet. This image's motor drivers are stubs that do nothing.

## Check your understanding

1. **Why not use a mutex between the main loop and the motor-loop interrupt?** If the main loop
   holds the lock when the interrupt fires, the interrupt has to wait for the main loop. But
   the main loop can't run until the interrupt finishes. On one core that is a deadlock.
   Everything shared has to be wait-free for the interrupt.

2. **The DriveCommand uses two slots. Why isn't one slot plus a "being written" flag enough?**
   With one slot, the interrupt can land mid-write. It would see the flag set and have nothing
   valid to use; it can't wait, so it would have to skip the command. With two slots, the
   previous complete command is always sitting in the other slot.

3. **Why does a restarted laptop app need the stale-filter reset?** Replay protection accepts
   only frames newer than the last one. A restarted app's clock starts again near zero, so
   every frame looks *older* and would be rejected forever. After 200 ms with nothing
   accepted, the filter forgets, and the new app's first frame sets a new baseline. That
   silence also means the robot is already in FAULT if it was armed, so a replayed old frame
   arriving first can't make it move.

4. **Why is the e-stop exempt from replay rejection?** A replayed stop is harmless, and
   refusing a real one is not. When in doubt, stop.

5. **The linker script has no heap. What happens if some code calls `malloc`?** The build fails
   at link time. That happened on the first build: libstdc++'s assertion handler pulls in
   `abort()`, which pulls in stdio, which needs `malloc`. The fix was a handler of our own that
   spins until the watchdog resets the chip. Failing the build is much better than finding a
   heap in use on the robot.

6. **Why does the first image run at 16 MHz instead of the chip's 170 MHz?** 16 MHz is the
   internal oscillator the chip boots on, so it needs no setup. 170 MHz needs the PLL, extra
   flash wait states and a voltage-range change. Those are more registers to get wrong, and
   there's no board yet to check them on. The control code is light enough for 16 MHz;
   `max_exec_us` will show by how much.

7. **Why read the reset reason before doing anything else in `main()`?** The flags say why the
   chip last reset. Some later step could reset the chip again, or something could clear the
   flags, and the evidence would be gone. It's read first, kept, then cleared, so the next boot
   reports a fresh cause.

8. **How is a failure in this layer different from a failure in `core`?** `core` runs in host
   tests and the simulator, so most mistakes there show up in CI. This layer only runs on the
   chip, so nothing here is verified until the bring-up checklist has been worked through on a
   real board. That's why it's kept so thin.
