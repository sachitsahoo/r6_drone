# Bring-up: the first STM32 image

The first image (`firmware/stm32/`, ADR 0015 and ADR 0016) runs the clock, the UART link, the
safety state machine and the IWDG on a **bare Nucleo-G474RE. It drives no motor**: the wheel and
pitch drivers are null stubs. Nothing here needs a motor, a battery, or anything wired to the
board, so the hardware-in-the-loop rules about wheels and kill switches don't come into play yet.

Every register value the device header could not check is marked [UNCLEAR] in the source. This
page is the list of checks that settles each one. Record results in the table at the end.

## You need

- Nucleo-G474RE and a USB cable (BOM wave 1). Nothing else.
- A flashing tool on the Mac. Not chosen yet; options are STM32CubeProgrammer (ST's GUI/CLI),
  `st-flash` (stlink tools), or OpenOCD. Choosing one is a small owner decision at bring-up.
- A serial terminal or script at **460 800 baud** on the ST-LINK virtual COM port. There is no
  operator app yet: the first useful tool is a short Python script that sends Heartbeat,
  SafetyStateRequest and EstopRequest through `protocol/codec.py` and prints the Fault and Nack
  frames that come back. [Not written yet; next tooling task.]

## Build

```
cmake -B build-stm32 -DTARGET=stm32 && cmake --build build-stm32
```

Output: `build-stm32/firmware/stm32/recon_firmware.elf` (debugger) and `.bin` (flasher).
The link prints FLASH and RAM use. At the time of writing that is 11.2 KB / 7.4 KB.

## Checks, in order

Each step depends on the ones before it.

| # | Check | How | Settles |
|---|---|---|---|
| 1 | Image runs at all | Flash; attach the debugger; break in `main()` | linker script memory map, startup file, vector table |
| 2 | Clock is 1 MHz | Break twice ~1 s apart (by wall clock); compare `TIM2->CNT` | TIM2 prescaler on HSI16, `kSysClock_Hz` |
| 3 | Motor loop at 1 kHz | Breakpoint counter in `TIM6_DAC_IRQHandler`, or read `MotorLoop::max_exec_us()` | TIM6 PSC/ARR, NVIC enable |
| 4 | UART link both ways | Send a Heartbeat; expect nothing back. Send a `SafetyStateRequest(FAULT)`; expect `Nack(RANGE_REJECT)` | PA2/PA3 AF12, `kBrr`, DMAMUX request 34, DMA circular RX, TX FIFO |
| 5 | Boot report | On power-up expect one `Fault(NONE, 0x800000xx)`; the low byte is the reset flags (power-on and pin reset bits set) | RCC_CSR flag positions |
| 6 | Arm and the comms watchdog | Stream Heartbeat + zero DriveCommand at 50 Hz, then arm, then stop streaming. Expect `Fault(COMMS_TIMEOUT)` | the whole glue path on silicon |
| 7 | WHEEL_STALL on silicon | Armed, stream a non-zero DriveCommand. The null wheel never turns, so expect `Fault(WHEEL_STALL, 3)` about 0.5 s later | stall detector, both wheels |
| 8 | IWDG timeout | Temporarily add an infinite loop in `main()` after 1 s. Expect a reset, then `Fault(WATCHDOG_RESET)` and a boot report with the IWDG bit. Time it: 45–55 ms | IWDG keys, PR = /4, RLR = 399, LSI tolerance (ADR 0014) |
| 9 | Debugger freeze | Halt at a breakpoint for 10 s; resume. No reset | DBGMCU `DBG_IWDG_STOP` |
| 10 | Stack margin | Paint RAM between `_ebss` and `_estack` with a pattern at boot (debugger), run steps 4–7, see how much was used | `_Min_Stack_Size` = 4 KB |
| 11 | Loop timing | Read `MotorLoop::max_exec_us()` after step 7 | hard rule 4; LOOP_OVERRUN margin at 16 MHz |

Step 8 is the one that matters most for safety. The IWDG is the only guard against a hung CPU
leaving the PWM running. Revert the infinite loop afterwards and confirm the revert in `git diff`.

## Results

| # | Date | Result | Notes |
|---|---|---|---|
| | | | |
