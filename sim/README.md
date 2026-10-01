# sim — physics model and simulated HAL

## What it does

Implements the [`../firmware/hal/`](../firmware/hal/) interfaces against a physics model
instead of hardware, so the real [`../firmware/core/`](../firmware/core/) can be run,
tested, and tuned on a laptop.

## How it fits the architecture

The simulator is not a separate model of the robot's software — it runs the same control
code that ships to the MCU. That is the payoff for keeping `core` vendor-free: a
software-in-the-loop (SIL) test can exercise a real control law with no hardware attached.

## Key design decisions

- **Runs `core` unmodified.** If the simulator ever needs a special case inside `core`,
  that is a design failure, not a shortcut.
- **Simulated `Clock` is virtual time**, advanced explicitly by the test. Loops can be
  stepped deterministically, and a SIL test is reproducible rather than timing-dependent.
- **The plant model starts deliberately crude.** Real parameters (inertia, motor constants,
  friction) come from system identification on hardware, which has not happened. A
  placeholder model that closes a loop is useful; a detailed model built from guessed
  constants is false confidence.

## What is here

| File | Implements | Notes |
|---|---|---|
| `sim/sim_clock.hpp` | `hal::Clock` | Virtual time; start it near 2^32 to test the wrap |
| `sim/sim_serial_link.hpp/.cpp` | `hal::SerialPort` x2 | 460 800 baud 8N1 line, 256 B buffers, RX overflow, seeded bit errors |
| `sim/wheel_plant.hpp/.cpp` | `hal::WheelMotor` + `hal::WheelEncoder` | First-order duty -> speed, exact discretisation, coast/brake, wrapping counter; optional constant load and extra viscous drag (slope / carpet cases) |
| `sim/static_sensors.hpp` | `Imu`, `AbsoluteEncoder`, `PowerMonitor`, `PitchPowerStage` | Settable constant samples stamped with virtual time; no dynamics yet |
| `sim/sim_world.hpp` | — | Owns one of everything and steps them together |

The library is host-only but compiled with the firmware language subset
(`-fno-exceptions -fno-rtti`) and allocates nothing.

## Known limitations

- The wheel velocity loop (ADR 0013) is closed on this plant in `tests/cpp/test_sil_drive.cpp`.
  That proves the loop's structure and logic, not its performance on hardware.
- The constant load acts in every mode (a slope); Coulomb friction, which cannot reverse
  motion, is not modelled.
- Every wheel-plant constant is a placeholder (see `sim/wheel_plant.hpp`). The gear ratio
  is not chosen and nothing has been measured.
- The output limit saturates rather than scales (owner decision, ADR 0013 question 3).
- No pitch dynamics: the IMU reports a level, still casing and the power stage only records
  duties. These arrive with the FOC and estimator designs.
- No camera or video simulation. Visual-stability evaluation needs real optics.
- Contact, impact, and tipping dynamics are out of scope for now.

## How to test

SIL tests live in [`../tests/`](../tests/) and run in CI with no hardware:

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
```
