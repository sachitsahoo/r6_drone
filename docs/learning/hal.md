# Learning notes: the hardware abstraction layer (`firmware/hal/`)

A walkthrough of the eight interfaces between the control code and the hardware, and why they
look the way they do. Design record: [`firmware/hal/design-proposal.md`](../../firmware/hal/design-proposal.md).

## The idea in one paragraph

The control code in `firmware/core/` never touches a register. When it needs the IMU, it
calls `imu.latest()` on an object it was handed, and doesn't know whether that object is the
real ICM-42688-P on SPI, the simulator, or a test fake. Each `hal::` class is a contract:
what the call returns, in what units, with what sign, and whether it is safe to call from an
interrupt. The STM32 code, the simulator and the fakes each implement the same contract. That
is what lets the whole control stack be tested on a laptop.

## The eight interfaces

| Interface | What `core` gets | Units and sign |
|---|---|---|
| `Clock` | `now_us()` | µs since boot, wraps every ~71.6 min |
| `SerialPort` | `write` / `read` bytes | raw bytes; framing is `core/protocol`'s |
| `Imu` | gyro + accel sample | rad/s, m/s², casing frame; pitch positive nose-down |
| `AbsoluteEncoder` | casing-to-chassis angle | rad in [0, 2π), within one turn |
| `WheelEncoder` | raw count | counts, wrapping at 2³²; positive = forward |
| `WheelMotor` | `set_duty`, `stop` | duty in [−1, 1]; positive = forward |
| `PitchPowerStage` | three phase duties | each in [0, 1] |
| `PowerMonitor` | battery sample | V, A; positive current = discharging |

## The rules, and why

**Poll the latest value; never wait.** The control loops run at fixed rates (1 kHz for the
motors, faster for FOC). A call that waited for a sensor would make loop timing depend on
the sensor. So interrupts and DMA fill buffers inside the implementation, and the
interface just hands over the last finished reading.

**Every sample has `timestamp_us` and `valid`.** When an IMU read fails, the worst thing the
HAL could do is return zeros that look like a level, motionless robot. Instead it says "not
valid", and the estimator and safety state machine decide what that means.

**The implementation owns orientation and sign.** The IMU chip may be mounted rotated, and
the left wheel encoder counts backwards relative to the right. Those are facts about the
build, so the implementation fixes them. `core` only ever sees the project frame: x forward,
y left, z up, pitch positive nose-down.

**Raw counts, not velocities, from the wheel encoders.** Turning counts into speed means
differentiating and filtering, which is a control decision. Raw counts also mean the 2³²
wraparound is handled once, in `core`, with one set of tests (hard rule 7).

**The pitch motor is two interfaces, not one.** ADR 0007 says we write the FOC ourselves, so
the HAL has to stop at the power stage: three duty cycles in, nothing about angles or torque.
And because the motor is direct drive, its rotor angle *is* the casing angle, so the one
encoder serves both FOC and the estimator. That makes it its own interface.

**Protected, non-virtual destructors.** Normally a C++ base class with virtual functions gets
a virtual destructor. Here that would generate code referencing `operator delete`, and the
firmware has no heap. Nothing is ever deleted through a HAL pointer (everything is
statically allocated), so the destructor is protected to make deleting through the base
impossible, and non-virtual to avoid the heap reference. `compile_check.cpp` makes the
compiler enforce this.

**`float`, not `double`.** The G474's floating-point unit is single precision. A `double`
would compile fine and silently run in software, an order of magnitude slower.

## Check your understanding

1. **Why doesn't `Imu::latest()` start a new SPI read?**
   *Because it would block the caller until the read finished, making loop timing depend
   on the bus. The implementation reads continuously in the background; `latest()` returns
   the last completed sample.*

2. **The IMU's SPI bus errors once. What does `latest()` return, and who decides what to do?**
   *A sample with `valid == false`. The estimator and the safety state machine in `core`
   decide, for example by holding the last estimate or faulting after N misses. The HAL
   never substitutes a value.*

3. **Why does `WheelEncoder` return a `uint32_t` count, and how do you get a correct delta
   across the wrap?**
   *Unsigned subtraction: `now - before` is correct modulo 2³² even when `now` has wrapped
   past zero, because C++ defines unsigned arithmetic as modular. Signed arithmetic or `<`
   comparisons would break at the wrap.*

4. **Where does the left wheel's sign flip happen, and why there?**
   *In the implementation. Mounting is a property of the build, so `core` stays identical for
   both wheels and for the simulator.*

5. **Why is `PitchPowerStage` not a `PitchActuator` that takes a target angle?**
   *ADR 0007 puts FOC in `core`, so the HAL has to stop at the power stage. A target-angle
   interface would hide the commutation and the stabilizer inside the HAL, which is the
   black box ADR 0007 rejected.*

6. **Why does `has_current_sense()` exist?**
   *So the SimpleFOCMini v1 (no current sensing) or v2.3 (with) can be chosen at purchase
   time without changing `core`. With v1, `latest_currents().valid` is always false.*

7. **What would go wrong if `Clock`'s destructor were `public virtual`?**
   *The compiler would emit a deleting destructor that calls `operator delete`. On a no-heap
   build that either fails to link or pulls in heap code. `compile_check.cpp`'s
   `static_assert` would also fail the build.*

8. **Which methods must be ISR-safe, and why those?**
   *Everything the FOC loop calls from its timer interrupt (`set_phase_duties`,
   `AbsoluteEncoder::latest`, `fault`), plus the stop and disable calls a fault handler
   needs (`WheelMotor::stop`, `set_enabled`). `SerialPort` is main-loop only.*

9. **On a comms timeout, what do the wheels do?**
   *Coast first, so a thrown robot keeps rolling, then brake if the link stays down long
   enough (owner decision). The HAL provides both modes; the timing lives in the safety state
   machine.*
