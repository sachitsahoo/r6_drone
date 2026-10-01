# firmware/hal — interface design proposal

**Status: APPROVED by the owner, 2026-10-01, and implemented** in `firmware/hal/hal/*.hpp`.
Owner-reviewed because these interfaces are the contract for motor control, the IMU and
safety. The owner's answers to the three questions are recorded at the end.

## Goal

Declare the interfaces through which `firmware/core/` touches hardware, so that slice 5 (the
minimal simulator) can implement them in `sim/` and the first software-in-the-loop test can run
`core` with no hardware. The STM32 implementations come later, behind the same headers.

## What every interface follows

These apply to all eight interfaces below, so they are stated once.

1. **Non-blocking, poll the latest value.** `core` runs fixed-rate loops and asks for the most
   recent sample. Interrupts and DMA live in the STM32 implementation and fill buffers there.
   No interface method waits, sleeps or spins. That keeps hard rule 3 (no blocking in loops or
   ISRs) enforceable at the boundary.
2. **Every sample carries its own timestamp and a validity flag.** `timestamp_us` is the
   `uint32_t` microsecond clock (wrap handled by `core::elapsed_us`). `valid == false` means
   "no fresh, trustworthy reading". The estimator and the safety state machine decide what to
   do about it; the HAL never guesses a value.
3. **SI units, unit suffixes, and project sign conventions are part of the interface.** Each
   implementation converts and flips signs so that `core` never knows which way a sensor was
   mounted. x forward, y left, z up; **pitch positive nose-down** (positive about +y).
4. **`float`, not `double`.** The G474's FPU is single precision; a `double` would silently run
   in software.
5. **Abstract base classes with a *protected, non-virtual* destructor.** A public virtual
   destructor generates a "deleting destructor" that references `operator delete`. With no
   heap (hard rule 2), that can fail to link or drag in heap code. Nothing ever deletes a HAL
   object through a base pointer, since they are all statically allocated, so this is safe.
6. **Header-only, `namespace recon::hal`, no vendor headers.** A CMake `INTERFACE` library
   `recon_hal`, linked by `recon_core`. `tools/check_core_purity.py` already bans vendor
   headers from `core`; it will be extended to cover `hal/` too.
7. **Each method states its ISR safety.** Methods the FOC loop calls from a timer interrupt
   must be ISR-safe in every implementation.

## The interfaces

```cpp
namespace recon::hal {

// ---------------------------------------------------------------- Clock
class Clock {
 public:
  /// Monotonic microseconds since boot. Wraps every ~71.6 min (core::elapsed_us).
  /// ISR-safe.
  virtual uint32_t now_us() const = 0;
 protected:
  ~Clock() = default;
};

// ----------------------------------------------------------- SerialPort
/// Byte pipe to the Pi. Framing (COBS, CRC) is core/protocol's job, not this one's.
class SerialPort {
 public:
  /// Copies up to `len` bytes into the transmit buffer. Returns how many were accepted;
  /// fewer than `len` means the buffer is full. Never blocks. Not ISR-safe.
  virtual size_t write(const uint8_t* data, size_t len) = 0;
  /// Copies up to `len` received bytes into `out`. Returns the count, 0 if none. Never
  /// blocks. Not ISR-safe.
  virtual size_t read(uint8_t* out, size_t len) = 0;
  /// Bytes lost to receive-buffer overflow since boot (wrapping). Feeds LinkStats.
  virtual uint32_t rx_overflow_count() const = 0;
 protected:
  ~SerialPort() = default;
};

// ------------------------------------------------------------------ Imu
struct ImuSample {
  float gyro_rad_s[3];      ///< casing frame, x forward / y left / z up; [1] is pitch rate,
                            ///< positive nose-down
  float accel_m_s2[3];      ///< casing frame, includes gravity (+9.81 on z when level)
  uint32_t timestamp_us;
  bool valid;
};
/// The ICM-42688-P on the casing (ADR 0005). Raw, unfiltered: filtering is core's job.
class Imu {
 public:
  virtual ImuSample latest() const = 0;   ///< ISR-safe
 protected:
  ~Imu() = default;
};

// ------------------------------------------------------- AbsoluteEncoder
struct AngleSample {
  float angle_rad;          ///< [0, 2pi), within one turn
  uint32_t timestamp_us;
  bool valid;               ///< false on magnet-field or communication fault
};
/// The single pitch encoder (ADR 0011): casing angle relative to chassis, within one turn.
/// Positive is the casing rotating nose-down relative to the chassis. Turn counting for the
/// wire loop (ADR 0009) and the zero offset are core's job, not this one's.
class AbsoluteEncoder {
 public:
  virtual AngleSample latest() const = 0; ///< ISR-safe: the FOC loop reads it
 protected:
  ~AbsoluteEncoder() = default;
};

// -------------------------------------------------------- WheelEncoder
/// An N20 quadrature encoder. A raw, wrapping counter rather than a velocity, so that core
/// owns the differentiation, its filtering, and the wraparound handling (hard rule 7, tested
/// once in core rather than once per implementation).
class WheelEncoder {
 public:
  /// Counts since boot, wrapping at 2^32. Positive = wheel rolling the robot forward.
  virtual uint32_t count() const = 0;     ///< ISR-safe
  /// Counts per wheel revolution, including the gearbox. Constant per build.
  virtual float counts_per_rev() const = 0;
 protected:
  ~WheelEncoder() = default;
};

// ----------------------------------------------------------- WheelMotor
enum class StopMode : uint8_t { kCoast, kBrake };
/// One channel of the TB6612. Knows nothing about speed or PID.
class WheelMotor {
 public:
  /// Duty in [-1, 1]; positive drives the robot forward. Out-of-range values are clamped by
  /// the implementation. A per-build output limit (CLAUDE.md: low by default) is applied
  /// on top of the clamp.
  virtual void set_duty(float duty) = 0;
  virtual void stop(StopMode mode) = 0;
 protected:
  ~WheelMotor() = default;
};

// ------------------------------------------------------ PitchPowerStage
struct PhaseCurrents {
  float a_A, b_A, c_A;
  uint32_t timestamp_us;
  bool valid;
};
/// The bought three-phase power stage (ADR 0007): SimpleFOCMini, DRV8313. Takes three duty
/// cycles; knows nothing about FOC. Inverse Park, SVPWM and the electrical angle are core's.
class PitchPowerStage {
 public:
  /// Each duty in [0, 1], clamped. Written to the timer's compare registers; takes effect at
  /// the next PWM period. ISR-safe: called from the FOC loop.
  virtual void set_phase_duties(float a, float b, float c) = 0;
  /// Drives the stage's enable lines. Disabled means all phases floating.
  virtual void set_enabled(bool enabled) = 0;
  /// The DRV8313's nFAULT line (overcurrent, overtemperature, undervoltage). ISR-safe.
  virtual bool fault() const = 0;
  /// Optional: only a current-sensing stage (SimpleFOCMini v2.3) has these. Lets the board
  /// be chosen at purchase time without changing core.
  virtual bool has_current_sense() const = 0;
  virtual PhaseCurrents latest_currents() const = 0;
 protected:
  ~PitchPowerStage() = default;
};

// --------------------------------------------------------- PowerMonitor
struct PowerSample {
  float bus_voltage_V;
  float current_A;          ///< positive = discharging the battery
  uint32_t timestamp_us;
  bool valid;
};
/// The INA226 on the battery line (ADR 0012).
class PowerMonitor {
 public:
  virtual PowerSample latest() const = 0;
 protected:
  ~PowerMonitor() = default;
};

}  // namespace recon::hal
```

## Choices worth defending

| Choice | Alternative | Why this way |
|---|---|---|
| Pitch drive split into `PitchPowerStage` + `AbsoluteEncoder` | One `PitchActuator` that takes an angle or torque | ADR 0007: FOC is ours and lives in `core`, so the HAL must stop at the power stage. One encoder is shared by FOC and estimation (ADR 0011), so it is its own interface, not part of the motor |
| Wheel encoder returns raw counts | Returns velocity in rad/s | Velocity needs differentiation and filtering, which is policy. Raw counts also keep the wrap handling in one tested place |
| Wheel motor takes duty, not voltage or speed | Takes a speed setpoint | The velocity PID is core's (CLAUDE.md). The HAL is the mechanism |
| Samples carry `valid` + timestamp | Return status codes, or throw | No exceptions (hard rule 3). A struct is copyable in an ISR and self-describing in a log |
| `Imu` reports the casing frame | Reports the chip's own axes | Mounting orientation is a fact about the build, so the implementation owns it. `core` sees the project frame only |

## How it gets tested

Interfaces have no behaviour, so the tests are of their users:

- `tests/cpp/hal_fakes.hpp`: one fake per interface. Settable samples, recorded commands
  (e.g. the last three phase duties). Used by every core unit test.
- `sim/`: physics-backed implementations of the same headers (slice 5). The first SIL test
  closes a loop through them, with frames round-tripping through the real codec.
- A compile-only test checks that every interface header builds with `-fno-exceptions
  -fno-rtti` under the STM32 toolchain, with no vendor headers.

## Not in this proposal

- **The implementations.** STM32 timer, DMA and interrupt configuration is owner-reviewed
  separately.
- **A hardware watchdog interface.** The 200 ms comms watchdog is software in `core`, built on
  `Clock`. Whether the MCU's independent hardware watchdog (IWDG) also gets an interface is a
  safety design question. My suggestion: yes, a one-method `kick()`, decided with the safety
  state machine.
- **Loop timing (hard rule 4).** Measured execution time comes from `Clock::now_us()` at 1 µs
  resolution. If µs proves too coarse for the 20 kHz FOC loop, a cycle-counter method can be
  added then.

## Owner decisions (2026-10-01)

1. **Pitch drive split into `PitchPowerStage` and `AbsoluteEncoder`: approved.**
2. **Wheel stop on comms timeout: coast, escalating to brake if the link stays down long
   enough.** Coast lets a thrown robot keep rolling; brake holds it on a slope once it is
   clearly not coming back. The interface provides both modes; the escalation delay is set in
   the safety state machine design, which is owner-reviewed.
3. **Hardware watchdog: designed with the safety state machine**, not now.
