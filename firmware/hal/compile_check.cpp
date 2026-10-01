// Compiled on BOTH host and stm32 with the firmware language subset (-fno-exceptions
// -fno-rtti). The interfaces are header-only, so without this file nothing would prove they
// build under the cross compiler. The static_asserts turn the design proposal's rules into
// build failures rather than conventions.

#include <type_traits>

#include "hal/absolute_encoder.hpp"
#include "hal/clock.hpp"
#include "hal/imu.hpp"
#include "hal/pitch_power_stage.hpp"
#include "hal/power_monitor.hpp"
#include "hal/serial_port.hpp"
#include "hal/wheel_encoder.hpp"
#include "hal/wheel_motor.hpp"

namespace recon::hal {
namespace {

// Rule: interfaces are abstract, and their destructors are NOT virtual. A virtual destructor
// would emit a deleting destructor referencing operator delete (hard rule 2).
template <typename T>
constexpr bool kIsHalInterface = std::is_abstract_v<T> && !std::has_virtual_destructor_v<T>;

static_assert(kIsHalInterface<Clock>);
static_assert(kIsHalInterface<SerialPort>);
static_assert(kIsHalInterface<Imu>);
static_assert(kIsHalInterface<AbsoluteEncoder>);
static_assert(kIsHalInterface<WheelEncoder>);
static_assert(kIsHalInterface<WheelMotor>);
static_assert(kIsHalInterface<PitchPowerStage>);
static_assert(kIsHalInterface<PowerMonitor>);

// Rule: samples are plain data, so an ISR can copy them by value and a log can store them.
static_assert(std::is_trivially_copyable_v<ImuSample>);
static_assert(std::is_trivially_copyable_v<AngleSample>);
static_assert(std::is_trivially_copyable_v<PhaseCurrents>);
static_assert(std::is_trivially_copyable_v<PowerSample>);

// Rule: single precision only. The G474's FPU has no double-precision unit, so a double in a
// sample would silently run in software.
static_assert(std::is_same_v<decltype(ImuSample::gyro_rad_s[0]), float&>);
static_assert(std::is_same_v<decltype(AngleSample::angle_rad), float>);

}  // namespace
}  // namespace recon::hal
