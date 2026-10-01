#pragma once

#include <cstdint>

#include "control/drive_loop.hpp"
#include "hal/clock.hpp"
#include "hal/pitch_power_stage.hpp"
#include "hal/wheel_motor.hpp"
#include "safety/check_in_monitor.hpp"
#include "safety/safety_mailbox.hpp"
#include "safety/safety_supervisor.hpp"

namespace recon::core {

/// Motor-loop period, us. CLAUDE.md: motor velocity loop at 1 kHz.
inline constexpr uint32_t kMotorLoopPeriod_us = 1000;

/// One motor-loop iteration: the body of the 1 kHz TIM6 interrupt (ADR 0015 §1).
///
/// Order, from ADR 0014 "Where it lives":
/// 1. take frames from the mailbox; a new DriveCommand goes to DriveLoop;
/// 2. `SafetySupervisor::step`, fed with the previous DriveLoop status and the pitch driver's
///    fault pin;
/// 3. `DriveLoop::step(armed)`;
/// 4. outside ARMED, `WheelMotor::stop(mode)` on both wheels; `PitchPowerStage::set_enabled`;
/// 5. queue Fault/Nack reports for the main loop;
/// 6. check in with the IWDG gate, and record execution time (hard rule 4).
///
/// Uses HAL interfaces only, so the identical code runs on the sim and on the G474.
/// \note Call from exactly one context (the motor-loop ISR). No allocation, no blocking.
class MotorLoop {
 public:
  MotorLoop(const hal::Clock& clock, DriveLoop& drive, SafetySupervisor& supervisor,
            SafetyMailbox& mailbox, ReportQueue& reports, CheckInMonitor& check_ins,
            hal::WheelMotor& left_motor, hal::WheelMotor& right_motor,
            hal::PitchPowerStage& pitch_stage);

  void tick();

  /// Outputs of the most recent tick.
  const SafetyOutputs& outputs() const { return outputs_; }
  /// Execution time of the last tick, us. Reported as LoopTiming(MOTOR_VELOCITY).
  uint32_t last_exec_us() const { return last_exec_us_; }
  uint32_t max_exec_us() const { return max_exec_us_; }
  /// Ticks whose execution exceeded kMotorLoopPeriod_us, since boot. Wraps at 2^32.
  uint32_t overrun_count() const { return overrun_count_; }

 private:
  const hal::Clock& clock_;
  DriveLoop& drive_;
  SafetySupervisor& supervisor_;
  SafetyMailbox& mailbox_;
  ReportQueue& reports_;
  CheckInMonitor& check_ins_;
  hal::WheelMotor& left_motor_;
  hal::WheelMotor& right_motor_;
  hal::PitchPowerStage& pitch_stage_;
  SafetyOutputs outputs_{};
  uint32_t last_exec_us_ = 0;
  uint32_t max_exec_us_ = 0;
  uint32_t overrun_count_ = 0;
};

}  // namespace recon::core
