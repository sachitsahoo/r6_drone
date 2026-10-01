#include "runtime/motor_loop.hpp"

#include "time/timestamp.hpp"

namespace recon::core {

MotorLoop::MotorLoop(const hal::Clock& clock, DriveLoop& drive, SafetySupervisor& supervisor,
                     SafetyMailbox& mailbox, ReportQueue& reports, CheckInMonitor& check_ins,
                     hal::WheelMotor& left_motor, hal::WheelMotor& right_motor,
                     hal::PitchPowerStage& pitch_stage)
    : clock_(clock),
      drive_(drive),
      supervisor_(supervisor),
      mailbox_(mailbox),
      reports_(reports),
      check_ins_(check_ins),
      left_motor_(left_motor),
      right_motor_(right_motor),
      pitch_stage_(pitch_stage) {}

void MotorLoop::tick() {
  const uint32_t start_us = clock_.now_us();

  // 1. Frames posted by the main loop since the last tick.
  SafetyInputs in;
  in.now_us = start_us;
  mailbox_.take(in);
  if (in.drive_command) {
    drive_.set_command(in.drive_linear_m_s, in.drive_angular_rad_s);
  }

  // 2. Detectors: the previous DriveLoop step's status, and the pitch driver's fault pin.
  const WheelStatus& l = drive_.left();
  const WheelStatus& r = drive_.right();
  const float limit = drive_.config().gains.duty_limit;
  in.left = {l.duty, limit, l.speed_rad_s, l.speed_valid};
  in.right = {r.duty, limit, r.speed_rad_s, r.speed_valid};
  in.encoder_fault_left = l.encoder_fault;
  in.encoder_fault_right = r.encoder_fault;
  in.driver_fault = pitch_stage_.fault();
  outputs_ = supervisor_.step(in);

  // 3-4. Drive, or stop. Stop every tick outside ARMED: idempotent, and it means a motor
  // written by anything else is stopped again within 1 ms.
  drive_.step(outputs_.drive_armed);
  if (!outputs_.drive_armed) {
    left_motor_.stop(outputs_.wheel_stop);
    right_motor_.stop(outputs_.wheel_stop);
  }
  pitch_stage_.set_enabled(outputs_.pitch_enabled);

  // 5. Reports for the main loop to send. A full queue counts the loss (ReportQueue::dropped).
  for (size_t i = 0; i < outputs_.fault_count; ++i) {
    OutboundReport report;
    report.kind = OutboundReport::Kind::kFault;
    report.fault = outputs_.faults[i];
    reports_.push(report);
  }
  if (outputs_.nack.present) {
    OutboundReport report;
    report.kind = OutboundReport::Kind::kNack;
    report.nack = outputs_.nack;
    reports_.push(report);
  }

  // 6. Liveness and timing.
  check_ins_.check_in(CheckInTask::kMotorLoop);
  last_exec_us_ = elapsed_us(start_us, clock_.now_us());
  if (last_exec_us_ > max_exec_us_) {
    max_exec_us_ = last_exec_us_;
  }
  if (last_exec_us_ > kMotorLoopPeriod_us) {
    ++overrun_count_;
  }
}

}  // namespace recon::core
