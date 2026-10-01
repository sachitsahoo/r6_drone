#pragma once

namespace recon::core {

/// Wheel speed references for one body command.
struct WheelSpeedRefs {
  float left_rad_s;
  float right_rad_s;
};

/// Differential-drive inverse kinematics with curvature-preserving saturation.
///
/// Design: ADR 0013, "Kinematics". Theory: docs/theory/wheel-velocity-loop.md §1.
///
///     v_L = v - w*b/2,   v_R = v + w*b/2,   wheel rad/s = rim m/s / r
///
/// If either wheel would exceed `max_wheel_speed_rad_s`, both are scaled by the same factor,
/// so the robot follows the commanded arc more slowly instead of a different arc.
///
/// \param linear_speed_m_s     Body forward speed, positive +x.
/// \param angular_rate_rad_s   Yaw rate, positive counter-clockwise about +z (turning left).
/// \param max_wheel_speed_rad_s Ceiling on |wheel speed|, >= 0.
/// \return Wheel speeds, positive = that wheel rolling the robot forward.
///
/// NaN inputs are treated as 0, so a corrupt command stops rather than drives.
WheelSpeedRefs diff_drive(float linear_speed_m_s, float angular_rate_rad_s,
                          float max_wheel_speed_rad_s);

}  // namespace recon::core
