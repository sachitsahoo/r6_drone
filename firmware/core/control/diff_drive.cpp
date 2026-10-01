#include "control/diff_drive.hpp"

#include <cmath>

#include "control/geometry.hpp"

namespace recon::core {

WheelSpeedRefs diff_drive(float linear_speed_m_s, float angular_rate_rad_s,
                          float max_wheel_speed_rad_s) {
  const float v = std::isnan(linear_speed_m_s) ? 0.0F : linear_speed_m_s;
  const float w = std::isnan(angular_rate_rad_s) ? 0.0F : angular_rate_rad_s;
  const float half_track_m = kTrackWidth_m / 2.0F;

  float left = (v - w * half_track_m) / kWheelRadius_m;
  float right = (v + w * half_track_m) / kWheelRadius_m;

  // One common factor for both wheels keeps left/right, and so the turning radius, fixed.
  const float largest = std::fmax(std::fabs(left), std::fabs(right));
  const float limit = std::fmax(0.0F, max_wheel_speed_rad_s);
  if (largest > limit) {
    const float scale = limit / largest;  // largest > limit >= 0, so no division by zero
    left *= scale;
    right *= scale;
  }
  return {left, right};
}

}  // namespace recon::core
