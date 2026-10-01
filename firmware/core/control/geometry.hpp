#pragma once

// Robot geometry the drive loop needs. Compile-time constants rather than parameters: they
// change only when the mechanical design does, and then the CAD changes with them.
//
// Both values are copies of cad/parameters.py, so tests/python/test_geometry_drift.py fails
// if either side changes without the other (tasks/lessons.md: copied figures drift silently).

namespace recon::core {

/// Wheel radius, m. 105 mm outer diameter (ADR 0008; cad/parameters.py WHEEL_OD).
inline constexpr float kWheelRadius_m = 0.0525F;

/// Distance between the two wheels' contact centres, m. The wheel centre planes sit at
/// +/-(CASING_LENGTH/2 + WHEEL_STANDOFF + WHEEL_WIDTH/2) = +/-(91 + 4 + 6) mm
/// (cad/parameters.py).
inline constexpr float kTrackWidth_m = 0.202F;

}  // namespace recon::core
