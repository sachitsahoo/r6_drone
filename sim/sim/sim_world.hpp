#pragma once

#include <cstdint>

#include "sim/sim_clock.hpp"
#include "sim/sim_serial_link.hpp"
#include "sim/static_sensors.hpp"
#include "sim/wheel_plant.hpp"

namespace recon::sim {

/// Every simulated HAL object for one robot, stepped together.
///
/// A test hands the `hal::` views of these members to `core`, then calls `step()` in a loop.
/// Statically allocatable: no heap, so the same world could later run on a target board for
/// processor-in-the-loop tests.
class SimWorld {
 public:
  struct Config {
    uint32_t start_us = 0;
    SimSerialLink::Config link{};
    WheelPlantParams left{};
    WheelPlantParams right{};
  };

  SimWorld() : SimWorld(Config{}) {}
  explicit SimWorld(const Config& config)
      : clock(config.start_us),
        link(config.link),
        left_wheel(config.left),
        right_wheel(config.right),
        imu(clock),
        pitch_encoder(clock),
        power(clock),
        pitch_stage(clock) {}

  /// Advances virtual time by `dt_us`, then the link and the plants over that interval.
  void step(uint32_t dt_us) {
    constexpr double kSPerUs = 1e-6;
    clock.advance_us(dt_us);
    link.step(dt_us);
    left_wheel.step(dt_us * kSPerUs);
    right_wheel.step(dt_us * kSPerUs);
  }

  SimClock clock;
  SimSerialLink link;
  SimWheel left_wheel;
  SimWheel right_wheel;
  SimImu imu;
  SimAbsoluteEncoder pitch_encoder;
  SimPowerMonitor power;
  SimPitchPowerStage pitch_stage;
};

}  // namespace recon::sim
