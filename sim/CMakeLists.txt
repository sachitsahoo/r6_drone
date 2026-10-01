# Simulated HAL implementations. Host-only: pulled in by the root CMakeLists when TARGET=host.
#
# Built with the firmware language subset too. The sim links into the same test binaries as
# recon_core, and keeping it exception- and RTTI-free means nothing in it can quietly depend
# on a feature the firmware does not have.

add_library(recon_sim STATIC
  sim/sim_serial_link.cpp
  sim/wheel_plant.cpp
)

# Includes are written "sim/sim_clock.hpp", so the include root is this directory.
target_include_directories(recon_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(recon_sim PUBLIC recon_hal)

target_compile_options(recon_sim PRIVATE
  ${RECON_WARNINGS}
  ${RECON_FIRMWARE_FLAGS}
)
