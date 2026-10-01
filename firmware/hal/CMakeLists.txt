# Abstract hardware interfaces. Header-only: implementations live in firmware/stm32/ and sim/.
# Includes are written "hal/clock.hpp", so the include root is this directory.

add_library(recon_hal INTERFACE)
target_include_directories(recon_hal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# Proves the headers build on both targets under the firmware language subset, and enforces
# the design rules with static_asserts. See compile_check.cpp.
add_library(recon_hal_compile_check OBJECT compile_check.cpp)
target_link_libraries(recon_hal_compile_check PRIVATE recon_hal)
target_compile_options(recon_hal_compile_check PRIVATE ${RECON_WARNINGS} ${RECON_FIRMWARE_FLAGS})
