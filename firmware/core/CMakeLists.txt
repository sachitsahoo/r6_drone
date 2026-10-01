# Portable control code. Must compile for both host and stm32, and must never
# include a vendor or STM32 header. CI enforces that (see .github/workflows/ci.yml).

add_library(recon_core STATIC
  time/timestamp.cpp
  protocol/cobs.cpp
  protocol/frame.cpp
)

# The generated protocol header must exist before anything here compiles.
add_dependencies(recon_core protocol_generated)

# core reaches hardware only through these interfaces.
target_link_libraries(recon_core PUBLIC recon_hal)

target_include_directories(recon_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${RECON_GENERATED_DIR}
)

target_compile_options(recon_core PRIVATE
  ${RECON_WARNINGS}
  ${RECON_FIRMWARE_FLAGS}
)
