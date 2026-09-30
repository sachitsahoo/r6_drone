# Portable control code. Must compile for both host and stm32, and must never
# include a vendor or STM32 header. CI enforces that (see .github/workflows/ci.yml).

add_library(recon_core STATIC
  time/timestamp.cpp
)

target_include_directories(recon_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(recon_core PRIVATE
  ${RECON_WARNINGS}
  ${RECON_FIRMWARE_FLAGS}
)
