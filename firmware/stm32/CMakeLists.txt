# Firmware image for the Nucleo-G474RE. Target build only (-DTARGET=stm32).
#
# Vendor files (ADR 0016: CMSIS headers only) are fetched at configure time, pinned by tag and
# SHA-256, the way GoogleTest is on the host. SOURCE_SUBDIR points at a directory that does not
# exist, so FetchContent only downloads them; neither package's own build scripts run.

include(FetchContent)

FetchContent_Declare(cmsis_device_g4
  URL      https://github.com/STMicroelectronics/cmsis-device-g4/archive/refs/tags/v1.2.6.tar.gz
  URL_HASH SHA256=a150d4b334610d8d8445385e9104d08aa7ae8c4e443b2a7dc6f19c9711d8bb4c
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR do-not-build)
FetchContent_Declare(cmsis_core
  URL      https://github.com/ARM-software/CMSIS_6/archive/refs/tags/v6.3.0.tar.gz
  URL_HASH SHA256=331df74000876b8fb07933658cbf402c4d9f831b429258792d2c34ab5a6c5bf7
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR do-not-build)
FetchContent_MakeAvailable(cmsis_device_g4 cmsis_core)

set(RECON_DEVICE_DIR ${cmsis_device_g4_SOURCE_DIR})
set(RECON_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32g474re.ld)

add_executable(recon_firmware
  main.cpp
  stm_clock.cpp
  stm_watchdog.cpp
  stm_serial.cpp
  # ST's templates, used unmodified: reset handler + vector table, and SystemInit (FPU on).
  ${RECON_DEVICE_DIR}/Source/Templates/gcc/startup_stm32g474xx.s
  ${RECON_DEVICE_DIR}/Source/Templates/system_stm32g4xx.c
)
set_target_properties(recon_firmware PROPERTIES SUFFIX ".elf" LINK_DEPENDS ${RECON_LINKER_SCRIPT})

target_compile_definitions(recon_firmware PRIVATE STM32G474xx)
# SYSTEM: vendor headers are not held to this project's -Werror -Wpedantic.
target_include_directories(recon_firmware SYSTEM PRIVATE
  ${RECON_DEVICE_DIR}/Include
  ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include)
target_include_directories(recon_firmware PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(recon_firmware PRIVATE recon_core)

# Our C++ gets the project warnings and the firmware subset; ST's C and assembly do not.
# -fno-threadsafe-statics: main() builds two function-local statics exactly once, before any
# interrupt that could race it is enabled, so the guard (and libsupc++'s lock) is not needed.
foreach(flag ${RECON_WARNINGS} ${RECON_FIRMWARE_FLAGS} -fno-threadsafe-statics)
  target_compile_options(recon_firmware PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${flag}>)
endforeach()
target_compile_options(recon_firmware PRIVATE -ffunction-sections -fdata-sections)

# nano.specs: newlib-nano (only __libc_init_array, memcpy/memset and libm are used). nosys.specs: stub syscalls. No heap is provided (see the linker
# script), so anything that calls malloc fails to link rather than failing at run time.
target_link_options(recon_firmware PRIVATE
  -T${RECON_LINKER_SCRIPT}
  --specs=nano.specs
  --specs=nosys.specs
  # Our entry point is ST's Reset_Handler. newlib's crt0 (_start -> exit -> stdio cleanup) is
  # never reached, and leaving it out keeps its syscall stubs out of the link entirely.
  -nostartfiles
  -Wl,--gc-sections
  -Wl,-Map=$<TARGET_FILE_DIR:recon_firmware>/recon_firmware.map
  -Wl,--print-memory-usage
  # RAM is one rwx region holding .data, .bss and the stack: normal on a Cortex-M with no MPU
  # configured. The warning is about memory-protection hygiene on hosted systems.
  -Wl,--no-warn-rwx-segments)

add_custom_command(TARGET recon_firmware POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:recon_firmware>
          $<TARGET_FILE_DIR:recon_firmware>/recon_firmware.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:recon_firmware>
  COMMENT "recon_firmware: flash image and size"
  VERBATIM)
