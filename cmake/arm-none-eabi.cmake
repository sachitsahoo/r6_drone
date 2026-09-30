# Toolchain file for the STM32 Nucleo-G474RE target.
#
# Selected automatically by the root CMakeLists.txt when -DTARGET=stm32.
# Do not include this from a host build.

set(CMAKE_SYSTEM_NAME      Generic)   # "Generic" == bare metal, no OS
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy CACHE FILEPATH "")
set(CMAKE_SIZE         arm-none-eabi-size    CACHE FILEPATH "")

# CMake's compiler sanity check builds and *links* a test executable by default.
# On bare metal there is no startup code or linker script yet, so linking fails and
# configuration aborts. Restricting the check to a static library verifies the
# compiler works without requiring a linkable image.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# STM32G474RE core: Arm Cortex-M4 with a single-precision FPU (FPv4-SP-D16).
# Source: STM32G474xE datasheet (DS12288), section 3.3 "ARM Cortex-M4 core with FPU".
# -mfloat-abi=hard uses FPU registers for float arguments; this must match every
# object linked into the image, including any vendor or precompiled library.
set(RECON_ARCH_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT   "${RECON_ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${RECON_ARCH_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${RECON_ARCH_FLAGS}")

# Never search the host's system paths for target libraries or headers.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BEFORE)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
