cmake_minimum_required(VERSION 3.20)

# -----------------------------------------------------------------------------
# Target selection
#
# Usage:
#   cmake -B build       -DTARGET=host
#   cmake -B build-stm32 -DTARGET=stm32
#
# CAUTION: "TARGET" is a reserved operator in CMake's if() command -- if(TARGET x)
# tests whether a CMake target named x exists. Writing if(TARGET STREQUAL "host")
# is parsed as that operator and silently does the wrong thing. Every test below
# therefore dereferences and quotes the variable: if("${TARGET}" STREQUAL "host").
# The name is fixed by CLAUDE.md, so the workaround lives here rather than a rename.
# -----------------------------------------------------------------------------
set(TARGET "host" CACHE STRING "Build target: host or stm32")
set_property(CACHE TARGET PROPERTY STRINGS host stm32)

if(NOT "${TARGET}" STREQUAL "host" AND NOT "${TARGET}" STREQUAL "stm32")
  message(FATAL_ERROR "TARGET must be 'host' or 'stm32', got '${TARGET}'")
endif()

# The toolchain file must be set before project() so that CMake's compiler
# detection uses the cross compiler rather than the host one.
if("${TARGET}" STREQUAL "stm32" AND NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake")
endif()

project(ReconUGV
  VERSION   0.1.0
  LANGUAGES CXX
  DESCRIPTION "Pitch-stabilized two-wheeled reconnaissance UGV")

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)   # ISO C++17, not GNU++17

# Warnings-as-errors applies to this project's own targets only, never to
# fetched dependencies such as GoogleTest.
set(RECON_WARNINGS -Wall -Wextra -Wpedantic -Werror)

# Firmware language subset. Applied to firmware targets on BOTH host and stm32
# builds, so host tests exercise the same language subset that runs on the MCU.
# See CLAUDE.md "Hard rules for firmware" and docs/decisions/0001.
set(RECON_FIRMWARE_FLAGS -fno-exceptions -fno-rtti)

message(STATUS "Recon UGV: TARGET=${TARGET}, compiler=${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# -----------------------------------------------------------------------------
# Protocol code generation
#
# protocol/schema/*.yaml is the single source of truth; messages.hpp and messages.py are
# generated from it and gitignored (docs/decisions/0003). The generator is a build-time
# host tool -- it never runs on the robot -- but it IS a hard build dependency: a clone
# with no Python and no PyYAML cannot build the firmware.
#
# Output goes into the source tree rather than the build tree because both the C++ and the
# Python consumers need one stable path, and pyproject.toml puts protocol/generated on
# pythonpath. Known limitation: two build trees generating concurrently would race. They
# are built sequentially here.
# -----------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Fail at configure time with an actionable message rather than mid-build with an
# ImportError. The system interpreter usually lacks PyYAML; the project venv has it.
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import yaml"
  RESULT_VARIABLE RECON_YAML_CHECK
  OUTPUT_QUIET ERROR_QUIET)
if(NOT RECON_YAML_CHECK EQUAL 0)
  message(FATAL_ERROR
    "The protocol generator needs PyYAML, and ${Python3_EXECUTABLE} does not have it.\n"
    "  Install it:        python3 -m pip install -r requirements-dev.txt\n"
    "  Or point CMake at an interpreter that has it:\n"
    "                     cmake -B build -DTARGET=host -DPython3_EXECUTABLE=$PWD/.venv/bin/python")
endif()

set(RECON_SCHEMA_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/protocol/schema")
set(RECON_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protocol/generated")
set(RECON_GENERATED_FILES
  "${RECON_GENERATED_DIR}/messages.hpp"
  "${RECON_GENERATED_DIR}/messages.py")

add_custom_command(
  OUTPUT ${RECON_GENERATED_FILES}
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/protocol/generate.py"
          --out-dir "${RECON_GENERATED_DIR}"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/protocol/generate.py"
          "${CMAKE_CURRENT_SOURCE_DIR}/protocol/schema_loader.py"
          "${RECON_SCHEMA_DIR}/messages.yaml"
          "${RECON_SCHEMA_DIR}/params.yaml"
  COMMENT "Generating protocol bindings from schema"
  VERBATIM)

add_custom_target(protocol_generated DEPENDS ${RECON_GENERATED_FILES})

add_subdirectory(firmware/hal)
add_subdirectory(firmware/core)

if("${TARGET}" STREQUAL "host")
  enable_testing()
  add_subdirectory(tests/cpp)
else()
  # The target build compiles firmware/core and the firmware/hal interfaces to prove
  # portability. It does not yet
  # link a flashable image: startup code, linker script, and firmware/stm32/ do
  # not exist (Phase 1).
  message(STATUS "Recon UGV: target build compiles firmware/core only; no image is linked yet")
endif()
