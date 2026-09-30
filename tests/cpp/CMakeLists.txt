# Host-only C++ tests. Pulled in by the root CMakeLists only when TARGET=host.

include(FetchContent)

# Pinned rather than tracking a branch: a test framework that changes underneath
# the project turns an unrelated push into a red build.
FetchContent_Declare(googletest
  URL      https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
  URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(core_tests
  test_timestamp.cpp
)

target_link_libraries(core_tests PRIVATE recon_core GTest::gtest_main)

# Warnings-as-errors on our test code, but not on GoogleTest's own sources.
# -fno-exceptions is deliberately NOT applied here: GoogleTest is built with
# exceptions enabled, and its assertion macros work either way. The firmware
# language subset is enforced on recon_core, which is the code that ships.
target_compile_options(core_tests PRIVATE ${RECON_WARNINGS})

include(GoogleTest)
gtest_discover_tests(core_tests)
