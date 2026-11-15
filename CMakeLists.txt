cmake_minimum_required(VERSION 3.13)
project(uav_fcs_firmware VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(FCS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(FCS_POSIX_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/platform/posix")
set(FCS_TEST_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/tests")

set(FCS_CORE_SRC
    src/math/fcs_math.c
    src/estimation/attitude.c
    src/control/controller.c
    src/control/mixer.c
    src/core/hal.c
    src/core/params.c
    src/core/failsafe.c
    src/core/state_machine.c
    src/core/telemetry.c
    src/core/fcs.c
)

# ---- flight core: hardware independent, no heap, no OS calls ----
add_library(fcs_core STATIC ${FCS_CORE_SRC})
target_include_directories(fcs_core PUBLIC ${FCS_INCLUDE_DIR})
if(MSVC)
    target_compile_options(fcs_core PRIVATE /W4)
else()
    target_compile_options(fcs_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(UNIX)
    target_link_libraries(fcs_core PUBLIC m)
endif()

# ---- software in the loop simulator (host only) ----
add_executable(sil
    platform/posix/sim_dynamics.c
    platform/posix/hal_sim.c
    platform/posix/main_sil.c
)
target_include_directories(sil PRIVATE ${FCS_POSIX_DIR})
target_link_libraries(sil PRIVATE fcs_core)

# ---- unit tests ----
add_executable(unit_tests
    tests/test_main.c
    tests/test_math.c
    tests/test_attitude.c
    tests/test_control.c
    tests/test_mixer.c
    tests/test_safety.c
    tests/test_integration.c
)
target_include_directories(unit_tests PRIVATE ${FCS_TEST_DIR})
target_link_libraries(unit_tests PRIVATE fcs_core)

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
add_test(NAME sil_hover
         COMMAND sil --scenario hover --seconds 12 --out "${CMAKE_BINARY_DIR}/sil_hover.csv")
add_test(NAME sil_waypoint
         COMMAND sil --scenario waypoint --out "${CMAKE_BINARY_DIR}/sil_waypoint.csv")
add_test(NAME sil_failsafe
         COMMAND sil --scenario failsafe --out "${CMAKE_BINARY_DIR}/sil_failsafe.csv")
