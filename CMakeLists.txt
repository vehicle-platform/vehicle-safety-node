cmake_minimum_required(VERSION 3.20)

# vehicle-safety-node: the safety partition, the Cortex-M4 core of the STM32H755
# (prototype board NUCLEO-H745ZI-Q, D-072, D-094). Two build flavours (CMakePresets.json):
#   host-tests : native compiler, pure (HAL-free) logic + Unity tests under ctest
#   target-m4  : arm-none-eabi-gcc (cmake/arm-none-eabi.cmake), the same logic
#                cross-compiled for the Cortex-M4F. The CubeMX CM4 project, startup code,
#                linker script and MPU setup arrive with the board (red zone, D-072 item 4).
project(vehicle_safety_node VERSION 0.1.0 LANGUAGES C) # semver, tagged vX.Y.Z (D-036)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(VEHICLE_BUILD_HOST_TESTS "Build the Unity host tests (native toolchain only)" OFF)

# Only gen/c/safety/ of the pinned vehicle-defs tag is used (D-003).
set(VEHICLE_DEFS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/vehicle-defs)
set(VEHICLE_DEFS_GEN ${VEHICLE_DEFS_DIR}/gen/c/safety)
if(NOT EXISTS ${VEHICLE_DEFS_GEN}/platform_mailbox.h)
    message(FATAL_ERROR "Missing ${VEHICLE_DEFS_GEN}. Run: git submodule update --init")
endif()

set(VEHICLE_WARNINGS -Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion
                     -Wstrict-prototypes -Wmissing-prototypes -Wundef -pedantic)

add_subdirectory(src)

if(VEHICLE_BUILD_HOST_TESTS)
    if(CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR "VEHICLE_BUILD_HOST_TESTS needs the native toolchain (preset host-tests)")
    endif()
    enable_testing()
    add_subdirectory(tests/host)
endif()
