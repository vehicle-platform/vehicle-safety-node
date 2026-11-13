# Generated vehicle-defs code for node SAFETY (D-003): platform.dbc pack/unpack and the
# E2E protect/check functions (D-005, D-026). Built with its own flags.
add_library(vehicle_defs_gen STATIC
    ${VEHICLE_DEFS_GEN}/platform.c
    ${VEHICLE_DEFS_GEN}/platform_e2e.c
    ${VEHICLE_DEFS_GEN}/vehicle_e2e.c
)
target_include_directories(vehicle_defs_gen PUBLIC ${VEHICLE_DEFS_GEN})

# Pure, HAL-free logic: unit-tested on the host and cross-compiled for the Cortex-M4.
add_library(vehicle_safety_logic STATIC
    services/mailbox_core.c
    features/cornering/cornering_core.c
    features/indicator/indicator_core.c
    app/node_health_core.c
)
target_include_directories(vehicle_safety_logic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vehicle_safety_logic PUBLIC vehicle_defs_gen m) # m: sinf/cosf (D-041)
target_compile_options(vehicle_safety_logic PRIVATE ${VEHICLE_WARNINGS})

# Firmware code above the HAL interfaces (hal/hsem.h): the same objects serve the host
# tests and the M4; each platform links its own hal/ implementation.
add_library(vehicle_safety_fw STATIC
    services/mailbox.c
    features/cornering/cornering.c
    features/indicator/indicator.c
    app/safety_app.c
)
target_link_libraries(vehicle_safety_fw PUBLIC vehicle_safety_logic)
target_compile_options(vehicle_safety_fw PRIVATE ${VEHICLE_WARNINGS})

if(NOT CMAKE_CROSSCOMPILING)
    # Host models of the HAL interfaces (tests only, never in the M4 image).
    add_library(vehicle_safety_hal_host STATIC
        hal/host/hsem_host.c
        hal/host/indicator_gpio_host.c
    )
    target_include_directories(vehicle_safety_hal_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(vehicle_safety_hal_host PRIVATE -Wall -Wextra -Werror)
endif()
