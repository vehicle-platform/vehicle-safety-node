# L0 host tests: Unity + ctest, native compiler, sanitizers from the host-tests preset.
include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(unity)

# vehicle_host_test(<name> [libraries...]): one Unity executable per test file.
function(vehicle_host_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE ${ARGN} vehicle_safety_logic unity)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

vehicle_host_test(test_mailbox_core)
vehicle_host_test(test_cornering_core)
vehicle_host_test(test_node_health_core)
vehicle_host_test(test_cornering vehicle_safety_fw vehicle_safety_hal_host)
vehicle_host_test(test_safety_app vehicle_safety_fw vehicle_safety_hal_host)
vehicle_host_test(test_mailbox vehicle_safety_fw vehicle_safety_hal_host)
vehicle_host_test(test_mailbox_timing vehicle_safety_fw vehicle_safety_hal_host)
vehicle_host_test(test_mailbox_stress vehicle_safety_fw vehicle_safety_hal_host)
find_package(Threads REQUIRED)
target_link_libraries(test_mailbox_stress PRIVATE Threads::Threads)

# Fault injection into the safety_app output paths (ISSUES E-29 (1)): the test builds its
# own copy of app/safety_app.c with the checked pack, the E2E protect and mailbox_offer()
# renamed to the test's fi_* seams. Its objects come first, so the library's safety_app.o is
# never pulled (no duplicate symbols); works with every linker, unlike -Wl,--wrap.
add_executable(test_safety_app_faults test_safety_app_faults.c ${PROJECT_SOURCE_DIR}/src/app/safety_app.c)
target_link_libraries(test_safety_app_faults PRIVATE vehicle_safety_fw vehicle_safety_hal_host vehicle_safety_logic unity)
target_compile_options(test_safety_app_faults PRIVATE -Wall -Wextra -Werror)
set_source_files_properties(${PROJECT_SOURCE_DIR}/src/app/safety_app.c TARGET_DIRECTORY test_safety_app_faults
    PROPERTIES COMPILE_OPTIONS "${VEHICLE_WARNINGS}" COMPILE_DEFINITIONS
    "platform_safety_warning_status_pack_checked=fi_warn_pack;platform_heartbeat_safety_pack_checked=fi_hbs_pack;platform_safety_warning_status_e2e_protect=fi_warn_protect;platform_heartbeat_safety_e2e_protect=fi_hbs_protect;mailbox_offer=fi_mailbox_offer")
add_test(NAME test_safety_app_faults COMMAND test_safety_app_faults)
