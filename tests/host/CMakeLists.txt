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
