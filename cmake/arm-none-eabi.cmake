# Toolchain file: arm-none-eabi-gcc for the STM32G474RE (Cortex-M4F, 170 MHz, FDCAN; D-118,
# D-007). Single-precision FPU (fpv4-sp-d16, hard float ABI). The board parts (CubeMX HAL,
# startup, linker script, NUCLEO-G474RE; fallback G431RB) come with the board (red zone).
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Honour a toolchain outside PATH, e.g. -DARM_TOOLCHAIN_DIR=~/.platformio/packages/toolchain-gccarmnoneeabi/bin
set(ARM_TOOLCHAIN_DIR "" CACHE PATH "Directory holding arm-none-eabi-gcc (empty = PATH)")
if(ARM_TOOLCHAIN_DIR)
    set(_prefix "${ARM_TOOLCHAIN_DIR}/arm-none-eabi-")
else()
    set(_prefix "arm-none-eabi-")
endif()

set(CMAKE_C_COMPILER   ${_prefix}gcc)
set(CMAKE_ASM_COMPILER ${_prefix}gcc)
set(CMAKE_AR           ${_prefix}ar)
set(CMAKE_OBJCOPY      ${_prefix}objcopy)
set(CMAKE_SIZE         ${_prefix}size)

# No executable can link without the board's linker script yet: test the compiler
# with a static library instead.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(VEHICLE_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${VEHICLE_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_ASM_FLAGS_INIT "${VEHICLE_CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nano.specs -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
