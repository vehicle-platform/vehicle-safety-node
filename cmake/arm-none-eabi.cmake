# Toolchain file: arm-none-eabi-gcc for the STM32H755 Cortex-M4 core (D-072, D-007).
# The CM4 of the STM32H745/H755 (D-094): single-precision FPU. The board parts (CubeMX HAL,
# startup, linker script for flash bank 2 and D2 SRAM) come with the board (red zone).
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
