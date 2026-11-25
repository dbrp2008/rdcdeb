cmake_minimum_required(VERSION 3.22)
project(MainBoardMotorTuner C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The IOC and CubeMX-managed Core/Drivers folders are at the project root.
# Hand-written code and build files live separately under App/ and buildsystem/.
set(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(HAL "${PROJECT_ROOT}/Drivers/STM32F4xx_HAL_Driver")
# CAN is CubeMX-managed alongside the rest of the root-level HAL package.
set(CMSIS "${PROJECT_ROOT}/Drivers/CMSIS")

set(CUBEMX_REQUIRED_FILES
    "${PROJECT_ROOT}/Core/Inc/main.h"
    "${PROJECT_ROOT}/Core/Inc/gpio.h"
    "${PROJECT_ROOT}/Core/Inc/dma.h"
    "${PROJECT_ROOT}/Core/Inc/spi.h"
    "${PROJECT_ROOT}/Core/Inc/usart.h"
    "${PROJECT_ROOT}/Core/Src/gpio.c"
    "${PROJECT_ROOT}/Core/Src/dma.c"
    "${PROJECT_ROOT}/Core/Src/spi.c"
    "${PROJECT_ROOT}/Core/Src/usart.c"
    "${PROJECT_ROOT}/Core/Src/can.c"
    "${PROJECT_ROOT}/Core/Src/stm32f4xx_hal_msp.c"
    "${HAL}/Src/stm32f4xx_hal.c"
    "${HAL}/Src/stm32f4xx_hal_can.c"
    "${CMSIS}/Include/core_cm4.h"
)
foreach(required_file IN LISTS CUBEMX_REQUIRED_FILES)
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR
            "Missing CubeMX output: ${required_file}. Open motortuner.ioc "
            "in STM32CubeMX and generate code in the tuner project root.")
    endif()
endforeach()

add_executable(MainBoardMotorTuner
    ${PROJECT_ROOT}/App/Core/Src/main.c
    ${PROJECT_ROOT}/App/Core/Src/motor_tuner.c
    ${PROJECT_ROOT}/App/Core/Src/rm_motor.c
    ${PROJECT_ROOT}/App/BoardSupport/lcd.c
    ${PROJECT_ROOT}/App/BoardSupport/lcd_graphics.c
    ${PROJECT_ROOT}/App/Startup/startup_stm32f405xx.s

    ${PROJECT_ROOT}/Core/Src/gpio.c
    ${PROJECT_ROOT}/Core/Src/dma.c
    ${PROJECT_ROOT}/Core/Src/spi.c
    ${PROJECT_ROOT}/Core/Src/usart.c
    ${PROJECT_ROOT}/Core/Src/can.c
    ${PROJECT_ROOT}/Core/Src/stm32f4xx_it.c
    ${PROJECT_ROOT}/Core/Src/stm32f4xx_hal_msp.c
    ${PROJECT_ROOT}/Core/Src/system_stm32f4xx.c
    ${PROJECT_ROOT}/Core/Src/sysmem.c
    ${PROJECT_ROOT}/Core/Src/syscalls.c

    ${HAL}/Src/stm32f4xx_hal.c
    ${HAL}/Src/stm32f4xx_hal_can.c
    ${HAL}/Src/stm32f4xx_hal_cortex.c
    ${HAL}/Src/stm32f4xx_hal_dma.c
    ${HAL}/Src/stm32f4xx_hal_dma_ex.c
    ${HAL}/Src/stm32f4xx_hal_exti.c
    ${HAL}/Src/stm32f4xx_hal_flash.c
    ${HAL}/Src/stm32f4xx_hal_flash_ex.c
    ${HAL}/Src/stm32f4xx_hal_flash_ramfunc.c
    ${HAL}/Src/stm32f4xx_hal_gpio.c
    ${HAL}/Src/stm32f4xx_hal_pwr.c
    ${HAL}/Src/stm32f4xx_hal_pwr_ex.c
    ${HAL}/Src/stm32f4xx_hal_rcc.c
    ${HAL}/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL}/Src/stm32f4xx_hal_spi.c
    ${HAL}/Src/stm32f4xx_hal_uart.c
)

target_include_directories(MainBoardMotorTuner PRIVATE
    ${PROJECT_ROOT}/App/Core/Inc
    ${PROJECT_ROOT}/App/BoardSupport
    ${PROJECT_ROOT}/App/BoardSupport/lcd
    ${PROJECT_ROOT}/Core/Inc
    ${HAL}/Inc
    ${HAL}/Inc/Legacy
    ${CMSIS}/Device/ST/STM32F4xx/Include
    ${CMSIS}/Include
)

target_compile_definitions(MainBoardMotorTuner PRIVATE
    USE_HAL_DRIVER
    STM32F405xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_link_libraries(MainBoardMotorTuner PRIVATE m)
set_target_properties(MainBoardMotorTuner PROPERTIES SUFFIX ".elf")
