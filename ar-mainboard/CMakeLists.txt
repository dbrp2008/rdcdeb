cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Select an entire image, not a run-time drive mode. Keep the ELF names and
# source graphs disjoint; do not repurpose a configured bench tree for drive.
set(AR_IMAGE Drive CACHE STRING "AR firmware image (Drive or EncoderBench)")
set_property(CACHE AR_IMAGE PROPERTY STRINGS Drive EncoderBench)
if(AR_IMAGE STREQUAL "Drive")
    set(CMAKE_PROJECT_NAME MainBoard)
elseif(AR_IMAGE STREQUAL "EncoderBench")
    set(CMAKE_PROJECT_NAME EncoderBench)
else()
    message(FATAL_ERROR "Unknown AR_IMAGE='${AR_IMAGE}'. Choose Drive or EncoderBench.")
endif()
if(DEFINED AR_CONFIGURED_IMAGE AND NOT AR_IMAGE STREQUAL AR_CONFIGURED_IMAGE)
    message(FATAL_ERROR "This build directory is for ${AR_CONFIGURED_IMAGE}, not ${AR_IMAGE}. Use a separate build directory.")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# A normal configure (without --preset) uses this board's CubeMX toolchain.
# Respect a toolchain selected explicitly by the caller or environment.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND "$ENV{CMAKE_TOOLCHAIN_FILE}" STREQUAL "")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "STM32 toolchain")
endif()

# CubeMX names the Arm tools without a path. Process tasks and terminal builds
# do not inherit the CMake Tools extension's per-project environment. Give
# find_program the same ST bundle location as the other boards; keep caller-supplied
# -DCMAKE_PROGRAM_PATH entries first and leave the user's PATH untouched.
if(NOT "$ENV{CUBE_BUNDLE_PATH}" STREQUAL "")
    file(TO_CMAKE_PATH "$ENV{CUBE_BUNDLE_PATH}" _ar_bundle_root)
    set(_ar_gnu_bin "${_ar_bundle_root}/gnu-tools-for-stm32/14.3.1+st.2/bin")
    if(IS_DIRECTORY "${_ar_gnu_bin}")
        list(APPEND CMAKE_PROGRAM_PATH "${_ar_gnu_bin}")
    endif()
endif()

# This firmware is C plus startup assembly. A bare project() would also require g++.
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# The historic AR_ENCODER_BENCH had a powered wheel-test page. None of these
# old switches may silently select a different image in a reused cache.
foreach(retired IN ITEMS AR_IR_BENCH AR_ENABLE_F2 AR_ENABLE_F3 AR_F1_DRY_RUN AR_ENCODER_BENCH)
    if(${retired})
        message(FATAL_ERROR "${retired} is retired. Choose the intended Drive or passive EncoderBench image in a fresh build directory.")
    endif()
    unset(${retired} CACHE)
endforeach()

set(AR_CONFIGURED_IMAGE "${AR_IMAGE}" CACHE INTERNAL "Image first configured in this directory")
message(STATUS "AR image: ${AR_IMAGE}")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# The line IRs are digital GPIO inputs (Core/Inc/ir_mode.h): no ADC, no adc.c.

target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Src/lcd.c)
if(AR_IMAGE STREQUAL "EncoderBench")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/encoder_bench.c
        Core/Src/encoder_monitor.c
        Core/Src/enc_capture.c
    )
else()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/lcd_graphics.c
        Core/Src/robotics_logo.c
        Core/Src/ir_sensor.c
        Core/Src/side_ir.c
        Core/Src/ir_setup.c
        Core/Src/calibration_app.c
        Core/Src/drive_plan.c
        Core/Src/function1.c
        Core/Src/retrace.c
        Core/Src/us1_capture.c
        Core/Src/function2.c
        Core/Src/function3.c
        Core/Src/enc_watch.c
        Core/Src/wheel_spin.c
        Core/Src/us2_sensor.c
        Core/Src/elcp_motor/elcp_motor.c
        Core/Src/elcp_zero_pwm.c
        Core/Src/enc_capture.c
    )
    # Only the drive image chains its side-IR dispatcher to the reference motor callback.
    set_property(SOURCE Core/Src/elcp_motor/elcp_motor.c APPEND PROPERTY
        COMPILE_DEFINITIONS HAL_GPIO_EXTI_Callback=ELCP_EncoderEXTI_Callback)
endif()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Ninja runs post-build commands in a separate process. Resolve CubeMX's
# bare objcopy name now, rather than requiring the bundle on PATH at build time.
if(NOT IS_ABSOLUTE "${CMAKE_OBJCOPY}")
    get_filename_component(_ar_compiler_bin "${CMAKE_C_COMPILER}" DIRECTORY)
    find_program(_ar_objcopy NAMES "${CMAKE_OBJCOPY}" HINTS "${_ar_compiler_bin}" NO_CACHE REQUIRED)
    set(CMAKE_OBJCOPY "${_ar_objcopy}")
endif()

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND "${CMAKE_OBJCOPY}" -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    VERBATIM
)
