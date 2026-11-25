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

# Set the project name
set(CMAKE_PROJECT_NAME MainBoard)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# CubeMX may add adc.c to the application target. Add it if absent.
get_target_property(AR_APP_SOURCES ${CMAKE_PROJECT_NAME} SOURCES)
set(AR_ADC_SOURCE_PRESENT FALSE)
foreach(AR_SOURCE IN LISTS AR_APP_SOURCES)
    if(AR_SOURCE MATCHES "(^|/)Core/Src/adc[.]c$")
        set(AR_ADC_SOURCE_PRESENT TRUE)
    endif()
endforeach()
if(NOT AR_ADC_SOURCE_PRESENT)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Src/adc.c)
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/lcd.c
    Core/Src/lcd_graphics.c
    Core/Src/robotics_logo.c
    Core/Src/ir_sensor.c
    Core/Src/ir_calibration.c
    Core/Src/ar_nav.c
    Core/Src/ar_merged_app.c
    Core/Src/us_capture_core.c
    Core/Src/us_capture.c
    Core/Src/elcp_motor/elcp_motor.c
    Core/Src/encoder_calibration.c
    # Add user sources here
)

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

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
