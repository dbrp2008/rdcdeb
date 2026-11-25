# Build only. The VS Code compound task may flash only after this exits 0.
cmake_minimum_required(VERSION 3.22)
get_filename_component(board "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
# Release keeps its original preset-free workflow. The opt-in task uses the
# actual configure AND build presets in the board folder (not the shell cwd).
if(NOT DEFINED AR_BUILD_PRESET)
    set(variant Release)
    set(output "${board}/build/Release")
    set(configure_args -S "${board}" -B "${output}" -G Ninja -DCMAKE_BUILD_TYPE=Release -DAR_IMAGE=Drive)
    set(build_args "${output}")
elseif(AR_BUILD_PRESET STREQUAL "EncoderBench")
    set(variant EncoderBench)
    set(output "${board}/build/EncoderBench")
    set(configure_args --preset EncoderBench)
    set(build_args --preset EncoderBench)
else()
    message(FATAL_ERROR "Unknown AR_BUILD_PRESET='${AR_BUILD_PRESET}'. Use EncoderBench, or omit it for Release.")
endif()
if(EXISTS "${output}/CMakeCache.txt")
    load_cache("${output}" READ_WITH_PREFIX saved_ CMAKE_PROGRAM_PATH CMAKE_C_COMPILER)
endif()

# If a custom compiler was discovered when building Release, a first bench
# build can reuse that hint without configuring or writing the Release tree.
if(variant STREQUAL "EncoderBench" AND EXISTS "${board}/build/Release/CMakeCache.txt")
    load_cache("${board}/build/Release" READ_WITH_PREFIX drive_ CMAKE_PROGRAM_PATH)
endif()

# Prefer the project's ST bundle version if present; try other installed
# versions newest first. Only look at known layouts, never scan an entire disk.
function(bundle_bins root out)
    file(TO_CMAKE_PATH "${root}" root)
    file(GLOB bins "${root}/gnu-tools-for-stm32/*/bin")
    list(SORT bins COMPARE NATURAL ORDER DESCENDING)
    set(${out} "${root}/gnu-tools-for-stm32/14.3.1+st.2/bin;${bins}" PARENT_SCOPE)
endfunction()

if(NOT DEFINED AR_GCC_BIN)
    set(AR_GCC_BIN "$ENV{AR_GCC_BIN}")
endif()
if(NOT "${AR_GCC_BIN}" STREQUAL "")
    # An explicit bin is authoritative. Do not silently build with a different install.
    file(TO_CMAKE_PATH "${AR_GCC_BIN}" candidates)
else()
    set(candidates ${saved_CMAKE_PROGRAM_PATH})
    if(IS_ABSOLUTE "${saved_CMAKE_C_COMPILER}")
        get_filename_component(cached_bin "${saved_CMAKE_C_COMPILER}" DIRECTORY)
        list(APPEND candidates "${cached_bin}")
    endif()
    list(APPEND candidates ${drive_CMAKE_PROGRAM_PATH})
    if(NOT "$ENV{CUBE_BUNDLE_PATH}" STREQUAL "")
        bundle_bins("$ENV{CUBE_BUNDLE_PATH}" bins)
        list(APPEND candidates ${bins})
    endif()
    file(TO_CMAKE_PATH "$ENV{PATH}" path_bins)
    list(APPEND candidates ${path_bins})
    if(NOT "$ENV{LOCALAPPDATA}" STREQUAL "")
        bundle_bins("$ENV{LOCALAPPDATA}/stm32cube/bundles" bins)
        list(APPEND candidates ${bins})
    endif()
    if(NOT "$ENV{SystemDrive}" STREQUAL "")
        file(TO_CMAKE_PATH "$ENV{SystemDrive}/ST" st)
        file(GLOB bins "${st}/STM32CubeCLT*/GNU-tools-for-STM32/bin"
            "${st}/STM32CubeIDE*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin"
            "${st}/STM32CubeIDE*/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin")
        list(SORT bins COMPARE NATURAL ORDER DESCENDING)
        list(APPEND candidates ${bins})
    endif()
endif()

if(CMAKE_HOST_WIN32)
    set(exe ".exe")
    set(separator ";")
else()
    set(exe "")
    set(separator ":")
endif()
list(REMOVE_DUPLICATES candidates)
foreach(dir IN LISTS candidates)
    if(IS_ABSOLUTE "${dir}" AND EXISTS "${dir}/arm-none-eabi-gcc${exe}" AND EXISTS "${dir}/arm-none-eabi-objcopy${exe}")
        set(bin "${dir}")
        break()
    endif()
endforeach()
if(NOT DEFINED bin)
    if(variant STREQUAL "EncoderBench")
        set(retry_preset "-DAR_BUILD_PRESET=EncoderBench ")
    endif()
    message(FATAL_ERROR "No complete Arm GNU bin directory found (AR_GCC_BIN='${AR_GCC_BIN}'). "
        "Install GNU Tools for STM32, or run cmake ${retry_preset}\"-DAR_GCC_BIN=C:/actual/toolchain/bin\" -P tools/build_release.cmake "
        "from firmware/ar-mainboard. That directory must contain arm-none-eabi-gcc and arm-none-eabi-objcopy. Nothing was flashed.")
endif()

# CubeMX's generated toolchain names gcc (for BOTH C and ASM) without a
# path. Give CMake, its try-compiles, Ninja and their children the same bin.
# This environment dies with this script; the user's PATH is never edited.
cmake_path(NATIVE_PATH bin native_bin)
set(ENV{PATH} "${native_bin}${separator}$ENV{PATH}")
set(gcc "${bin}/arm-none-eabi-gcc${exe}")
execute_process(COMMAND "${gcc}" -dumpmachine OUTPUT_VARIABLE target OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
if(NOT target STREQUAL "arm-none-eabi")
    message(FATAL_ERROR "Expected arm-none-eabi, but ${gcc} reports '${target}'. Check AR_GCC_BIN.")
endif()
message(STATUS "AR ${variant} toolchain: ${gcc} (${target})")
set(hints "${bin}" ${saved_CMAKE_PROGRAM_PATH})
list(REMOVE_DUPLICATES hints)
# Preserve the existing cache (including retired-mode guards), not --fresh.
execute_process(COMMAND "${CMAKE_COMMAND}" ${configure_args} "-DCMAKE_PROGRAM_PATH:STRING=${hints}"
    WORKING_DIRECTORY "${board}" COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND "${CMAKE_COMMAND}" --build ${build_args}
    WORKING_DIRECTORY "${board}" COMMAND_ERROR_IS_FATAL ANY)
