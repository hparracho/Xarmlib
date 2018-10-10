INCLUDE(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION   1)

set(CMAKE_INSTALL_PREFIX "" CACHE STRING "" FORCE)

set(CMAKE_CXX_COMPILER ${COMPILER_PATH}arm-none-eabi-g++)

if(WIN32)
    set(CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}.exe)
endif()

set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Set a default build type if none was specified
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

## MCU architecture definition
## @TODO: automatically define accordingly to the specified target
set(mcu_flags "-mcpu=cortex-m0plus -mthumb")

## Link-time optimization flags
set(lto_flags "-flto -ffat-lto-objects")

## C/C++ common flags
set(common_flags    "${mcu_flags} -pedantic")
set(common_flags "${common_flags} -Wall")
set(common_flags "${common_flags} -Wextra")
set(common_flags "${common_flags} -fno-common")
set(common_flags "${common_flags} -fno-builtin")
set(common_flags "${common_flags} -ffunction-sections")
set(common_flags "${common_flags} -fdata-sections")
set(common_flags "${common_flags} -fmessage-length=0")
set(common_flags "${common_flags} -ffreestanding")

## C++ flags
set(cxx_flags "${common_flags} -std=c++17")
set(cxx_flags    "${cxx_flags} -fno-rtti")
set(cxx_flags    "${cxx_flags} -fno-exceptions")

set(CMAKE_CXX_FLAGS                "${cxx_flags}"              CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE        "${lto_flags} -Os -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_MINSIZEREL     "${lto_flags} -Os -DNDEBUG" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Og -g -DDEBUG"            CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG          "-O0 -g -DDEBUG"            CACHE STRING "" FORCE)

## Linker script path and filename
set(linker_script_path "${PROJECT_SOURCE_DIR}/../../ldscripts")
## @TODO: automatically define accordingly to the specified target
set(linker_script "lpc810.ld")

## Linker flags
set(linker_flags    "${mcu_flags} -s")
set(linker_flags "${linker_flags} -nostdlib")
set(linker_flags "${linker_flags} -Xlinker -Map=${project_name}.map")
set(linker_flags "${linker_flags} -Xlinker --gc-sections")
set(linker_flags "${linker_flags} -Xlinker -print-memory-usage")
set(linker_flags "${linker_flags} -L ${linker_script_path}")
set(linker_flags "${linker_flags} -T ${linker_script}")

set(CMAKE_EXE_LINKER_FLAGS                "${linker_flags}" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_RELEASE        "-flto -Os"       CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL     "-flto -Os"       CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO ""                CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_DEBUG          ""                CACHE STRING "" FORCE)
