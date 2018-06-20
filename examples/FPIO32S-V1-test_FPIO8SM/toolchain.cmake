INCLUDE(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_SYSTEM_VERSION   1)

set(CMAKE_INSTALL_PREFIX "" CACHE STRING "" FORCE)

set(CMAKE_CXX_COMPILER ${COMPILER_PATH}arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${COMPILER_PATH}arm-none-eabi-gcc)

if(WIN32)
    set(CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}.exe)
    set(CMAKE_ASM_COMPILER ${CMAKE_ASM_COMPILER}.exe)
endif()

set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_ASM_COMPILER_FORCED TRUE)

# Set a default build type if none was specified
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()

## MCU architecture definition
## @TODO: automatically define accordingly to the specified target
set(mcu_flags "-mcpu=cortex-m0plus -mthumb")

## C++ flags
set(cxx_flags "${mcu_flags} -std=c++17")
set(cxx_flags "${cxx_flags} -pedantic")
set(cxx_flags "${cxx_flags} -Wall")
set(cxx_flags "${cxx_flags} -Wextra")
set(cxx_flags "${cxx_flags} -fno-common")
set(cxx_flags "${cxx_flags} -fno-builtin")
set(cxx_flags "${cxx_flags} -ffunction-sections")
set(cxx_flags "${cxx_flags} -fdata-sections")
set(cxx_flags "${cxx_flags} -fmessage-length=0")
set(cxx_flags "${cxx_flags} -ffreestanding")
set(cxx_flags "${cxx_flags} -fno-rtti")
set(cxx_flags "${cxx_flags} -fno-exceptions")

set(CMAKE_CXX_FLAGS                "${cxx_flags}"   CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG          "-O0 -g -DDEBUG" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG"   CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE        "-Os -DNDEBUG"   CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Og -g -DDEBUG" CACHE STRING "" FORCE)

## Assembler flags
set(asm_flags "${mcu_flags} -x assembler-with-cpp")

set(CMAKE_ASM_FLAGS                "${asm_flags}"   CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_DEBUG          "-O0 -g -DDEBUG" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_MINSIZEREL     "-Os -DNDEBUG"   CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE        "-Os -DNDEBUG"   CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "-Og -g -DDEBUG" CACHE STRING "" FORCE)

## Linker script path and filename
set(linker_script_path "${PROJECT_SOURCE_DIR}/../../ldscripts")
## @TODO: automatically define accordingly to the specified target
set(linker_script "lpc812.ld")

## Linker flags
set(linker_flags    "${mcu_flags} -nostdlib")
set(linker_flags "${linker_flags} -Xlinker -Map=${project_name}.map")
set(linker_flags "${linker_flags} -Xlinker --gc-sections")
set(linker_flags "${linker_flags} -Xlinker -print-memory-usage")
set(linker_flags "${linker_flags} -L ${linker_script_path}")
set(linker_flags "${linker_flags} -T ${linker_script}")

set(CMAKE_EXE_LINKER_FLAGS "${linker_flags}")
