#Using GNU toolchain command line example in Windows
#cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCOMPILER_PATH="c:/GNU ARM Toolchain/bin/" ..

#Using NXP MCUXpresso toolchain command line example in Windows
#cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCOMPILER_PATH="c:/nxp/MCUXpressoIDE_10.2.0_759/ide/tools/bin/" ..

cmake_minimum_required(VERSION 3.0)

set(MCU "LPC810M021FN8")

#if(NOT DEFINED MCU)
#    message(FATAL_ERROR "\nFatal error: MCU not defined!!!\nAdd '-D MCU=<TARGET_MCU>' to the cmake command line with the target MCU part number.\nExample:\n    cmake -D MCU=LPC810M021FN8 [<OTHER_OPTIONS> ...]")
#endif()

add_definitions(-D ${MCU})

set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)

set(project_name LPC800-Mini-Kit-blinky)
project(${project_name} CXX)

include_directories("../../external/BITMASK/include")
include_directories("../../external/CMSIS/CMSIS/Core/include")
include_directories("../../external/SPAN/include")
include_directories("../../include/cpp")
include_directories("../../include")
include_directories("include")

file(GLOB xarmlib_source "../../source/api/*"
                         "../../source/core/*"
                         "../../source/cpp/*"
                         "../../source/targets/LPC81x/*")

file(GLOB_RECURSE user_source "source/*")

add_library(xarmlib OBJECT ${xarmlib_source})

add_executable(${project_name}.elf ${user_source} $<TARGET_OBJECTS:xarmlib> ${linker_script_path}/${linker_script})

set(hex_file ${project_name}.hex)
set(bin_file ${project_name}.bin)

add_custom_command(TARGET ${project_name}.elf POST_BUILD
    COMMENT "Building ${hex_file}"
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${project_name}.elf> ${hex_file}
    COMMENT "Building ${bin_file}"
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${project_name}.elf> ${bin_file})
