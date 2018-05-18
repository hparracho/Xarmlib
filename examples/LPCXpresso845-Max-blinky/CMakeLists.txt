#Using GNU toolchain command line example in Windows
#cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCOMPILER_PATH="c:/GNU ARM Toolchain/bin/" ..

#Using NXP MCUXpresso toolchain command line example in Windows
#cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCOMPILER_PATH="c:/nxp/MCUXpressoIDE_10.1.1_606/ide/tools/bin/" ..

cmake_minimum_required(VERSION 3.0)

set(MCU "LPC845M301JBD64")

#if(NOT DEFINED MCU)
#    message(FATAL_ERROR "\nFatal error: MCU not defined!!!\nAdd '-D MCU=<TARGET_MCU>' to the cmake command line with the target MCU part number.\nExample:\n    cmake -D MCU=LPC845M301JBD64 [<OTHER_OPTIONS> ...]")
#endif()

add_definitions(-D ${MCU})

set(CMAKE_TOOLCHAIN_FILE toolchain.cmake)

set(project_name LPCXpresso845-Max-blinky)
project(${project_name} CXX ASM)

include_directories("../../external/BITMASK/include")
include_directories("../../external/CMSIS/Core/include")
include_directories("../../external/GSL/include")
include_directories("../../include")
include_directories("include")

file(GLOB_RECURSE xarmlib_source "../../source/system/*.cpp"
                                 "../../source/targets/LPC84x/*.cpp"
                                 "../../source/targets/LPC84x/*.s")
file(GLOB_RECURSE user_source    "source/*.cpp")

add_library(xarmlib OBJECT ${xarmlib_source})

add_executable(${project_name}.elf ${user_source} $<TARGET_OBJECTS:xarmlib> ${linker_script_path}/${linker_script})

set(hex_file ${project_name}.hex)
set(bin_file ${project_name}.bin)

add_custom_command(TARGET ${project_name}.elf POST_BUILD
    COMMENT "Building ${hex_file}"
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${project_name}.elf> ${hex_file}
    COMMENT "Building ${bin_file}"
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${project_name}.elf> ${bin_file})
