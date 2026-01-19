# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(FS26-DAQ C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(FS26-DAQ 
    FS26-DAQ.c 
    gps.c      # <--- Check if this is named gps.c in your folder!
)

pico_set_program_name(FS26-DAQ "FS26-DAQ")
pico_set_program_version(FS26-DAQ "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(FS26-DAQ 0)
pico_enable_stdio_usb(FS26-DAQ 1)

# 2. Add pico_multicore here to fix the 'multicore_launch_core1' error
target_link_libraries(FS26-DAQ
        pico_stdlib
        pico_multicore
)

# Add the standard include files to the build
target_include_directories(FS26-DAQ PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(FS26-DAQ)

