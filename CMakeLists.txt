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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_flexray C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_flexray
     src/main.c
     src/replay_frame.c
     src/flexray_frame.c
     src/panda_usb.c
     src/usb_descriptors.c
     src/flexray_bss_streamer.c
     src/flexray_fifo.c
     )

pico_set_program_name(pico_flexray "pico_flexray")
pico_set_program_version(pico_flexray "0.1")

# no_flash means the target is to run from RAM
# pico_set_binary_type(pico_flexray no_flash)


# Generate PIO header
pico_generate_pio_header(pico_flexray 
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_bss_streamer.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_replay_q8_frame.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_override_pipeline.pio
    )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_flexray 1)
# The panda interface is a vendor-specific USB device, not a CDC (serial) device.
# So we don't enable stdio_usb, but we will link the tinyusb_device library directly.
# pico_enable_stdio_usb(pico_flexray 1)

# Add the standard library to the build
target_link_libraries(pico_flexray
        hardware_spi
        hardware_gpio
        pico_stdlib
        tinyusb_device
        )

# Add the standard include files to the build
target_include_directories(pico_flexray PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add any user requested libraries
target_link_libraries(pico_flexray 
        hardware_dma
        hardware_pio
        hardware_interp
        pico_unique_id
        tinyusb_device
        )

pico_add_extra_outputs(pico_flexray)

