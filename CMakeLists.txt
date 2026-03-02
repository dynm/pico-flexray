cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(flexray_signal_gen C CXX ASM)

pico_sdk_init()

# ---------- FlexRay Signal Generator ----------

add_executable(flexray_signal_gen
    signal_gen/main.c
    signal_gen/flexray_signal_gen.c
    signal_gen/flexray_frame.c
    signal_gen/usb_descriptors.c
)

pico_set_program_name(flexray_signal_gen "flexray_signal_gen")
pico_set_program_version(flexray_signal_gen "0.1")

pico_generate_pio_header(flexray_signal_gen
    ${CMAKE_CURRENT_LIST_DIR}/signal_gen/flexray_signal_gen.pio
)

pico_enable_stdio_uart(flexray_signal_gen 1)

target_link_libraries(flexray_signal_gen
    hardware_gpio
    pico_stdlib
    tinyusb_device
    hardware_dma
    hardware_pio
    pico_unique_id
)

target_include_directories(flexray_signal_gen PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/signal_gen
)

target_compile_options(flexray_signal_gen PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra -Wstrict-prototypes -Werror>
)

pico_add_extra_outputs(flexray_signal_gen)
