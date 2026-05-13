cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(PICO_FLEXRAY_SINGLE_TARGET "Build one Pico SDK board target instead of both UF2 variants" OFF)

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

if(NOT PICO_FLEXRAY_SINGLE_TARGET)
    project(flexray_signal_gen_superbuild NONE)
    include(ExternalProject)

    if(CMAKE_BUILD_TYPE)
        set(child_build_type ${CMAKE_BUILD_TYPE})
    else()
        set(child_build_type Release)
    endif()

    set(child_common_args
        -DPICO_FLEXRAY_SINGLE_TARGET=ON
        -DCMAKE_BUILD_TYPE=${child_build_type}
    )

    ExternalProject_Add(flexray_signal_gen_rp2040
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}
        BINARY_DIR ${CMAKE_BINARY_DIR}/rp2040
        BUILD_ALWAYS TRUE
        CMAKE_ARGS
            ${child_common_args}
            -DPICO_BOARD=pico
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR>
        INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_if_different
            <BINARY_DIR>/flexray_signal_gen.uf2
            ${CMAKE_BINARY_DIR}/flexray_signal_gen_rp2040.uf2
    )

    ExternalProject_Add(flexray_signal_gen_rp2350
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}
        BINARY_DIR ${CMAKE_BINARY_DIR}/rp2350
        BUILD_ALWAYS TRUE
        CMAKE_ARGS
            ${child_common_args}
            -DPICO_BOARD=pico2_w
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR>
        INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_if_different
            <BINARY_DIR>/flexray_signal_gen.uf2
            ${CMAKE_BINARY_DIR}/flexray_signal_gen_rp2350.uf2
    )

    add_custom_target(flexray_signal_gen_all_uf2 ALL
        DEPENDS flexray_signal_gen_rp2040 flexray_signal_gen_rp2350
    )

    add_custom_target(flexray_signal_gen_rp2040_uf2
        DEPENDS flexray_signal_gen_rp2040
    )

    add_custom_target(flexray_signal_gen_rp2350_uf2
        DEPENDS flexray_signal_gen_rp2350
    )

    return()
endif()

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
pico_set_program_version(flexray_signal_gen "0.2")

pico_generate_pio_header(flexray_signal_gen
    ${CMAKE_CURRENT_LIST_DIR}/signal_gen/flexray_signal_gen.pio
)

pico_enable_stdio_uart(flexray_signal_gen 1)

target_link_libraries(flexray_signal_gen
    hardware_gpio
    hardware_pwm
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
