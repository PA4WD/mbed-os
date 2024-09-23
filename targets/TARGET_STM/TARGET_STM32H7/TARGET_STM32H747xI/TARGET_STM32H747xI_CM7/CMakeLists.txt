# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(${MBED_TOOLCHAIN} STREQUAL "GCC_ARM")
    set(STARTUP_FILE startup_stm32h747xx.S)
    set(LINKER_FILE STM32H747xI_CM7.ld)
endif()

add_library(mbed-stm32h747xi-cm7 INTERFACE)

target_sources(mbed-stm32h747xi-cm7
    INTERFACE
        ${STARTUP_FILE}
)

target_include_directories(mbed-stm32h747xi-cm7
    INTERFACE
        .
)

mbed_set_linker_script(mbed-stm32h747xi-cm7 ${CMAKE_CURRENT_SOURCE_DIR}/${LINKER_FILE})

target_link_libraries(mbed-stm32h747xi-cm7 INTERFACE mbed-stm32h747xi)
