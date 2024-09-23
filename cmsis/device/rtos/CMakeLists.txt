# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_include_directories(mbed-rtos
    INTERFACE
        include
)

target_sources(mbed-rtos
    INTERFACE
        source/mbed_boot.c
        source/mbed_rtos_rtx.c
        source/mbed_rtx_handlers.c
        source/mbed_rtx_idle.cpp
		source/mbed_boot_gcc_arm.c
)

target_compile_definitions(mbed-rtos
    INTERFACE
        MBED_CONF_RTOS_PRESENT=1
)
