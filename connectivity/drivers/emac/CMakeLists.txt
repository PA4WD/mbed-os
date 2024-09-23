# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if("STM" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_STM)
endif()

if("WHD" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(COMPONENT_WHD)
endif()

target_link_libraries(mbed-emac
    INTERFACE
        mbed-netsocket
)
