# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if("CRYPTOCELL310" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(FEATURE_CRYPTOCELL310)
endif()

if("STM" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_STM)
endif()
