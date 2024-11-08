# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_sources(mbed-storage
    INTERFACE
        source/PlatformStorage.cpp
)

list(APPEND mbed-storage-libs
    mbed-storage-blockdevice
    mbed-storage-filesystem
    mbed-storage-fat
    # mbed-storage-littlefs
)

if("QSPIF" IN_LIST MBED_TARGET_LABELS)
    list(APPEND mbed-storage-libs mbed-storage-qspif)
endif()

target_link_libraries(mbed-storage
    INTERFACE
        ${mbed-storage-libs}
)
