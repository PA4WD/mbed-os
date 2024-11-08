# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    if(BUILD_GREENTEA_TESTS)
        # add greentea test
    else()
        add_subdirectory(COMPONENT_QSPIF)
        add_subdirectory(tests/UNITTESTS)
    endif()
endif()

if("QSPIF" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(COMPONENT_QSPIF)
endif()

target_include_directories(mbed-storage-blockdevice
    INTERFACE
        .
        ./include
        ./include/blockdevice
        ./include/blockdevice/internal
)

target_sources(mbed-storage-blockdevice
    INTERFACE
        source/BufferedBlockDevice.cpp
        source/ChainingBlockDevice.cpp
        source/ExhaustibleBlockDevice.cpp
        source/FlashSimBlockDevice.cpp
        source/HeapBlockDevice.cpp
        source/MBRBlockDevice.cpp
        source/ObservingBlockDevice.cpp
        source/ProfilingBlockDevice.cpp
        source/ReadOnlyBlockDevice.cpp
        source/SFDP.cpp
        source/SlicingBlockDevice.cpp
)
