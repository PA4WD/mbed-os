# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    if(BUILD_GREENTEA_TESTS)
        # add greentea test
    else()
        add_subdirectory(tests/UNITTESTS)
    endif()
endif()

add_subdirectory(tdbstore)
add_subdirectory(filesystemstore)

target_include_directories(mbed-storage-kvstore
    INTERFACE
        .
        include
        include/kvstore
)

if("QSPIF" IN_LIST MBED_TARGET_LABELS)
    list(APPEND mbed_blockdevice_libs mbed-storage-qspif)
endif()

target_link_libraries(mbed-storage-kvstore
    INTERFACE
        mbed-storage-kv-global-api
        ${mbed_blockdevice_libs}
)
