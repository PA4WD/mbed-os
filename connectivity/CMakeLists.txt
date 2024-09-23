# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# List of all connectivity libraries available.
add_library(mbed-emac INTERFACE)
add_library(mbed-lwipstack INTERFACE)
add_library(mbed-mbedtls INTERFACE)
add_library(mbed-netsocket INTERFACE)
add_library(mbed-ppp INTERFACE)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    # Add these subdirectories for tests
    add_subdirectory(drivers)
    add_subdirectory(libraries)
    add_subdirectory(mbedtls)
    add_subdirectory(netsocket)
else()
    # The directories below contain optional target libraries
    add_subdirectory(drivers EXCLUDE_FROM_ALL)
    add_subdirectory(libraries EXCLUDE_FROM_ALL)
    add_subdirectory(lwipstack EXCLUDE_FROM_ALL)
    add_subdirectory(mbedtls EXCLUDE_FROM_ALL)
    add_subdirectory(netsocket EXCLUDE_FROM_ALL)
endif()
