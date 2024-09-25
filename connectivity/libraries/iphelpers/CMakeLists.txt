# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_include_directories(mbed-core
    INTERFACE
        .
        ./include
)

target_sources(mbed-core
    INTERFACE
        source/common_functions.c
        source/ip6tos.c
		source/ip4tos.c
)
