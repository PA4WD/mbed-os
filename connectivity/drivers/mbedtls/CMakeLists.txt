# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_sources(mbed-mbedtls
    INTERFACE
        aes_alt.cpp
        aes_alt_stm32l4.c
        ccm_alt.cpp
        cryp_stm32.c
        gcm_alt.cpp
        hash_stm32.c
        md5_alt.cpp
        sha1_alt.cpp
        sha256_alt.cpp
)

target_include_directories(mbed-mbedtls
    INTERFACE
        .
)
