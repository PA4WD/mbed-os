# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_link_libraries(mbed-emac
    INTERFACE
        mbed-netsocket
)

target_include_directories(mbed-emac
    INTERFACE
        .
		./lan8742
)

target_sources(mbed-emac
    INTERFACE
        stm32xx_emac.cpp
        stm32xx_eth_irq_callback.cpp
		lan8742/lan8742.c
)
