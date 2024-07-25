/* Copyright (c) 2017-2019 ARM Limited
 * Copyright (c) 2017-2019 STMicroelectronics
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if DEVICE_EMAC

#include <stdlib.h>

#include "cmsis_os.h"

#include "mbed_interface.h"
#include "mbed_assert.h"
#include "events/mbed_shared_queues.h"
#include "netsocket/nsapi_types.h"
#include "platform/mbed_power_mgmt.h"
#include "platform/mbed_error.h"

//#include "stm32xx_emac_config.h"
#include "stm32xx_emac.h"

#include "mbed-trace/mbed_trace.h"

#include "lan8742/lan8742.h"
#include "lwip/memp.h"
#include "lwip/api.h"

#define TRACE_GROUP "STE2"

/* mbed trace feature is supported */
/* ex in mbed_app.json */
/*   "mbed-trace.enable": "1" */

/* mbed_trace: debug traces (tr_debug) can be disabled here with no change in mbed_app.json */
// #undef TRACE_LEVEL_DEBUG
// #define TRACE_LEVEL_DEBUG 0

/* To get trace from every packet, enable deep trace macro */
// #define STM32xx_DEEP_TRACE
#ifdef STM32xx_DEEP_TRACE
#define tr_debug_deep(...) tr_debug(__VA_ARGS__)
#else
#define tr_debug_deep(...)
#endif

using namespace std::chrono;

/* \brief Flags for worker thread */
#define FLAG_RX                 1

/** \brief  Driver thread priority */
#define THREAD_PRIORITY         (osPriorityHigh)

#define PHY_TASK_PERIOD      200ms

#define STM_HWADDR_SIZE         (6)
#define STM_ETH_MTU_SIZE        1500
#define STM_ETH_IF_NAME         "st"


ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

static lan8742_Object_t LAN8742;

static int32_t ETH_PHY_IO_Init(void);
static int32_t ETH_PHY_IO_DeInit(void);
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
static int32_t ETH_PHY_IO_GetTick(void);

static lan8742_IOCtx_t LAN8742_IOCtx = {
    ETH_PHY_IO_Init,
    ETH_PHY_IO_DeInit,
    ETH_PHY_IO_WriteReg,
    ETH_PHY_IO_ReadReg,
    ETH_PHY_IO_GetTick
};

static ETH_TxPacketConfig TxConfig;


MBED_WEAK uint8_t mbed_otp_mac_address(char *mac);
void mbed_default_mac_address(char *mac);

#ifdef __cplusplus
extern "C" {
#endif

void _eth_config_mac(ETH_HandleTypeDef *heth);
void ETH_IRQHandler(void);
MBED_WEAK void STM_HAL_ETH_Handler(ETH_HandleTypeDef *heth);

#ifdef __cplusplus
}
#endif

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    /* Configure the MPU attributes as Device not cacheable
       for ETH DMA descriptors */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Configure the MPU attributes as Cacheable write through
       for LwIP RAM heap which contains the Tx buffers */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30044000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Enable the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * IRQ Handler
 *
 * @param  heth: ETH handle
 * @retval None
 */
MBED_WEAK void STM_HAL_ETH_Handler()
{
}

/**
 * Ethernet IRQ Handler
 *
 * @param  None
 * @retval None
 */
void ETH_IRQHandler(void)
{
    STM_HAL_ETH_Handler();
}

STM32_EMAC::STM32_EMAC()
    : thread(0)
    , phy_status(0)
{
}

static osThreadId_t create_new_thread(const char *threadName, void (*thread)(void *arg), void *arg, int stacksize, osPriority_t priority, mbed_rtos_storage_thread_t *thread_cb)
{
    osThreadAttr_t attr = {0};
    attr.name = threadName;
    attr.stack_mem  = malloc(stacksize);
    attr.cb_mem  = thread_cb;
    attr.stack_size = stacksize;
    attr.cb_size = sizeof(mbed_rtos_storage_thread_t);
    attr.priority = priority;
    return osThreadNew(thread, arg, &attr);
}

/**
 * In this function, the hardware should be initialized.
 */
bool STM32_EMAC::low_level_init_successful()
{
    uint32_t idx;

    MPU_Config();

    /* Init ETH */
    uint8_t MACAddr[6];
    EthHandle.Instance = ETH;
#if (MBED_MAC_ADDRESS_SUM != MBED_MAC_ADDR_INTERFACE)
    MACAddr[0] = MBED_MAC_ADDR_0;
    MACAddr[1] = MBED_MAC_ADDR_1;
    MACAddr[2] = MBED_MAC_ADDR_2;
    MACAddr[3] = MBED_MAC_ADDR_3;
    MACAddr[4] = MBED_MAC_ADDR_4;
    MACAddr[5] = MBED_MAC_ADDR_5;
#else
    mbed_mac_address((char *)MACAddr);
#endif
    EthHandle.Init.MACAddr = &MACAddr[0];
    EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    EthHandle.Init.RxDesc = DMARxDscrTab;
    EthHandle.Init.TxDesc = DMATxDscrTab;
    EthHandle.Init.RxBuffLen = 1524;

    tr_debug("MAC Addr %02x:%02x:%02x:%02x:%02x:%02x", MACAddr[0], MACAddr[1], MACAddr[2], MACAddr[3], MACAddr[4], MACAddr[5]);    
    tr_info("ETH buffers - Rx:%u size:%u - Tx:%u size:%u - Rx buff size:%u ", ETH_RX_DESC_CNT, sizeof(DMARxDscrTab), ETH_TX_DESC_CNT, sizeof(DMATxDscrTab), sizeof(Rx_Buff));

    if (HAL_ETH_Init(&EthHandle) != HAL_OK) {
        return false;
    }

    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    for (idx = 0; idx < ETH_RX_DESC_CNT; idx++) {
        HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
    }

    tr_info("low_level_init_successful");

    /* Set PHY IO functions */
    LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);

    /* Initialize the LAN8742 ETH PHY */
    return LAN8742_Init(&LAN8742) == LAN8742_STATUS_OK;
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the memory buffer chain that is passed to the function.
 *
 * @param buf the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return true if the packet could be sent
 *         false value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
bool STM32_EMAC::link_out(emac_mem_buf_t *buf)
{
    bool success = false;
    uint32_t i = 0;
    struct pbuf *q;
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
    HAL_StatusTypeDef status;
    struct pbuf *p = NULL;
    p = (struct pbuf *)buf;
    /* Get exclusive access */
    TXLockMutex.lock();

    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));

    /* copy frame from pbufs to driver buffers */
    for (q = p; q != NULL; q = q->next) {
        if (i >= ETH_TX_DESC_CNT) {
            tr_error("Error : ETH_TX_DESC_CNT not sufficient");
            goto error;
        }

        Txbuffer[i].buffer = (uint8_t *)q->payload;
        Txbuffer[i].len = q->len;

        if (i > 0) {
            Txbuffer[i - 1].next = &Txbuffer[i];
        }

        if (q->next == NULL) {
            Txbuffer[i].next = NULL;
        }

        i++;
    }

    TxConfig.Length = p->tot_len;
    TxConfig.TxBuffer = Txbuffer;

    status = HAL_ETH_Transmit(&EthHandle, &TxConfig, 50);
    if (status == HAL_OK) {
        success = 1;
    } else {
        tr_error("Error returned by HAL_ETH_Transmit (%d)", status);
        success = 0;
    }

error:

    if (p->ref > 1) {
        pbuf_free(p);
    }

    /* Restore access */
    TXLockMutex.unlock();

    return success;
}

/**
 * Should allocate a contiguous memory buffer and transfer the bytes of the incoming
 * packet to the buffer.
 *
 * @param buf If a frame was received and the memory buffer allocation was successful, a memory
 *            buffer filled with the received packet (including MAC header)
 * @return negative value when no more frames,
 *         zero when frame is received
 */
int STM32_EMAC::low_level_input(emac_mem_buf_t **buf)
{
    ETH_BufferTypeDef RxBuff;
    uint32_t frameLength = 0;

    if (HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff) == HAL_OK) {
        if (HAL_ETH_GetRxDataLength(&EthHandle, &frameLength) != HAL_OK) {
            tr_error("Error: returned by HAL_ETH_GetRxDataLength");
            return -1;
        }

        /* Build Rx descriptor to be ready for next data reception */
        HAL_ETH_BuildRxDescriptors(&EthHandle);

#if !(defined(DUAL_CORE) && defined(CORE_CM4))
        /* Invalidate data cache for ETH Rx Buffers */
        SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuff.buffer, frameLength);
#endif

        *buf = pbuf_alloc(PBUF_RAW, frameLength, PBUF_POOL);
        if (*buf) {
            pbuf_take((struct pbuf *)*buf, RxBuff.buffer, frameLength);
        }
    } else {
        return -1;
    }

    return 0;
}

/** \brief  Attempt to read a packet from the EMAC interface.
 *
 */
void STM32_EMAC::packet_rx()
{
    /* move received packet into a new buf */
    while (1) {
        emac_mem_buf_t *p = NULL;
        RXLockMutex.lock();
        if (low_level_input(&p) < 0) {
            RXLockMutex.unlock();
            break;
        }
        if (p) {
            emac_link_input_cb(p);
        }
        RXLockMutex.unlock();
    }
}

/** \brief  Worker thread.
 *
 * Woken by thread flags to receive packets or clean up transmit
 *
 *  \param[in] pvParameters pointer to the interface data
 */
void STM32_EMAC::thread_function(void *pvParameters)
{
    static struct STM32_EMAC *stm32_enet = static_cast<STM32_EMAC *>(pvParameters);

    for (;;) {
        uint32_t flags = osThreadFlagsWait(FLAG_RX, osFlagsWaitAny, osWaitForever);

        if (flags & FLAG_RX) {
            stm32_enet->packet_rx();
        }
    }
}

/**
 * This task checks phy link status and updates net status
 */
void STM32_EMAC::phy_task()
{
    const int32_t status = LAN8742_GetLinkState(&LAN8742);
    const bool is_up = (status > LAN8742_STATUS_LINK_DOWN);
    const bool was_up = (phy_status > LAN8742_STATUS_LINK_DOWN);

    if (is_up && !was_up) 
    {
        tr_info("Link Up");

        ETH_MACConfigTypeDef MACConf;
        HAL_ETH_GetMACConfig(&EthHandle, &MACConf);

        switch (status) {
        case LAN8742_STATUS_100MBITS_FULLDUPLEX:
            MACConf.DuplexMode = ETH_FULLDUPLEX_MODE;
            MACConf.Speed = ETH_SPEED_100M;
            break;
        case LAN8742_STATUS_100MBITS_HALFDUPLEX:
            MACConf.DuplexMode = ETH_HALFDUPLEX_MODE;
            MACConf.Speed = ETH_SPEED_100M;
            break;
        case LAN8742_STATUS_10MBITS_FULLDUPLEX:
            MACConf.DuplexMode = ETH_FULLDUPLEX_MODE;
            MACConf.Speed = ETH_SPEED_10M;
            break;
        case LAN8742_STATUS_10MBITS_HALFDUPLEX:
            MACConf.DuplexMode = ETH_HALFDUPLEX_MODE;
            MACConf.Speed = ETH_SPEED_10M;
            break;
        default:
            MACConf.DuplexMode = ETH_FULLDUPLEX_MODE;
            MACConf.Speed = ETH_SPEED_10M;
            break;
        }

        HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
        HAL_ETH_Start_IT(&EthHandle);
    } 
    else if (was_up && !is_up) 
    {
        tr_info("Link Down");
        // Stop ETH
        disable_interrupts();
        HAL_ETH_Stop(&EthHandle);
        enable_interrupts();
    }

    if (emac_link_state_cb) {
        if (is_up && !was_up) {
            emac_link_state_cb(true);
            tr_info("emac_link_state_cb set to true");
        } else if (!is_up && was_up) {
            emac_link_state_cb(false);
            tr_info("emac_link_state_cb set to false");
        }
    }

    phy_status = status;
}

#if defined (STM32F767xx) || defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx)
/**
 * workaround for the ETH RMII bug in STM32F76x and STM32F77x revA
 *
 * \param[in] netif the lwip network interface structure
 */

/** \brief  Worker thread.
 *
 * Woken by thread flags to receive packets or clean up transmit
 *
 *  \param[in] pvParameters pointer to the interface data
 */
void STM32_EMAC::rmii_watchdog_thread_function(void *pvParameters)
{
    struct STM32_EMAC *stm32_enet = static_cast<STM32_EMAC *>(pvParameters);

    while (1) {
        /* some good packets are received */
        if (stm32_enet->EthHandle.Instance->MMCRGUFCR > 0) {
            /* RMII Init is OK - would need service to terminate or suspend
             * the thread */
            while (1) {
                /*  don't do anything anymore */
                osDelay(0xFFFFFFFF);
            }
        } else if (stm32_enet->EthHandle.Instance->MMCRFCECR > 10) {
            /* ETH received too many packets with CRC errors, resetting RMII */
            SYSCFG->PMC &= ~SYSCFG_PMC_MII_RMII_SEL;
            SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;
            stm32_enet->EthHandle.Instance->MMCCR |= ETH_MMCCR_CR;
        } else {
            osDelay(100);
        }
    }
}
#endif

void STM32_EMAC::enable_interrupts(void)
{
    HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

void STM32_EMAC::disable_interrupts(void)
{
    NVIC_DisableIRQ(ETH_IRQn);
}

/** This returns a unique 6-byte MAC address, based on the device UID
*  This function overrides hal/common/mbed_interface.c function
*  @param mac A 6-byte array to write the MAC address
*/

void mbed_mac_address(char *mac)
{
    if (mbed_otp_mac_address(mac)) {
        return;
    } else {
        mbed_default_mac_address(mac);
    }
    return;
}

MBED_WEAK uint8_t mbed_otp_mac_address(char *mac)
{
    return 0;
}

void mbed_default_mac_address(char *mac)
{
    unsigned char ST_mac_addr[3] = {0x00, 0x80, 0xe1}; // default STMicro mac address

    // Read unic id
#if defined (TARGET_STM32F2)
    uint32_t word0 = *(uint32_t *)0x1FFF7A10;
#elif defined (TARGET_STM32F4)
    uint32_t word0 = *(uint32_t *)0x1FFF7A10;
#elif defined (TARGET_STM32F7)
    uint32_t word0 = *(uint32_t *)0x1FF0F420;
#elif defined (TARGET_STM32H7)
    uint32_t word0 = *(uint32_t *)0x1FF1E800;
#else
#error MAC address can not be derived from target unique Id
#endif

    mac[0] = ST_mac_addr[0];
    mac[1] = ST_mac_addr[1];
    mac[2] = ST_mac_addr[2];
    mac[3] = (word0 & 0x00ff0000) >> 16;
    mac[4] = (word0 & 0x0000ff00) >> 8;
    mac[5] = (word0 & 0x000000ff);

    return;
}

bool STM32_EMAC::power_up()
{
    sleep_manager_lock_deep_sleep();

    /* Initialize the hardware */
    if (!low_level_init_successful()) {
        return false;
    }

    /* Worker thread */
#if MBED_CONF_MBED_TRACE_ENABLE
    thread = create_new_thread("stm32_emac_thread", &STM32_EMAC::thread_function, this, MBED_CONF_STM32_EMAC_THREAD_STACKSIZE * 2, THREAD_PRIORITY, &thread_cb);
#else
    thread = create_new_thread("stm32_emac_thread", &STM32_EMAC::thread_function, this, MBED_CONF_STM32_EMAC_THREAD_STACKSIZE, THREAD_PRIORITY, &thread_cb);
#endif


    phy_task_handle = mbed::mbed_event_queue()->call_every(PHY_TASK_PERIOD, mbed::callback(this, &STM32_EMAC::phy_task));

#if defined (STM32F767xx) || defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx)
    rmii_watchdog_thread = create_new_thread("stm32_rmii_watchdog", &STM32_EMAC::rmii_watchdog_thread_function, this, 128, THREAD_PRIORITY, &rmii_watchdog_thread_cb);
#endif

    /* Allow the PHY task to detect the initial link state and set up the proper flags */
    osDelay(10);

    enable_interrupts();

    return true;
}

uint32_t STM32_EMAC::get_mtu_size() const
{
    return STM_ETH_MTU_SIZE;
}

uint32_t STM32_EMAC::get_align_preference() const
{
    return 0;
}

void STM32_EMAC::get_ifname(char *name, uint8_t size) const
{
    memcpy(name, STM_ETH_IF_NAME, (size < sizeof(STM_ETH_IF_NAME)) ? size : sizeof(STM_ETH_IF_NAME));
}

uint8_t STM32_EMAC::get_hwaddr_size() const
{
    return STM_HWADDR_SIZE;
}

bool STM32_EMAC::get_hwaddr(uint8_t *addr) const
{
    mbed_mac_address((char *)addr);
    return true;
}

void STM32_EMAC::set_hwaddr(const uint8_t *addr)
{
    /* No-op at this stage */
}

void STM32_EMAC::set_link_input_cb(emac_link_input_cb_t input_cb)
{
    emac_link_input_cb = input_cb;
}

void STM32_EMAC::set_link_state_cb(emac_link_state_change_cb_t state_cb)
{
    emac_link_state_cb = state_cb;
}

void STM32_EMAC::add_multicast_group(const uint8_t *addr)
{
    /* No-op at this stage */	
}

void STM32_EMAC::remove_multicast_group(const uint8_t *addr)
{
    /* No-op at this stage */
}

void STM32_EMAC::set_all_multicast(bool all)
{
	tr_info("---------------set_all_multicast----------------------------------");
    /* No-op at this stage */
	STM32_EMAC &emac = STM32_EMAC::get_instance();
	ETH_MACFilterConfigTypeDef pFilterConfig;
	if (HAL_ETH_GetMACFilterConfig(&emac.EthHandle, &pFilterConfig) != HAL_OK) {
		return;
	}
	pFilterConfig.PassAllMulticast = all ? ENABLE : DISABLE;
	HAL_ETH_SetMACFilterConfig(&emac.EthHandle, &pFilterConfig);
}

void STM32_EMAC::power_down()
{
    tr_info("power_down");

    /* No-op at this stage */
    sleep_manager_unlock_deep_sleep();
}

void STM32_EMAC::set_memory_manager(EMACMemoryManager &mem_mngr)
{
    memory_manager = &mem_mngr;
}

STM32_EMAC &STM32_EMAC::get_instance()
{
    static STM32_EMAC emac;
    return emac;
}

// Weak so a module can override
MBED_WEAK EMAC &EMAC::get_default_instance()
{
    return STM32_EMAC::get_instance();
}

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/

/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
static int32_t ETH_PHY_IO_Init(void)
{
    /* We assume that MDIO GPIO configuration is already done
        in the ETH_MspInit() else it should be done here
    */
    STM32_EMAC &emac = STM32_EMAC::get_instance();

    /* Configure the MDIO Clock */
    HAL_ETH_SetMDIOClockRange(&emac.EthHandle);

    return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
static int32_t ETH_PHY_IO_DeInit(void)
{
    return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value
  * @retval 0 if OK -1 if Error
  */
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
    STM32_EMAC &emac = STM32_EMAC::get_instance();
    if (HAL_ETH_ReadPHYRegister(&emac.EthHandle, DevAddr, RegAddr, pRegVal) != HAL_OK) {
        return -1;
    }

    return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written
  * @retval 0 if OK -1 if Error
  */
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
    STM32_EMAC &emac = STM32_EMAC::get_instance();
    if (HAL_ETH_WritePHYRegister(&emac.EthHandle, DevAddr, RegAddr, RegVal) != HAL_OK) {
        return -1;
    }

    return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
static int32_t ETH_PHY_IO_GetTick(void)
{
    return HAL_GetTick();
}

/**
  * Ethernet DMA transfer error callbacks
  */
void HAL_ETH_DMAErrorCallback(ETH_HandleTypeDef *heth)
{
    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_DMAErrorCallback)\n");
}

/**
  * Ethernet MAC transfer error callbacks
  */
void HAL_ETH_MACErrorCallback(ETH_HandleTypeDef *heth)
{
    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_MACErrorCallback)\n");
}

#endif /* DEVICE_EMAC */