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
#ifdef ETH_IP_VERSION_V3

#include <stdlib.h>

#include "cmsis_os.h"

#include "mbed_interface.h"
#include "mbed_assert.h"
#include "events/mbed_shared_queues.h"
#include "netsocket/nsapi_types.h"
#include "platform/mbed_power_mgmt.h"
#include "platform/mbed_error.h"

#include "stm32xx_emac_config.h"
#include "stm32xx_emac.h"

#include "mbed-trace/mbed_trace.h"

#include "lan8742/lan8742.h"
#include "lwip/memp.h"
#include "lwip/api.h"

#include "rtos/Semaphore.h"

#include "Semaphore.h"

#define TRACE_GROUP "STE3"

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
#define FLAG_CONNECTED          2

/** \brief  Driver thread priority */
#define THREAD_PRIORITY         (osPriorityHigh)

//#define PHY_TASK_PERIOD      200ms
#define PHY_TASK_PERIOD      500ms

#define ETH_RX_BUFFER_SIZE            1000U

#define STM_HWADDR_SIZE         (6)
#define STM_ETH_MTU_SIZE        1500
#define STM_ETH_IF_NAME         "st"

//#define ETH_MAX_PACKET_SIZE     1524

// typedef struct
// {
//   struct pbuf_custom pbuf_custom;
//   uint8_t buff[(ETH_MAX_PACKET_SIZE + 31) & ~31] __ALIGNED(32);
// } RxBuff_t;

// typedef enum
// {
//   RX_ALLOC_OK       = 0x00,
//   RX_ALLOC_ERROR    = 0x01
// } RxAllocStatusTypeDef;/* Variable Definitions */
// static uint8_t RxAllocStatus;

//ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__(( section(".RxDecripSection"), aligned(32)));/* Ethernet Rx DMA Descriptor */

//ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"), aligned(32)));/* Ethernet Tx DMA Descriptor */

//uint32_t Rx_Buff_Idx;
//uint8_t DMARxBuffer[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */
//uint8_t DMARxBuffer[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".emac_section"), aligned(32))); /* Ethernet Receive Buffer */
uint8_t DMARxBuffer[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] __attribute__((section(".emac_section"), aligned(32))); /* Ethernet Receive Buffer */



//uint8_t Tx_Buff[ETH_TX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".TxArraySection"))); /* Ethernet Transmit Buffers */

//MBED_ASSERT(ETH_MAX_PACKET_SIZE % 4 == 0, "Rx buffer size must be a multiple of 4");

// typedef struct my_custom_pbuf
// {
//    struct pbuf_custom p;
//    //void* dma_descriptor;
//    uint8_t buff[(ETH_MAX_PACKET_SIZE + 31) & ~31] __ALIGNED(32);
// } my_custom_pbuf_t;

/* Memory Pool Declaration */
//LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_DESC_CNT, sizeof(my_custom_pbuf_t), "Zero-copy RX PBUF pool");

//osSemaphoreId RxPktSemaphore = NULL;   /* Semaphore to signal incoming packets */
//osSemaphoreId TxPktSemaphore = NULL;   /* Semaphore to signal transmit packet complete */
//osSemaphoreId TxPktSemaphore = osSemaphoreNew(1, 1, NULL);;   /* Semaphore to signal transmit packet complete */
rtos::Semaphore TxPktSemaphore(1);


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

volatile bool IsConnected = false;

MBED_WEAK uint8_t mbed_otp_mac_address(char *mac);
void mbed_default_mac_address(char *mac);

#ifdef __cplusplus
extern "C" {
#endif

void _eth_config_mac(ETH_HandleTypeDef *heth);
//void ETH_IRQHandler(void);
//MBED_WEAK void STM_HAL_ETH_Handler(ETH_HandleTypeDef *heth);

#ifdef __cplusplus
}
#endif

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    /* Configure the MPU attributes as Device not cacheable for ETH DMA descriptors */
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

    // MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    // MPU_InitStruct.BaseAddress = 0x30040400;
    // MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    // MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    // MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    // MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    // MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    // MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    // MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    // MPU_InitStruct.SubRegionDisable = 0x00;
    // MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    // HAL_MPU_ConfigRegion(&MPU_InitStruct);

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
// MBED_WEAK void STM_HAL_ETH_Handler()
// {
// }
void STM_HAL_ETH_Handler()
{
   STM32_EMAC &emac = STM32_EMAC::get_instance();
   HAL_ETH_IRQHandler(&emac.EthHandle);
}

/**
 * Ethernet IRQ Handler
 *
 * @param  None
 * @retval None
 */
// void ETH_IRQHandler(void)
// {
//     STM_HAL_ETH_Handler();
// }

STM32_EMAC::STM32_EMAC()
    : thread(0)
    , phy_status(0)
{
}

static osThreadId_t create_new_thread(const char *threadName, void (*thread)(void *arg), void *arg, int stacksize, osPriority_t priority, mbed_rtos_storage_thread_t *thread_cb)
{
    tr_info("create_new_thread %s stack:%d", threadName, stacksize);
    
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
    MPU_Config();

    /* Init ETH */
    EthHandle.Instance = ETH;

    uint8_t MACAddr[6];
    mbed_mac_address((char *)MACAddr);
    EthHandle.Init.MACAddr = &MACAddr[0];

    EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    EthHandle.Init.RxDesc = DMARxDscrTab;
    EthHandle.Init.TxDesc = DMATxDscrTab;
    //EthHandle.Init.RxBuffLen = ETH_MAX_PACKET_SIZE;
    //EthHandle.Init.RxBuffLen = 1524;
    EthHandle.Init.RxBuffLen = 1536;

    tr_info("MAC Addr %02x:%02x:%02x:%02x:%02x:%02x", MACAddr[0], MACAddr[1], MACAddr[2], MACAddr[3], MACAddr[4], MACAddr[5]);
    tr_info("ETH buffers Rx:%u size:%u Tx:%u size:%u - rxbuffer:%u", ETH_RX_DESC_CNT, sizeof(DMARxDscrTab), ETH_TX_DESC_CNT, sizeof(DMATxDscrTab), sizeof(DMARxBuffer));

    //NVIC_SetVector(ETH_IRQn, (uint32_t)&HAL_ETH_IRQHandler);
    NVIC_SetVector(ETH_IRQn, (uint32_t)&STM_HAL_ETH_Handler);

    //HAL_ETH_RegisterRxAllocateCallback(&EthHandle, HAL_ETH_RxAllocateCallback);
    //HAL_ETH_RegisterRxLinkCallback(&EthHandle, HAL_ETH_RxLinkCallback);

    //HAL_ETH_RegisterTxFreeCallback(&EthHandle, HAL_ETH_RxLinkCallback);

    /* create a binary semaphore used for informing ethernetif of frame reception */
    //RxPktSemaphore = osSemaphoreNew(1, 1, NULL);
    /* create a binary semaphore used for informing ethernetif of frame transmission */
    //TxPktSemaphore = osSemaphoreNew(1, 1, NULL);


    // for (idx = 0; idx < ETH_RX_DESC_CNT; idx++) {
    //     HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
    // }




    /* Set PHY IO functions */
    LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
    /* Initialize the LAN8742 ETH PHY */
    if(LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
    {
        tr_error("LAN8742_Init failed");
        return false;
    }

    if (HAL_ETH_Init(&EthHandle) != HAL_OK) 
    {
        tr_error("HAL_ETH_Init failed");
        return false;
    }

    // Enable timestamping of RX packets. We enable all packets to be timestamped to cover both IEEE 1588 and gPTP.
    //EthHandle.Instance->MACTSCR |= ETH_MACTSCR_TSENALL;

    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    //TxConfig.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    tr_info("low_level_init_successful");
    return true;
}


const char *getMACError(ETH_HandleTypeDef *heth)
{
    uint32_t mac_error = HAL_ETH_GetMACError(heth);
    if (mac_error & ETH_RECEIVE_WATCHDOG_TIMEOUT)
        return "ETH_RECEIVE_WATCHDOG_TIMEOUT";
    else if (mac_error & ETH_EXECESSIVE_COLLISIONS)
        return "ETH_EXECESSIVE_COLLISIONS";
    else if (mac_error & ETH_LATE_COLLISIONS)
        return "ETH_LATE_COLLISIONS";
    else if (mac_error & ETH_EXECESSIVE_DEFERRAL)
        return "ETH_EXECESSIVE_DEFERRAL";
    else if (mac_error & ETH_TRANSMIT_JABBR_TIMEOUT)
        return "ETH_TRANSMIT_JABBR_TIMEOUT";
    else if (mac_error & ETH_LOSS_OF_CARRIER)
        return "ETH_LOSS_OF_CARRIER";
    else if (mac_error & ETH_NO_CARRIER)
        return "ETH_NO_CARRIER";
    return "?????";
}

//--------------------- TX ---------------------------


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
    tr_info("link_out");
     
    bool success = false;
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
    HAL_StatusTypeDef status;

    struct pbuf *p = (struct pbuf *)buf;

    printf("buf %lu\r\n", buf);
    printf("p   %lu\r\n", p);

    /* Get exclusive access */
    TXLockMutex.lock();

    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));

    uint32_t i = 0;
    struct pbuf *q = NULL;
    /* copy frame from pbufs to driver buffers */
    for (q = p; q != NULL; q = q->next) 
    {
        if (i >= ETH_TX_DESC_CNT) 
        {
            tr_error("Error : ETH_TX_DESC_CNT not sufficient");
            //goto error;
            return false;
        }

        Txbuffer[i].buffer = (uint8_t *)q->payload;
        Txbuffer[i].len = q->len;

        if (i > 0) 
        {
            Txbuffer[i - 1].next = &Txbuffer[i];
        }
        if (q->next == NULL) 
        {
            Txbuffer[i].next = NULL;
        }
        i++;
    }

    //printf("frameLength:%lu tot_len:%lu\r\n", frameLength, p->tot_len);
    printf("tot_len:%u\r\n", p->tot_len);
    printf("ref:%u\r\n", p->ref);

    TxConfig.Length = p->tot_len;
    TxConfig.TxBuffer = Txbuffer;
    TxConfig.pData = p;

    //TxConfig.PayloadLen
    //pbuf_ref(p);

    //tr_info("send %lu", frameLength);

    // status = HAL_ETH_Transmit(&EthHandle, &TxConfig, 50);
    // if (status == HAL_OK) 
    // {
    //     success = true;
    // } 
    // else 
    // {
    //     tr_error("Error returned by HAL_ETH_Transmit (%d)", status);
    //     if (HAL_ETH_GetState(&EthHandle) == HAL_ETH_STATE_ERROR) {
    //         tr_error("ETH error (%lx)", HAL_ETH_GetError(&EthHandle));
	// 	}
    //     if (HAL_ETH_GetDMAError(&EthHandle)) {
    //         tr_error("DMA error (%lx)", HAL_ETH_GetDMAError(&EthHandle));
	// 	}
    //     if (HAL_ETH_GetMACError(&EthHandle)) {
    //         tr_error("MAC error (%lx) %s", HAL_ETH_GetMACError(&EthHandle), getMACError(&EthHandle));
	// 	}
    //     success = false;
    // }
    //----------------------

    //pbuf_ref(p);

    printf("transmit\r\n");

    err_t errval = ERR_OK;
    do
    {
        if(HAL_ETH_Transmit_IT(&EthHandle, &TxConfig) == HAL_OK)
        {
            errval = ERR_OK;
            success = true;
        }
        else
        {
            if(HAL_ETH_GetError(&EthHandle) & HAL_ETH_ERROR_BUSY)
            {
                printf("waiting for tx\r\n");
                /* Wait for descriptors to become available */
                TxPktSemaphore.try_acquire_for(500ms);
                HAL_ETH_ReleaseTxPacket(&EthHandle);
                errval = ERR_BUF;                
            }
            else
            {
                /* Other error */
                pbuf_free(p);
                errval =  ERR_IF;                
            }
        }
    }while(errval == ERR_BUF);

    printf("send done\r\n");

    /* Restore access */
    TXLockMutex.unlock();
    return success;


//     status = HAL_ETH_Transmit_IT(&EthHandle, &TxConfig);
//     printf("status %lu\r\n", status);

//     if (status == HAL_OK) 
//     {
//         success = true;
//     } 
//     else 
//     {
//         tr_error("Error returned by HAL_ETH_Transmit_IT (%d)", status);
//         if (HAL_ETH_GetState(&EthHandle) == HAL_ETH_STATE_ERROR) {
//             tr_error("ETH error (%x)", HAL_ETH_GetError(&EthHandle));
// 		}
//         if (HAL_ETH_GetDMAError(&EthHandle)) {
//             tr_error("DMA error (%x)", HAL_ETH_GetDMAError(&EthHandle));
// 		}
//         if (HAL_ETH_GetMACError(&EthHandle)) {
//             tr_error("MAC error (%x) %s", HAL_ETH_GetMACError(&EthHandle), getMACError(&EthHandle));
// 		}
//         success = false;
//     }
//     //TxPktSemaphore.try_acquire_for(500ms);
//     //osSemaphoreWait(TxPktSemaphore, osWaitForever);
//     // while(osSemaphoreWait(TxPktSemaphore, osWaitForever) != osOK)
//     // {
//     // }
//     tr_info("release");
//     HAL_ETH_ReleaseTxPacket(&EthHandle);

// error:

//     if (p->ref > 1) {
//         pbuf_free(p);
//     }

//     /* Restore access */
//     TXLockMutex.unlock();

//     return success;
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth_handle)
{
    //tr_info("HAL_ETH_TxCpltCallback");
    //osSemaphoreRelease(TxPktSemaphore);
    TxPktSemaphore.release();
}

void HAL_ETH_TxFreeCallback(uint32_t * buff)
{
    tr_info("HAL_ETH_TxFreeCallback");
    pbuf_free((struct pbuf *)buff);
}


//--------------------- RX ---------------------------


/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
// void pbuf_free_custom(struct pbuf *p)
// {
//   struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
//   LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
//    /* If the Rx Buffer Pool was exhausted, signal the ethernetif_input task to
//    * call HAL_ETH_GetRxDataBuffer to rebuild the Rx descriptors. */
//   if (RxAllocStatus == RX_ALLOC_ERROR)
//   {
//     RxAllocStatus = RX_ALLOC_OK;
//   }
// }

struct eth_stm32_rx_buffer_header {
	struct eth_stm32_rx_buffer_header *next;
	uint16_t size;
	bool used;
    //uint8_t buff[(ETH_MAX_PACKET_SIZE + 31) & ~31] __ALIGNED(32);
};

// struct eth_stm32_tx_buffer_header {
// 	ETH_BufferTypeDef tx_buff;
// 	bool used;
// };

// struct eth_stm32_tx_context {
// 	struct net_pkt *pkt;
// 	uint16_t first_tx_buffer_index;
// };

// typedef struct my_custom_pbuf
// {
//    struct pbuf_custom p;
//    void* dma_descriptor;
// } my_custom_pbuf_t;



/* Memory Pool Declaration */
//LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool")
//LWIP_PBUF_MEMPOOL()


static struct eth_stm32_rx_buffer_header DMARxBufferHeader[ETH_RX_DESC_CNT];
//static struct eth_stm32_tx_buffer_header dma_tx_buffer_header[ETH_TX_DESC_CNT];

//ETH_BufferTypeDef RxBuff;

/**
  \brief       Rx Allocate callback.
  \param[in]   buff  pointer to allocated buffer
  \return      None
*/
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
    //tr_info("HAL_ETH_RxAllocateCallback %u connected:%u", **buff, IsConnected);
    tr_info("HAL_ETH_RxAllocateCallback %u", **buff);

    //STM32_EMAC &emac = STM32_EMAC::get_instance();
    //emac.memory_manager->alloc_pool()


    for (size_t i = 0; i < ETH_RX_DESC_CNT; ++i)
    {        
		if (!DMARxBufferHeader[i].used)
        {
			DMARxBufferHeader[i].next = NULL;
			DMARxBufferHeader[i].size = 0;
			DMARxBufferHeader[i].used = true;
			*buff = DMARxBuffer[i];
            //*buff = DMARxBufferHeader[i].buff;
			return;
		}
	}
	*buff = NULL;



    // STM32_EMAC &emac = STM32_EMAC::get_instance();
    // for (size_t i = 0; i < ETH_RX_DESC_CNT; ++i)
    // {
    //     emac_mem_buf_t *p = emac.memory_manager->alloc_pool(ETH_MAX_PACKET_SIZE, i);
    //     if(p)
    //     {
    //         printf("ref %u\r\n", ((pbuf *)p)->ref);
    //         //((pbuf)p).ref
    //         //((pbuf)p).payload = DMARxBuffer[];

    //         *buff = (uint8_t *)p;
    //         //*buff = (uint8_t *)p + offsetof(emac_mem_buf_t, buff);
    //         //pbuf_alloc(PBUF_RAW, ETH_MAX_PACKET_SIZE,)
    //         return;
    //     }
	// }    
    // *buff = NULL;
    // printf("buff == null\r\n");


    // STM32_EMAC &emac = STM32_EMAC::get_instance();
    // emac_mem_buf_t *p = emac.memory_manager->alloc_pool(ETH_MAX_PACKET_SIZE, 0);
    // if(p)
    // {
    //     printf("ref %u\r\n", ((pbuf *)p)->ref);
    //     //((pbuf)p).ref
    //     //((pbuf)p).payload = DMARxBuffer[];

    //     *buff = (uint8_t *)p;
    //     //*buff = (uint8_t *)p + offsetof(emac_mem_buf_t, buff);
    //     //pbuf_alloc(PBUF_RAW, ETH_MAX_PACKET_SIZE,)
    // }
    // else 
    // {
    //     printf("buff == null\r\n");
    //     *buff = NULL;
    // }

    
}

/* Pointer to an array of ETH_MAX_PACKET_SIZE uint8_t's */
typedef uint8_t (*RxBufferPtr)[ETH_RX_BUFFER_SIZE];

/**
  \brief       Rx Link callback.
  \param[in]   pStart  pointer to packet start
  \param[in]   pStart  pointer to packet end
  \param[in]   buff    pointer to received data
  \param[in]   Length  received data length
  \return      None
*/
void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
    tr_info("HAL_ETH_RxLinkCallback buf:%d len:%d", *buff, Length);



    // //tr_info("HAL_ETH_RxLinkCallback start:%d end:%d buf:%d len:%d", (uint32_t)**pStart, (uint32_t)**pEnd, *buff, Length);
    // tr_info("HAL_ETH_RxLinkCallback buf:%d len:%d", *buff, Length);

    // /* buff points to the begin on one of the rx buffers,
	//  * so we can compute the index of the given buffer
	//  */
	// size_t index = (RxBufferPtr)buff - &DMARxBuffer[0];
    // tr_info("link index %d", index);

	// struct eth_stm32_rx_buffer_header *header = &DMARxBufferHeader[index];
    // tr_info("header size:%d next:%d used:%d", header->size, header->next, header->used);

    // MBED_ASSERT(index < ETH_RX_DESC_CNT);

	// header->size = Length;

	// if (!*pStart) {
	// 	/* first packet, set head pointer of linked list */
	// 	*pStart = header;
	// 	*pEnd = header;
	// } else {
    //     MBED_ASSERT(*pEnd != NULL);
	// 	/* not the first packet, add to list and adjust tail pointer */
	// 	((struct eth_stm32_rx_buffer_header *)*pEnd)->next = header;
	// 	*pEnd = header;
	// }
    // tr_info("header size:%d next:%d used:%d", header->size, header->next, header->used);
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
	//tr_info("HAL_ETH_RxCpltCallback");
    //osSemaphoreRelease(RxPktSemaphore);
    // STM32_EMAC &emac = STM32_EMAC::get_instance();
    // if (emac.thread) {
    //     osThreadFlagsSet(emac.thread, FLAG_RX);
    // }
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
// int STM32_EMAC::low_level_input(emac_mem_buf_t **buf)
// {
//     tr_info("low_level_input");

//     struct pbuf *p = NULL;

//     HAL_StatusTypeDef ret = HAL_ETH_ReadData(&EthHandle, (void **)&p);

//     //if(HAL_ETH_ReadData(&EthHandle, (void **)&p) == HAL_OK)
//     if(ret == HAL_OK)
//     {
//         tr_info("read len:%d next:%d", p->len, p->next);

//         *buf = pbuf_alloc(PBUF_RAW, p->len, PBUF_POOL);
//         if (*buf) {
//             pbuf_take((struct pbuf *)*buf, p->payload, p->len);
//         }
//     }
//     else
//     {
//         return -1;
//     }
//     return 0;


//     ETH_BufferTypeDef RxBuff;
//     uint32_t frameLength = 0;

//     if (HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff) == HAL_OK) {
//         if (HAL_ETH_GetRxDataLength(&EthHandle, &frameLength) != HAL_OK) {
//             tr_error("Error: returned by HAL_ETH_GetRxDataLength");
//             return -1;
//         }

//         /* Build Rx descriptor to be ready for next data reception */
//         HAL_ETH_BuildRxDescriptors(&EthHandle);

// #if !(defined(DUAL_CORE) && defined(CORE_CM4))
//         /* Invalidate data cache for ETH Rx Buffers */
//         SCB_InvalidateDCache_by_Addr((uint32_t *)RxBuff.buffer, frameLength);
// #endif

//         *buf = pbuf_alloc(PBUF_RAW, frameLength, PBUF_POOL);
//         if (*buf) {
//             pbuf_take((struct pbuf *)*buf, RxBuff.buffer, frameLength);
//         }


//     } else {
//         return -1;
//     }

//     return 0;
//}

/** \brief  Attempt to read a packet from the EMAC interface.
 *
 */
// void STM32_EMAC::packet_rx()
// {
//     /* move received packet into a new buf */
//     //while (1) {
//         emac_mem_buf_t *p = NULL;
//         RXLockMutex.lock();
//         if (low_level_input(&p) < 0) 
//         {
//             RXLockMutex.unlock();
//             //break;
//         }
//         if (p) 
//         {
//             emac_link_input_cb(p);
//         }
//         RXLockMutex.unlock();
//     //}
// }

/** \brief  Worker thread.
 *
 * Woken by thread flags to receive packets or clean up transmit
 *
 *  \param[in] pvParameters pointer to the interface data
 */
void STM32_EMAC::thread_function(void *pvParameters)
{
    tr_info("thread_function");

    static struct STM32_EMAC *stm32_enet = static_cast<STM32_EMAC *>(pvParameters);
    while (1)
    {
        //printf(".");
        if(IsConnected)
        {        
        //uint32_t flags = osThreadFlagsWait(FLAG_CONNECTED, osFlagsWaitAny, osWaitForever);
        //if (flags & FLAG_CONNECTED)
        //{ 
            //osThreadFlagsClear(FLAG_CONNECTED);
        struct pbuf *pb = NULL;
        HAL_StatusTypeDef ret = HAL_ETH_ReadData(&stm32_enet->EthHandle, (void **)&pb);

        //tr_info("ret:%d error:%d state:0x%x", ret, stm32_enet->EthHandle.ErrorCode, stm32_enet->EthHandle.gState);

        //if(HAL_ETH_ReadData(&stm32_enet->EthHandle, (void **)&pb) == HAL_OK)
        if(ret == HAL_OK)
        {
            tr_info("read len:%d next:%d", pb->len, pb->next);

                    // emac_mem_buf_t *buf = NULL;
                    // buf = pbuf_alloc(PBUF_RAW, pb->len, PBUF_POOL);
                    // if (buf) 
                    // {
                    //     pbuf_take((struct pbuf *)buf, pb->payload, pb->len);
                    //     stm32_enet->emac_link_input_cb(buf);
                    // }
        }
        //}
        }
        else
        {
            osDelay(1000);
            printf(".");
        }

        // tr_info("wait for flag");
        // uint32_t flags = osThreadFlagsWait(FLAG_RX, osFlagsWaitAny, osWaitForever);

        // if (flags & FLAG_RX)
        // {            
        //     tr_info("FLAG_RX");
        //     //stm32_enet->packet_rx();
        //     //if(IsConnected)
        //     //{
        //         emac_mem_buf_t *buf = NULL;
        //         // if (stm32_enet->low_level_input(&p) == 0) 
        //         // {
        //         //     //RXLockMutex.unlock();
        //         //     //break;
        //         //     stm32_enet->emac_link_input_cb(p);
        //         // }
        //         // if (p) 
        //         // {
        //         //     emac_link_input_cb(p);
        //         // }


        //         //stm32_enet->EthHandle

                
                
                
        //         //dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
  
        //         tr_info("RxDescIdx %d", stm32_enet->EthHandle.RxDescList.RxDescIdx);
        //         //tr_info("RxBuildDescCnt val %d", stm32_enet->EthHandle.RxDescList.RxBuildDescCnt);
        //         //tr_info("RxBuildDescCnt - %d", (ETH_RX_DESC_CNT - stm32_enet->EthHandle.RxDescList.RxBuildDescCnt));
        //         tr_info("RxBuildDescCnt bool %d", 0 < (ETH_RX_DESC_CNT - stm32_enet->EthHandle.RxDescList.RxBuildDescCnt));
        //         tr_info("state:0x%x", stm32_enet->EthHandle.gState);
                
        //         struct pbuf *pb = NULL;
        //         HAL_StatusTypeDef ret = HAL_ETH_ReadData(&stm32_enet->EthHandle, (void **)&pb);

        //         tr_info("ret:%d error:%d state:0x%x", ret, stm32_enet->EthHandle.ErrorCode, stm32_enet->EthHandle.gState);

        //         //if(HAL_ETH_ReadData(&stm32_enet->EthHandle, (void **)&pb) == HAL_OK)
        //         if(ret == HAL_OK)
        //         {
        //             tr_info("read len:%d next:%d", pb->len, pb->next);

        //             buf = pbuf_alloc(PBUF_RAW, pb->len, PBUF_POOL);
        //             if (buf) 
        //             {
        //                 pbuf_take((struct pbuf *)buf, pb->payload, pb->len);
        //                 stm32_enet->emac_link_input_cb(buf);
        //             }
        //         }



        //     //}
        //}
    }
}


/**
 * This task checks phy link status and updates net status
 */
void STM32_EMAC::phy_task()
{
    const int32_t status = LAN8742_GetLinkState(&LAN8742);
    //const int32_t old_status = (int32_t)phy_status;
    //const int32_t old_status = phy_status;
    const bool is_up  = (status > LAN8742_STATUS_LINK_DOWN);
    //const bool was_up = (old_status > LAN8742_STATUS_LINK_DOWN);
    const bool was_up = (phy_status > LAN8742_STATUS_LINK_DOWN);

    //tr_info("check phy");

    if (is_up && !was_up)
    {
        tr_info("------------------------- connected -------------------------------");
        ETH_MACConfigTypeDef MACConf;
        HAL_ETH_GetMACConfig(&EthHandle, &MACConf);
        switch (status) 
        {
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

        //MACConf.Watchdog = DISABLE;
        //tr_info("watchdog %d", MACConf.Watchdog);
        //tr_info("watchdog time %d", MACConf.WatchdogTimeout);
        HAL_ETH_SetMACConfig(&EthHandle, &MACConf);

        //HAL_ETH_Start_IT(&EthHandle);    
        tr_info("start IT");    
        if(HAL_ETH_Start_IT(&EthHandle) != HAL_OK)
        {
            tr_error("HAL_ETH_Start_IT failed");
        }
        enable_interrupts();

        // STM32_EMAC &emac = STM32_EMAC::get_instance();
        // if (emac.thread) {
        //     osThreadFlagsSet(emac.thread, FLAG_CONNECTED);
        // }
        IsConnected = true;
    } 
    else if (was_up && !is_up)
    {
        tr_info("------------------- disconnected ----------------------");
        IsConnected = false;
        // STM32_EMAC &emac = STM32_EMAC::get_instance();
        // if (emac.thread) {
        //     osThreadFlagsClear(emac.thread, FLAG_CONNECTED);
        // }

        // Stop ETH
        disable_interrupts();
        //HAL_ETH_Stop(&EthHandle);
        tr_info("stop IT");
        if(HAL_ETH_Stop_IT(&EthHandle) != HAL_OK)
        {
            tr_error("HAL_ETH_Stop_IT failed");
        }
        enable_interrupts();
    }
    
    if (emac_link_state_cb)
    {
        if (is_up && !was_up) {
            emac_link_state_cb(true);
            tr_info("emac_link_state_cb set to true");            
        } else if (!is_up && was_up) {
            emac_link_state_cb(false);
            tr_info("emac_link_state_cb set to false");            
        }
    }
    //phy_status = (uint32_t)status;
    phy_status = status;
}



void STM32_EMAC::enable_interrupts(void)
{
    tr_info("enable_interrupts");
    //NVIC_SetVector(ETH_IRQn, (uint32_t)&ETH_IRQHandler);
    //NVIC_SetVector(ETH_IRQn, (uint32_t)&HAL_ETH_IRQHandler);
    HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
    NVIC_SetVector(ETH_IRQn, (uint32_t)&STM_HAL_ETH_Handler);
}

void STM32_EMAC::disable_interrupts(void)
{
    tr_info("disable_interrupts");
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
    uint32_t word0 = *(uint32_t *)0x1FF1E800;

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
    if (!low_level_init_successful()) 
    {
        tr_info("low_level_init_successful failed");
        return false;
    }

    /* Worker thread */
#if MBED_CONF_MBED_TRACE_ENABLE
    //thread = create_new_thread("stm32_emac_thread", &STM32_EMAC::thread_function, this, MBED_CONF_STM32_EMAC_THREAD_STACKSIZE * 2, THREAD_PRIORITY, &thread_cb);
#else
    thread = create_new_thread("stm32_emac_thread", &STM32_EMAC::thread_function, this, MBED_CONF_STM32_EMAC_THREAD_STACKSIZE, THREAD_PRIORITY, &thread_cb);
#endif

    tr_info("Start phy task");
    //check phy every period.
    phy_task_handle = mbed::mbed_event_queue()->call_every(PHY_TASK_PERIOD, mbed::callback(this, &STM32_EMAC::phy_task));

    tr_info("wait");
    /* Allow the PHY task to detect the initial link state and set up the proper flags */
    osDelay(10);
    
    //enable_interrupts();

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
    /* No-op at this stage */
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
/**
  * Ethernet error callbacks
  */
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    MBED_ASSERT(heth != NULL);

    uint32_t error_code = HAL_ETH_GetError(heth);
    switch (error_code) 
    {
	    case HAL_ETH_ERROR_NONE:
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_NONE\n");
            break;
        case HAL_ETH_ERROR_PARAM:
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_PARAM\n");
            break;
        case HAL_ETH_ERROR_BUSY:
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_BUSY\n");
            break;
        case HAL_ETH_ERROR_TIMEOUT:
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_TIMEOUT\n");
            break;
        case HAL_ETH_ERROR_DMA:
        {
            uint32_t dma_error = HAL_ETH_GetDMAError(heth);
            if (dma_error & ETH_DMA_RX_WATCHDOG_TIMEOUT_FLAG)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA ETH_DMA_RX_WATCHDOG_TIMEOUT_FLAG\n");
            else if(dma_error & ETH_DMA_RX_PROCESS_STOPPED_FLAG)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA ETH_DMA_RX_PROCESS_STOPPED_FLAG\n");
            else if(dma_error & ETH_DMA_RX_BUFFER_UNAVAILABLE_FLAG)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA ETH_DMA_RX_BUFFER_UNAVAILABLE_FLAG\n");
            else if(dma_error & ETH_DMA_EARLY_TX_IT_FLAG)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA ETH_DMA_EARLY_TX_IT_FLAG\n");
            else if(dma_error & ETH_DMA_TX_PROCESS_STOPPED_FLAG)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA ETH_DMA_TX_PROCESS_STOPPED_FLAG\n");
            else if(dma_error & ETH_DMA_TX_BUFFER_UNAVAILABLE_IT)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA ETH_DMA_TX_BUFFER_UNAVAILABLE_IT\n");
		   
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_DMA\n");
        }
            break;
        case HAL_ETH_ERROR_MAC:
            uint32_t mac_error = HAL_ETH_GetMACError(heth);
            if (mac_error & ETH_RECEIVE_WATCHDOG_TIMEOUT)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_RECEIVE_WATCHDOG_TIMEOUT\n");
            else if (mac_error & ETH_EXECESSIVE_COLLISIONS)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_EXECESSIVE_COLLISIONS\n");
            else if (mac_error & ETH_LATE_COLLISIONS)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_LATE_COLLISIONS\n");
            else if (mac_error & ETH_EXECESSIVE_DEFERRAL)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_EXECESSIVE_DEFERRAL\n");
            else if (mac_error & ETH_TRANSMIT_JABBR_TIMEOUT)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_TRANSMIT_JABBR_TIMEOUT\n");
            else if (mac_error & ETH_LOSS_OF_CARRIER)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_LOSS_OF_CARRIER\n");
            else if (mac_error & ETH_NO_CARRIER)
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC ETH_NO_CARRIER\n");

            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, EIO), "Error from ethernet HAL (HAL_ETH_ErrorCallback) HAL_ETH_ERROR_MAC\n");
            break;
        //case HAL_ETH_ERROR_INVALID_CALLBACK:
            //break;
    }
    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_ETHERNET, error_code), "Error from ethernet HAL (HAL_ETH_ErrorCallback)\n");

	//dev_data->stats.error_details.rx_crc_errors = heth->Instance->MMCRCRCEPR;
	//dev_data->stats.error_details.rx_align_errors = heth->Instance->MMCRAEPR;

}


static bool eth_is_ptp_pkt(struct net_if *iface, struct net_pkt *pkt)
{
    tr_info("eth_is_ptp_pkt");
	// if (ntohs(NET_ETH_HDR(pkt)->type) != NET_ETH_PTYPE_PTP) {
	// 	return false;
	// }

	// net_pkt_set_priority(pkt, NET_PRIORITY_CA);

	return true;
}

void HAL_ETH_TxPtpCallback(uint32_t *buff, ETH_TimeStampTypeDef *timestamp)
{
    tr_info("HAL_ETH_TxPtpCallback");

	// struct eth_stm32_tx_context *ctx = (struct eth_stm32_tx_context *)buff;

	// ctx->pkt->timestamp.second = timestamp->TimeStampHigh;
	// ctx->pkt->timestamp.nanosecond = timestamp->TimeStampLow;

	// net_if_add_tx_timestamp(ctx->pkt);
}






#endif //ETH_IP_VERSION_V3
#endif /* DEVICE_EMAC */