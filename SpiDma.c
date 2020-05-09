#include "SpiDma.h"
#include "SPIMaster.h"
#include "cyhal.h"
#include "types.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/******************************************************************************
* TX DMA
******************************************************************************/
cy_stc_dma_descriptor_t txDma_Descriptor_0 =
{
	.ctl = 0UL,
	.src = 0UL,
	.dst = 0UL,
	.xCtl = 0UL,
	.yCtl = 0UL,
	.nextPtr = 0UL,
};
const cy_stc_dma_channel_config_t txDma_channelConfig =
{
	.descriptor = &txDma_Descriptor_0,
	.preemptable = false,
	.priority = 3,
	.enable = false,
	.bufferable = false,
};

const cy_stc_dma_descriptor_config_t txDma_Descriptor_0_config =
{
	.retrigger = CY_DMA_RETRIG_IM,
	.interruptType = CY_DMA_X_LOOP,
	.triggerOutType = CY_DMA_1ELEMENT,
	.channelState = CY_DMA_CHANNEL_DISABLED,
	.triggerInType = CY_DMA_1ELEMENT,
	.dataSize = CY_DMA_BYTE,
	.srcTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
	.dstTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
	.descriptorType = CY_DMA_1D_TRANSFER,
	.srcAddress = NULL,
	.dstAddress = NULL,
	.srcXincrement = 1,
	.dstXincrement = 0,
	.xCount = 27,
	.srcYincrement = 0,
	.dstYincrement = 0,
	.yCount = 1,
	.nextDescriptor = &txDma_Descriptor_0,
};

volatile uint32_t txDmaDone = 0;

uint32_t ConfigureTxDma(uint8_t* txBuffer)
 {
     cy_en_dma_status_t dma_init_status;
     const cy_stc_sysint_t intTxDma_cfg =
     {
         .intrSrc      = txDma_IRQ,
         .intrPriority = 7u
     };
     /* Initialize descriptor */
     dma_init_status = Cy_DMA_Descriptor_Init(&txDma_Descriptor_0, &txDma_Descriptor_0_config);
     if(dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     dma_init_status = Cy_DMA_Channel_Init(txDma_HW, txDma_CHANNEL, &txDma_channelConfig);
     if(dma_init_status!=CY_DMA_SUCCESS)
     {
    	 return INIT_FAILURE;
     }

     /* Set source and destination for descriptor 1 */
     Cy_DMA_Descriptor_SetSrcAddress(&txDma_Descriptor_0, (uint8_t *)txBuffer);
     Cy_DMA_Descriptor_SetDstAddress(&txDma_Descriptor_0, (void *)&mSPI_HW->TX_FIFO_WR);

      /* Initialize and enable the interrupt from TxDma */
     Cy_SysInt_Init(&intTxDma_cfg, &TxDmaComplete);
     NVIC_EnableIRQ((IRQn_Type)intTxDma_cfg.intrSrc);

      /* Enable DMA interrupt source. */
     Cy_DMA_Channel_SetInterruptMask(txDma_HW, txDma_CHANNEL, CY_DMA_INTR_MASK);

     Cy_DMA_Enable(txDma_HW);
     return INIT_SUCCESS;
 }

void TxDmaComplete(void)
 {
     /* Check tx DMA status */
     if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_CHANNEL)) &&
         (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus(txDma_HW, txDma_CHANNEL)))
     {
         /* DMA error occurred while TX operations */
    	 CY_ASSERT(0);
     }

     txDmaDone=1;

     /* Clear tx DMA interrupt */
     Cy_DMA_Channel_ClearInterrupt(txDma_HW, txDma_CHANNEL);
 }

/******************************************************************************
* RX DMA Descriptors
******************************************************************************/
cy_stc_dma_descriptor_t RxDma_Descriptor_0 =
{
	.ctl = 0UL,
	.src = 0UL,
	.dst = 0UL,
	.xCtl = 0UL,
	.yCtl = 0UL,
	.nextPtr = 0UL,
};

/******************************************************************************
* RX DMA Channel Config
******************************************************************************/
const cy_stc_dma_channel_config_t RxDma_channelConfig =
{
	.descriptor = &RxDma_Descriptor_0,
	.preemptable = false,
	.priority = 3,
	.enable = false,
	.bufferable = false,
};

/******************************************************************************
* RX DMA Descriptor Configs
******************************************************************************/
const cy_stc_dma_descriptor_config_t RxDma_Descriptor_0_config =
{
	.retrigger = CY_DMA_RETRIG_IM,
	.interruptType = CY_DMA_DESCR,
	.triggerOutType = CY_DMA_1ELEMENT,
	.channelState = CY_DMA_CHANNEL_ENABLED,
	.triggerInType = CY_DMA_1ELEMENT,
	.dataSize = CY_DMA_BYTE,
	.srcTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
	.dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
	.descriptorType = CY_DMA_1D_TRANSFER,
	.srcAddress = NULL,
	.dstAddress = NULL,
	.srcXincrement = 0,
	.dstXincrement = 1,
	.xCount = 27,
	.srcYincrement = 0,
	.dstYincrement = 0,
	.yCount = 1,
	.nextDescriptor = &RxDma_Descriptor_0,
};

volatile uint8_t rx_dma_done;     /* RxDma done flag */
volatile uint8_t rx_dma_error;    /* RxDma error flag */
volatile tcp_data_packet_t tcp_packet;

void ConfigureRxDma()
{
	cy_en_dma_status_t dma_init_status;
    cy_stc_sysint_t intConfig =
    {
    	.intrSrc      = (IRQn_Type)RxDma_IRQ,
    	.intrPriority = 7u,
    };

    /* Initialize descriptor 1 */
    dma_init_status = Cy_DMA_Descriptor_Init(&RxDma_Descriptor_0, &RxDma_Descriptor_0_config);
	if(dma_init_status != CY_DMA_SUCCESS)
    {
        CY_ASSERT(0);
    }

    dma_init_status = Cy_DMA_Channel_Init(RxDma_HW, RxDma_CHANNEL, &RxDma_channelConfig);
	if(dma_init_status != CY_DMA_SUCCESS)
    {
        // ERROR
    }

    /* Set source and destination address for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&RxDma_Descriptor_0, (void *) &mSPI_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&RxDma_Descriptor_0, (uint8_t *) tcp_packet.data);

    Cy_DMA_Channel_SetDescriptor(RxDma_HW, RxDma_CHANNEL, &RxDma_Descriptor_0);

    /* Initialize and enable interrupt from RxDma */
    Cy_SysInt_Init(&intConfig, &RxDmaComplete);
    NVIC_EnableIRQ(intConfig.intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(RxDma_HW, RxDma_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable channel and DMA block to start descriptor execution process */
    Cy_DMA_Channel_Enable(RxDma_HW, RxDma_CHANNEL);
    Cy_DMA_Enable(RxDma_HW);
}

void RxDmaComplete(void)
{
    Cy_DMA_Channel_ClearInterrupt(RxDma_HW, RxDma_CHANNEL);

	BaseType_t xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;

	if(tcp_task_started)
	{
		/* Send TCP data packet to the tcp_client_task */
		xQueueSendToBackFromISR(tcp_client_queue, &tcp_packet, &xHigherPriorityTaskWoken);
	}

    /* Check interrupt cause to capture errors. */
    if(CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(RxDma_HW, RxDma_CHANNEL))
    {
	    /* Wait for at least t_CSSC and set CS HIGH */
		cyhal_gpio_write(ADC_CS, true);
		rx_dma_done = 1;
	}
	else
	{
		/* DMA error occurred while RX operations */
		rx_dma_error = 1;
		CY_ASSERT(0);
	}

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

