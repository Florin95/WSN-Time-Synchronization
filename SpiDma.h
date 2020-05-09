#ifndef SOURCE_SPIDMA_H_
#define SOURCE_SPIDMA_H_

#include "cy_pdl.h"
#include "cycfg.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define txDma_ENABLED 1U
#define txDma_HW DW0
#define txDma_CHANNEL 2U
#define txDma_IRQ cpuss_interrupts_dw0_2_IRQn

#define RxDma_ENABLED 1U
#define RxDma_HW DW0
#define RxDma_CHANNEL 3U
#define RxDma_IRQ cpuss_interrupts_dw0_3_IRQn

#define RX_DMA_DESCR0       (0u)

#define ADC_BYTES_PER_FRAME         (27)

/************************ Function Prototypes ********************************/
uint32_t ConfigureTxDma(uint8_t* txBuffer);
void TxDmaComplete(void);
void ConfigureRxDma(void);
void RxDmaComplete(void);

extern volatile uint32_t txDmaDone;
extern volatile uint8_t rx_dma_done;
extern volatile QueueHandle_t tcp_client_queue;
extern volatile bool tcp_task_started;

#endif /* SOURCE_SPIDMA_H_ */
