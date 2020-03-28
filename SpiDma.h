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
#define RX_DMA_DESCR1       (1u)
#define RX_DMA_DESCR2       (2u)
#define RX_DMA_DESCR3       (3u)
#define RX_DMA_DESCR4       (4u)

#define RX_DMA_NUM          (5)

#define ADS1298_BYTES_PER_FRAME         (27)
#define ADS1298_NR_OF_SAMPLES_TO_BUFFER (50)
#define ADS1298_BUF_CAPACITY            (ADS1298_BYTES_PER_FRAME * ADS1298_NR_OF_SAMPLES_TO_BUFFER)

#define CLIENT_TASK_Q_TICKS_TO_TIMEOUT (100)

/************************ Function Prototypes ********************************/
uint32_t ConfigureTxDma(uint8_t* txBuffer);
void TxDmaComplete(void);
void ConfigureRxDma(uint8_t* rxBuffer);
void RxDmaComplete(void);

/* Data structure to TCP data and data length */
typedef struct
{
	uint8_t timestamp;
    uint8_t data[27];
}tcp_data_packet_t;


extern volatile uint32_t txDmaDone;
extern volatile uint8_t rx_dma_done;
extern volatile uint8_t activeDescr;
extern volatile QueueHandle_t tcp_client_queue;
extern volatile bool tcp_task_started;

#endif /* SOURCE_SPIDMA_H_ */
