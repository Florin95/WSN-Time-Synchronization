#ifndef SOURCE_SPIDMA_H_
#define SOURCE_SPIDMA_H_

#include "cy_pdl.h"
#include "cycfg.h"

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

/************************ Function Prototypes ********************************/
uint32_t ConfigureTxDma(uint8_t* txBuffer);
void TxDmaComplete(void);
void ConfigureRxDma(uint8_t* rxBuffer);
void RxDmaComplete(void);

extern volatile uint32_t txDmaDone;
extern volatile uint8_t rx_dma_done;
extern volatile uint8_t activeDescr;


#endif /* SOURCE_SPIDMA_H_ */
