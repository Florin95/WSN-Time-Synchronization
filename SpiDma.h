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

/************************ Function Prototypes ********************************/

uint32_t ConfigureTxDma(uint8_t* txBuffer);
void TxDmaComplete(void);
void ConfigureRxDma(uint8_t* rxBuffer);
void RxDmaComplete(void);

#endif /* SOURCE_SPIDMA_H_ */
