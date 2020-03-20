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
extern volatile uint16_t rxBuffer_WP;
extern volatile uint16_t rxBuffer_RP;
extern volatile bool tcp_task_started;
extern uint8_t SPI_receive_data[ADS1298_BYTES_PER_FRAME * ADS1298_NR_OF_SAMPLES_TO_BUFFER];

extern cy_stc_dma_descriptor_t RxDma_Descriptor_0;

#endif /* SOURCE_SPIDMA_H_ */
