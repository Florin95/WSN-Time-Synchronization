#ifndef SPIMASTER_H_
#define SPIMASTER_H_

#include "cy_pdl.h"
#include "cycfg.h"

/***************************************
*                Macros
****************************************/
#define MASTER_ERROR_MASK  (CY_SCB_SPI_SLAVE_TRANSFER_ERR  | CY_SCB_SPI_TRANSFER_OVERFLOW    | \
                            CY_SCB_SPI_TRANSFER_UNDERFLOW)

/***************************************
*         Function Prototypes
****************************************/
uint32 initMaster(void);
void sendPacket(void);
uint32 checkTranferStatus(void);

#define mSPI_ENABLED 1U
#define mSPI_HW SCB1
#define mSPI_IRQ scb_1_interrupt_IRQn

/* Initialization status */
#define INIT_SUCCESS			(0)
#define INIT_FAILURE 			(1)

/* Communication status */
#define TRANSFER_COMPLETE 		(0)
#define TRANSFER_FAILURE		(1)
#define TRANSFER_IN_PROGRESS	(2)
#define IDLE					(3)

cy_stc_scb_spi_context_t spiContext;

#endif /* SPIMASTER_H_ */
