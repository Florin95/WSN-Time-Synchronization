#ifndef SOURCE_SPIMASTER_H_
#define SOURCE_SPIMASTER_H_

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
void sendPacket(uint8_t *txBuffer);
void AssignAndConfigurePins();
uint32 checkTranferStatus(void);


#endif /* SOURCE_SPIMASTER_H_ */
