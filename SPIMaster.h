#ifndef SPIMASTER_H_
#define SPIMASTER_H_

#include "cy_pdl.h"
#include "cycfg.h"

/***************************************
*                Macros
****************************************/
#define MASTER_ERROR_MASK  (CY_SCB_SPI_SLAVE_TRANSFER_ERR  | CY_SCB_SPI_TRANSFER_OVERFLOW    | \
                            CY_SCB_SPI_TRANSFER_UNDERFLOW)

/* Assign pins for SPI on SCB1: P10[0], P10[1], P10[2] and P10[3] */
#define SPI_PORT        P10_0_PORT
#define SPI_MOSI_NUM    P10_0_NUM
#define SPI_MISO_NUM    P10_1_NUM
#define SPI_SCLK_NUM    P10_2_NUM
#define SPI_SS_NUM      P10_3_NUM

#define ADC_CS      (P9_4)

/* Assign divider type and number for SPI */
#define SPI_CLK_DIV_TYPE    (CY_SYSCLK_DIV_8_BIT)
#define SPI_CLK_DIV_NUM     (0U)

/***************************************
*         Function Prototypes
****************************************/
uint32 initMaster(void);
void sendPacket(void);
uint32 checkTranferStatus(void);
void send_command(uint8_t* tr_buf, uint32_t buf_size, uint32_t transmit_size, uint32_t command_delay_us);

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
