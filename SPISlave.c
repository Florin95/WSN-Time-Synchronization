/******************************************************************************
* File Name: SPISlave.c
*
* Description: This file contains function definitions for SPI Slave.
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "SPISlave.h"
#include <stdio.h>
#include "Interface.h"

/* Assign pins for SPI on SCB2: P9[0], P9[1], P9[2] and P9[3] */
#define SPI_PORT        P9_0_PORT
#define SPI_MOSI_NUM    P9_0_NUM
#define SPI_MISO_NUM    P9_1_NUM
#define SPI_SCLK_NUM    P9_2_NUM
#define SPI_SS_NUM      P9_3_NUM

/* Assign divider type and number for SPI */
#define SPI_CLK_DIV_TYPE    (CY_SYSCLK_DIV_8_BIT)
#define SPI_CLK_DIV_NUM     (5U)

const cy_stc_scb_spi_config_t sSPI_config =
{
	.spiMode = CY_SCB_SPI_SLAVE,
	.subMode = CY_SCB_SPI_MOTOROLA,
	.sclkMode = CY_SCB_SPI_CPHA0_CPOL0,
	.oversample = 0UL,
	.rxDataWidth = 8UL,
	.txDataWidth = 8UL,
	.enableMsbFirst = true,
	.enableInputFilter = false,
	.enableFreeRunSclk = false,
	.enableMisoLateSample = false,
	.enableTransferSeperation = false,
	.ssPolarity = ((CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT0) | \
				 (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT1) | \
				 (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT2) | \
				 (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT3)),
	.enableWakeFromSleep = false,
	.rxFifoTriggerLevel = 0UL,
	.rxFifoIntEnableMask = 0UL,
	.txFifoTriggerLevel = 0UL,
	.txFifoIntEnableMask = 0UL,
	.masterSlaveIntEnableMask = 0UL,
};

/*Only dedicated SCB pins can be used for SPI operation. The HSIOM register must be
 * configured to connect dedicated SCB SPI pins to the SCB block. Also, the SPI output
 * pins must be configured in Strong Drive Input Off mode and SPI input pins in Digital High-Z.*/
void AssignAndConfigurePinsSlave()
{
	/* Connect SCB2 SPI function to pins */
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_MISO_NUM, P9_1_SCB2_SPI_MISO);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_MOSI_NUM, P9_0_SCB2_SPI_MOSI);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_SCLK_NUM, P9_2_SCB2_SPI_CLK);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_SS_NUM,   P9_3_SCB2_SPI_SELECT0);

    /* Configure SCB1 pins for SPI Slave operation */
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_MISO_NUM, CY_GPIO_DM_STRONG_IN_OFF);
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_MOSI_NUM, CY_GPIO_DM_HIGHZ);
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_SCLK_NUM, CY_GPIO_DM_HIGHZ);
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_SS_NUM,   CY_GPIO_DM_HIGHZ);
}

/******************************************************************************
* Function Name: initSlave
*******************************************************************************
*
* Summary: 		This function initializes the SPI Slave based on the
* 				configuration done in design.modus file.
*
* Parameters: 	None
*
* Return:		(uint32) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32 initSlave(void)
{
	cy_stc_scb_spi_context_t sSPI_context;
	cy_en_scb_spi_status_t initStatus;

	/* Configure the SPI block */
	initStatus = Cy_SCB_SPI_Init(sSPI_HW, &sSPI_config, &sSPI_context);

	/* If the initialization fails, return failure status */
	if(initStatus != CY_SCB_SPI_SUCCESS)
	{
		return(INIT_FAILURE);
	}

	AssignAndConfigurePinsSlave();

	/* Connect assigned divider to be a clock source for SPI */
	Cy_SysClk_PeriphAssignDivider(PCLK_SCB2_CLOCK, SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM);

	/* SPI data rate is defined by the SPI master because it drives SCLK.
	* This clk_scb enables SPI slave operate up to maximum supported data rate.
	* For clk_peri = 50 MHz, select divider value 1 and get clk_scb = (50 MHz / 1) = 50 MHz.
	*/
	Cy_SysClk_PeriphSetDivider   (SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM, 0UL);
	Cy_SysClk_PeriphEnableDivider(SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM);

	/* Set active slave select to line 0 */
	Cy_SCB_SPI_SetActiveSlaveSelect(sSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

	/* Enable the SPI Slave block */
	Cy_SCB_SPI_Enable(sSPI_HW);

	/* Initialization completed */

	return(INIT_SUCCESS);
}


/******************************************************************************
* Function Name: readPacket
*******************************************************************************
*
* Summary: 		This function reads the data received by the slave. Note that
* 				the below function is blocking until the required number of
* 				bytes is received by the slave.
*
* Parameters:  	(uint32 *) rxBuffer - Pointer to the receive buffer where data
* 				needs to be stored
* 				(uint32)	 transferSize - Number of bytes to be received
*
* Return:		None
*
******************************************************************************/
uint32 readPacket(uint32 *rxBuffer, uint32 transferSize)
{
	uint32 slaveStatus;

	/* Wait till all the bytes are received */
    while(Cy_SCB_SPI_GetNumInRxFifo(sSPI_HW) != transferSize)
    {
    	if (Cy_SCB_SPI_GetNumInRxFifo(sSPI_HW)!= 0)
    		printf("Received = %lu\n", Cy_SCB_SPI_GetNumInRxFifo(sSPI_HW));
    }

	/* Read RX FIFO */
	Cy_SCB_SPI_ReadArray(sSPI_HW, rxBuffer, transferSize);

	/* Check start and end of packet markers */
	if ((rxBuffer[PACKET_SOP_POS] == PACKET_SOP) && (rxBuffer[PACKET_EOP_POS] == PACKET_EOP))
	{
		/* Data received correctly */
		slaveStatus = TRANSFER_COMPLETE;
	}
	else
	{
		/* Data was not received correctly */
		slaveStatus = TRANSFER_FAILURE;
	}

    return slaveStatus;
}

