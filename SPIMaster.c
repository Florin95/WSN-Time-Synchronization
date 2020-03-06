/******************************************************************************
* File Name: SPIMaster.c
*
* Description: 	This file contains function definitions for SPI Master.
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
#include "SPIMaster.h"

/* Assign pins for SPI on SCB1: P10[0], P10[1], P10[2] and P10[3] */
#define SPI_PORT        P10_0_PORT
#define SPI_MISO_NUM    P10_1_NUM
#define SPI_MOSI_NUM    P10_0_NUM
#define SPI_SCLK_NUM    P10_2_NUM
#define SPI_SS_NUM      P10_3_NUM

/* Assign divider type and number for SPI */
#define SPI_CLK_DIV_TYPE    (CY_SYSCLK_DIV_8_BIT)
#define SPI_CLK_DIV_NUM     (0U)

const cy_stc_scb_spi_config_t mSPI_config =
{
	.spiMode = CY_SCB_SPI_MASTER,
	.subMode = CY_SCB_SPI_MOTOROLA,
	.sclkMode = CY_SCB_SPI_CPHA0_CPOL1,
	.oversample = 16,
	.rxDataWidth = 8UL,
	.txDataWidth = 8UL,
	.enableMsbFirst = true,
	.enableInputFilter = false,
	.enableFreeRunSclk = false,
	.enableMisoLateSample = true,
	.enableTransferSeperation = false,
	.ssPolarity = ((CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT0) | \
                                         (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT1) | \
                                         (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT2) | \
                                         (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT3)),
	.enableWakeFromSleep = false,
	.rxFifoTriggerLevel = 63UL,
	.rxFifoIntEnableMask = 0UL,
	.txFifoTriggerLevel = 63UL,
	.txFifoIntEnableMask = 0UL,
	.masterSlaveIntEnableMask = 0UL,
};

extern volatile uint32_t txDmaDone;
/* Allocate context for SPI operation */
extern cy_stc_scb_spi_context_t spiContext;
/******************************************************************************
* Function Name: initMaster
*******************************************************************************
*
* Summary: 		This function initializes the SPI Master based on the
* 				configuration done in design.modus file.
*
* Parameters: 	None
*
* Return:		(uint32) INIT_SUCCESS or INIT_FAILURE
*
******************************************************************************/
uint32 initMaster(void)
{
    cy_en_scb_spi_status_t initStatus;

	/* Configure SPI block */
	initStatus = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &spiContext);

	/* Connect SCB1 SPI function to pins */
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_MISO_NUM, P10_1_SCB1_SPI_MISO);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_MOSI_NUM, P10_0_SCB1_SPI_MOSI);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_SCLK_NUM, P10_2_SCB1_SPI_CLK);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_SS_NUM,   P10_3_SCB1_SPI_SELECT0);

    /* Configure SCB1 pins for SPI Master operation */
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_MISO_NUM, CY_GPIO_DM_HIGHZ);
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_MOSI_NUM, CY_GPIO_DM_STRONG_IN_OFF);
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_SCLK_NUM, CY_GPIO_DM_STRONG_IN_OFF);
    Cy_GPIO_SetDrivemode(SPI_PORT, SPI_SS_NUM,   CY_GPIO_DM_STRONG_IN_OFF);

    /* Connect assigned divider to be a clock source for SPI */
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB1_CLOCK, SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM);

    /* SPI master desired data rate is 1 Mbps.
    * The SPI master data rate = (clk_scb / Oversample).
    * For clk_peri = 50 MHz, select divider value 5 and get SCB clock = (50 MHz / 5) = 10 MHz.
    * Select Oversample = 10. These setting results SPI data rate = 10 MHz / 10 = 1 Mbps.
    */
    Cy_SysClk_PeriphSetDivider   (SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM, 4UL);
    Cy_SysClk_PeriphEnableDivider(SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM);

	/* If the initialization fails, return failure status */
	if(initStatus != CY_SCB_SPI_SUCCESS)
	{
		return(INIT_FAILURE);
	}

	/* Set active slave select to line 0 */
	Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

	/* Enable SPI master block. */
	Cy_SCB_SPI_Enable(mSPI_HW);

	/* Initialization completed */

	return(INIT_SUCCESS);
}


/******************************************************************************
* Function Name: sendPacket
*******************************************************************************
*
* Summary: 		This function sends the data to the slave. Note that the below
* 				function is blocking until all the bytes are transferred.
*
* Parameters:  	None
*
* Return:		None
*
******************************************************************************/
void sendPacket(void)
{
	/* Enable DMA channel to transfer 12 bytes of data from txBuffer into mSPI TX-FIFO */
	//Cy_DMA_Channel_Enable(txDma_HW, txDma_CHANNEL);
}

/******************************************************************************
* Function Name: checkTranferStatus
*******************************************************************************
*
* Summary: 		This function checks for master transfer completion status
*
* Parameters:  	None
*
* Return:		Status of transfer completion
*
******************************************************************************/
uint32 checkTranferStatus(void)
{
	volatile uint32 masterStatus;
	uint32 transferStatus;
    /* Wait until master complete the transfer */
	do
	{
		masterStatus  = Cy_SCB_SPI_GetSlaveMasterStatus(mSPI_HW);

	} while ((0UL == (masterStatus & (CY_SCB_SPI_MASTER_DONE | CY_SCB_SPI_SLAVE_ERR))) || txDmaDone == 0);

    txDmaDone = 0;
	/* Check for any errors */
	if(CY_SCB_SPI_MASTER_DONE & masterStatus)
	{
		/* No error */
		transferStatus = TRANSFER_COMPLETE;
	}
	else
	{
		/* Error encountered in the transfer */
		transferStatus = TRANSFER_FAILURE;
	}
	/* Clear the SPI master status */
	Cy_SCB_SPI_ClearSlaveMasterStatus(mSPI_HW, masterStatus);

	return(transferStatus);
}

