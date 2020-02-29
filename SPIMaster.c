#include "SPIMaster.h"

#include "Interface.h"

/* Assign pins for SPI on SCB1: P10[0], P10[1], P10[2] and P10[3] */
// !!!!!!!!!! MAPPING FROM DOCS IS ERRONEOUS! (MISO AND MOSI ARE INTERCHANGED)
#define SPI_PORT        P9_0_PORT
#define SPI_MOSI_NUM    P9_0_NUM
#define SPI_MISO_NUM    P9_1_NUM
#define SPI_SCLK_NUM    P9_2_NUM
#define SPI_SS_NUM      P9_3_NUM

/* Assign divider type and number for SPI */
#define SPI_CLK_DIV_TYPE    (CY_SYSCLK_DIV_8_BIT)
#define SPI_CLK_DIV_NUM     (5U)

/* Allocate context for SPI operation */
cy_stc_scb_spi_context_t spiContext;

const cy_stc_scb_spi_config_t Master_SPI_config =
{
	.spiMode  = CY_SCB_SPI_MASTER,
	.subMode  = CY_SCB_SPI_MOTOROLA,
	.sclkMode = CY_SCB_SPI_CPHA0_CPOL0,
	.oversample = 2UL,

	.rxDataWidth = 8UL,
	.txDataWidth = 8UL,
	.enableMsbFirst           = true,
	.enableInputFilter        = false,
	.enableFreeRunSclk        = false,
	.enableMisoLateSample     = true,
	.enableTransferSeperation = false,
	.ssPolarity = ((CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT0) | \
				   (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT1) | \
				   (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT2) | \
				   (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT3)),
	.enableWakeFromSleep = false,

	.rxFifoTriggerLevel  = 0UL,
	.rxFifoIntEnableMask = 0UL,
	.txFifoTriggerLevel  = 0UL,
	.txFifoIntEnableMask = 0UL,
	.masterSlaveIntEnableMask = 0UL,
};

extern volatile uint32_t txDmaDone;

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
	initStatus = Cy_SCB_SPI_Init(mSPI_HW, &Master_SPI_config, &spiContext);
	/* If the initialization fails, return failure status */
	if(initStatus != CY_SCB_SPI_SUCCESS)
	{
		return(INIT_FAILURE);
	}

	AssignAndConfigurePins();

	/* Connect assigned divider to be a clock source for SPI */
	Cy_SysClk_PeriphAssignDivider(PCLK_SCB2_CLOCK, SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM);

	/* SPI master desired data rate is 1 Mbps.
	* The SPI master data rate = (clk_scb / Oversample).
	* For clk_peri = 50 MHz, select divider value 5 and get SCB clock = (50 MHz / 5) = 10 MHz.
	* Select Oversample = 10. These setting results SPI data rate = 10 MHz / 10 = 1 Mbps.
	*/
	Cy_SysClk_PeriphSetDivider   (SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM, 4UL);
	Cy_SysClk_PeriphEnableDivider(SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM);

	/* Set active slave select to line 0 */
	Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT1);

	/* Enable SPI master block. */
	Cy_SCB_SPI_Enable(mSPI_HW);

	/* Initialization completed */

	return(INIT_SUCCESS);
}

/*Only dedicated SCB pins can be used for SPI operation. The HSIOM register must be
 * configured to connect dedicated SCB SPI pins to the SCB block. Also, the SPI output
 * pins must be configured in Strong Drive Input Off mode and SPI input pins in Digital High-Z.*/
void AssignAndConfigurePins()
{
	/* Connect SCB1 SPI function to pins */
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_MISO_NUM, P9_1_SCB2_SPI_MISO);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_MOSI_NUM, P9_0_SCB2_SPI_MOSI);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_SCLK_NUM, P9_2_SCB2_SPI_CLK);
	Cy_GPIO_SetHSIOM(SPI_PORT, SPI_SS_NUM,   P9_4_SCB2_SPI_SELECT1);

	/* Configure SCB1 pins for SPI Master operation */
	Cy_GPIO_SetDrivemode(SPI_PORT, SPI_MISO_NUM, CY_GPIO_DM_HIGHZ);
	Cy_GPIO_SetDrivemode(SPI_PORT, SPI_MOSI_NUM, CY_GPIO_DM_STRONG_IN_OFF);
	Cy_GPIO_SetDrivemode(SPI_PORT, SPI_SCLK_NUM, CY_GPIO_DM_STRONG_IN_OFF);
	Cy_GPIO_SetDrivemode(SPI_PORT, SPI_SS_NUM,   CY_GPIO_DM_STRONG_IN_OFF);
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
void sendPacket(uint32_t *txBuffer)
{
	/* Master: start a transfer. Slave: prepare for a transfer. */
	Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, txBuffer, sizeof(txBuffer));

	/* Blocking wait for transfer completion */
	while (!Cy_SCB_SPI_IsTxComplete(mSPI_HW))
	{
	}

//	Cy_SCB_SPI_Write(mSPI_HW, txBuffer[0]);

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
	// TODO: remove
	txDmaDone = 1;

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

