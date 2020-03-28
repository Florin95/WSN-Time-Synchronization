#include "SPIMaster.h"
#include "SpiDma.h"
#include "cyhal.h"

const cy_stc_scb_spi_config_t mSPI_config =
{
	.spiMode = CY_SCB_SPI_MASTER,
	.subMode = CY_SCB_SPI_MOTOROLA,
	.sclkMode = CY_SCB_SPI_CPHA1_CPOL0,
	.oversample = 2,
	.rxDataWidth = 8UL,
	.txDataWidth = 8UL,
	.enableMsbFirst = true,
	.enableInputFilter = false,
	.enableFreeRunSclk = false,
	.enableMisoLateSample = true,
	.enableTransferSeperation = false,
	.ssPolarity = CY_SCB_SPI_ACTIVE_LOW,
	.rxFifoTriggerLevel = 27UL,
	.rxFifoIntEnableMask = 0UL,
	.txFifoTriggerLevel = 27UL,
	.txFifoIntEnableMask = 0UL,
	.masterSlaveIntEnableMask = 0UL,
};

extern volatile uint32_t txDmaDone;
/* Allocate context for SPI operation */
extern cy_stc_scb_spi_context_t spiContext;

/******************************************************************************
* This function initializes the SPI Master based on the
* configuration done in design.modus file.
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
	cyhal_gpio_init(ADS1298_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);

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
    Cy_SysClk_PeriphSetDivider   (SPI_CLK_DIV_TYPE, SPI_CLK_DIV_NUM, 1UL);
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
* This function sends the data to the slave. Note that the below
* function is blocking until all the bytes are transferred.
******************************************************************************/
void sendPacket(void)
{
	/* Enable DMA channel to transfer data from txBuffer into mSPI TX-FIFO */
	Cy_DMA_Channel_Enable(txDma_HW, txDma_CHANNEL);
}

/******************************************************************************
* This function checks for master transfer completion status.
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

/**
 * Send SPI data using the CPU.
 * @param tr_buf
 * @param buf_size
 * @param chunk_size
 * @param command_delay_us
 */
void send_command(uint8_t* tr_buf, uint32_t buf_size, uint32_t chunk_size, uint32_t command_delay_us)
{
	if (buf_size % chunk_size != 0)
	{
		// ERROR
		CY_ASSERT(0);
	}
	else
	{
		/* Wait for at least t_CSSC and set CS HIGH */
		cyhal_gpio_write(ADS1298_CS, false);
		Cy_SysLib_DelayUs(10);

		/* Send multibyte command */
		for(int i = 0; i < buf_size; i += chunk_size)
		{
			/* Don't put a delay before the first command word */
			if (i != 0)
			{
				/* Set delay between command words */
				Cy_SysLib_DelayUs(command_delay_us);
			}

			Cy_SCB_SPI_WriteArrayBlocking(mSPI_HW, tr_buf + i, chunk_size);
			/* Blocking wait for transfer completion */
			while (!Cy_SCB_SPI_IsTxComplete(mSPI_HW)) {}
		}

		/* Wait for at least t_SCCS and set CS HIGH */
		Cy_SysLib_DelayUs(10);
		cyhal_gpio_write(ADS1298_CS, true);
	}
}
