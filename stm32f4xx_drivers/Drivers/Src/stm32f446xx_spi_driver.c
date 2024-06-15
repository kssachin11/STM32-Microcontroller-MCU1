/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jun 14, 2024
 *      Author: Sachin K S
 */


#include "stm32f446xx_spi_driver.h"
//#include "stm32f446xx.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();

		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();

		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();

		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(EnorDi == DISABLE)
		{

			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DN();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_DN();
			}
		}
	}
}
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - look to congif structure

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg = pSPIHandle->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR;

	//2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<< SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<< SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RxONLY)
	{
		//BIDI mode should be cleared

		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
		tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	//4.  Configure the DFF
		tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
		tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

		tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


		pSPIHandle->pSPIx->CR1 = tempreg;
}
