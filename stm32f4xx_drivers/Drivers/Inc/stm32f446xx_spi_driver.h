/*
 * stm32f445mm_spi_driver.h
 *
 *  Created on: Jun 14, 2024
 *      Author: Sachin K S
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_
#include "stm32f446xx.h"

/*
 * Configure structure for SPIx peripherals
 */


typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 *
 */

typedef struct
{

	SPI_RegDef	*pSPIx;
	SPI_Config_t	SPIConfig;

}SPI_Handle_t;
#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */


/*
 * Init and Deinit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);  // API for initializing GPIO port
void GPIO_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Peripheral Clock setup
 */


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(uint8_t PinNumber);  // When interrupt occurs,this function can be called to process the interrupt
void SPI_IRQPriorityConfig(SPI_Handle_t *pHandle);

