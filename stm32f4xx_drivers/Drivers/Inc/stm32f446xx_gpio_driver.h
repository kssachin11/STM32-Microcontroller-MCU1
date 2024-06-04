/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jun 4, 2024
 *      Author: Sachin K S
 */
// contains driver specific data
#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_
#include "stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_Pin0PType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t ;
typedef struct
{// pointer to hold the base address of GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // holds the base address of the GPIO port to which the pin belongs

	GPIO_PinConfig_t GPIO_PinConfig;// this holds GPIO pin configuration settings
}GPIO_Handle_t;

/*********************************************************************************************************************************************
 *                                       API's supported by this driver
 *                                       For more information about the API's check the function defenitions
 *
 *********************************************************************************************************************************************/
/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);  // API for initializing GPIO port
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Peri clock setup
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * Data Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);  // When interrupt occurs,this function can be called to process the interrupt



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
