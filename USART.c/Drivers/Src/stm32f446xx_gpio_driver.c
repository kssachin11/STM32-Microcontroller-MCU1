/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Jun 4, 2024
 *      Author: Sachin K S
 */


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */


/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// API for initializing GPIO port

	uint32_t temp = 0; //temp.register
	// enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin
	if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode << (2*(pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle ->pGPIOx-> MODER &= ~(0x3 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle ->pGPIOx-> MODER |= temp;

	}else
	{
		 // GPIO PIN INTERRUPT CONFIG
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode== GPIO_MODE_IT_FT)
		{
			//1. Config the FTSR
			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding RTSR bit -- Riding Edge

			EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			//1. Config RTSR
			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding RTSR bit -- Riding Edge

			EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
		{
			//1. Config FTSR and RTSR

			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Config the GPIO port selection in SYSCFG_EXTICR

			uint8_t temp1 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//3. Enable the EXTI interrupt delivery using IMR

		EXTI->IMR |= 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}
	temp = 0;


	//2. configure the speed
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	//3. configure the pupd settings
	temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x1 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;


	//4. configure the optype
	temp =(pGPIOHandle->GPIO_PinConfig.GPIO_Pin0PType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER&= ~(0x1 <<pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// config alternate function
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; /* for choosing high or low register*/
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<< (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2);

	}
}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx==GPIOA)
			{
				GPIOA_REG_RESET();  // clock enable macro
			}else if(pGPIOx==GPIOB)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOC)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOD)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOE)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOF)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOG)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOH)
			{
				GPIOA_REG_RESET();
			}

}


/*
 * Peri clock setup
 */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx==GPIOA)
		{
			GPIOA_PERI_CLOCK_ENABLE();  // clock enable macro
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PERI_CLOCK_ENABLE();
		}else if(pGPIOx==GPIOC)
		{
			GPIOC_PERI_CLOCK_ENABLE();
		}else if(pGPIOx==GPIOD)
		{
			GPIOD_PERI_CLOCK_ENABLE();
		}else if(pGPIOx==GPIOE)
		{
			GPIOE_PERI_CLOCK_ENABLE();
		}else if(pGPIOx==GPIOF)
		{
			GPIOF_PERI_CLOCK_ENABLE();
		}else if(pGPIOx==GPIOG)
		{
			GPIOG_PERI_CLOCK_ENABLE();
		}else if(pGPIOx==GPIOH)
		{
			GPIOH_PERI_CLOCK_ENABLE();
		}
	}else

	{
		if (pGPIOx==GPIOA)
			{
				GPIOA_PERI_CLOCK_DISABLE();  // clock enable macro
			}else if(pGPIOx==GPIOB)
			{
				GPIOB_PERI_CLOCK_DISABLE();
			}else if(pGPIOx==GPIOC)
			{
				GPIOC_PERI_CLOCK_DISABLE();
			}else if(pGPIOx==GPIOD)
			{
				GPIOD_PERI_CLOCK_DISABLE();
			}else if(pGPIOx==GPIOE)
			{
				GPIOE_PERI_CLOCK_DISABLE();
			}else if(pGPIOx==GPIOF)
			{
				GPIOF_PERI_CLOCK_DISABLE();
			}else if(pGPIOx==GPIOG)
			{
				GPIOG_PERI_CLOCK_DISABLE();
			}else if(pGPIOx==GPIOH)
			{
				GPIOH_PERI_CLOCK_DISABLE();
			}
	}

}



/*
 * Data Read and write
 */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001) ;  // bitwise right shift and masking to get the value of lsb,making every bit 0 except lsb

	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
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

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR ;
	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
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

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		{
			//write 1 to the output data register at the bit field corresponding to the pin number
			pGPIOx->ODR |= ( 1 << PinNumber);
		}else
		{
			//write 0
			pGPIOx->ODR &= ~( 1 << PinNumber);
		}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<< PinNumber); // USING XOR FOR TOGGLING
}


/*
 * IRQ Configuration and ISR handling
 */

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + (iprx)) |=  ( IRQPriority << shift_amount );

}


void GPIO_IRQHandling(uint8_t PinNumber)
{

	// When interrupt occurs,this function can be called to process the interrupt

	//clear the exti pr register corresponding to the pin number
		if(EXTI->PR & ( 1 << PinNumber))
		{
			//clear
			EXTI->PR |= ( 1 << PinNumber);
		}
}
