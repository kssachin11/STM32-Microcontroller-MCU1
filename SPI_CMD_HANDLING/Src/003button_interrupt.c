/*
 * 003button_interrupt.c
 *
 *  Created on: Jun 11, 2024
 *      Author: Sachin K S
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f446xx_gpio_driver.h"

#define BTN_PRESSED LOW

#define LOW 0
void delay(void)
{
	// this will introduce 200 ms delys when system clock is 16MHz
	for(uint32_t i =0; i<50000/2 ;i++);  // implementing software delay
}
int main(void)
{
	GPIO_Handle_t Gpio_Led, Gpio_Btn; // creating a varible to handle led

	memset(&Gpio_Led,0,sizeof(Gpio_Led));
	memset(&Gpio_Btn,0,sizeof(Gpio_Btn));

	Gpio_Led.pGPIOx = GPIOA ; // selecting the port of led PA5
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;  // selecting pin number
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;		// selecting pin mode
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Selecting speed
	Gpio_Led.GPIO_PinConfig.GPIO_Pin0PType = GPIO_OP_TYPE_PP; // output type
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  //selecting pin push pull

	GPIO_PeriClockControl(GPIOA,ENABLE);  // enabling peripheral clk
	GPIO_Init(&Gpio_Led);

	// creating variable for button handling
	Gpio_Btn.pGPIOx = GPIOC ; // selecting the port of led PA5
	Gpio_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // selecting pin number
	Gpio_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;		// selecting pin mode
	Gpio_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Selecting speed
	Gpio_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  //selecting pin push pull
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET);
	GPIO_PeriClockControl(GPIOA,ENABLE);  // enabling peripheral clk
	GPIO_Init(&Gpio_Btn);


	//IRQ config

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	while(1);

}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
