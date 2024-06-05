/*
 * 001led_toggle.c
 *
 *  Created on: Jun 5, 2024
 *      Author: Sachin K S
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i =0; i<500000/2 ;i++);  // implementing software delay
}
int main(void)
{
	GPIO_Handle_t Gpio_Led ; // creating a varible to handle led
	Gpio_Led.pGPIOx = GPIOA ; // selecting the port of led PA5
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;  // selecting pin number
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;		// selecting pin mode
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Selecting speed
	Gpio_Led.GPIO_PinConfig.GPIO_Pin0PType = GPIO_OP_TYPE_PP; // output type
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  //selecting pin push pull

	GPIO_PeriClockControl(GPIOA, ENABLE);  // enabling peripheral clk
	GPIO_Init(&Gpio_Led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
