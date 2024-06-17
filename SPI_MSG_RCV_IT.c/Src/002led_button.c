/*
 * 002led_button.c
 *
 *  Created on: Jun 6, 2024
 *      Author: Sachin K S
 */




#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"


#define BTN_PRESSED LOW

#define LOW 0
void delay(void)
{
	for(uint32_t i =0; i<50000/2 ;i++);  // implementing software delay
}
int main(void)
{
	GPIO_Handle_t Gpio_Led, Gpio_Btn; // creating a varible to handle led

	Gpio_Led.pGPIOx = GPIOA ; // selecting the port of led PA5
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;  // selecting pin number
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;		// selecting pin mode
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Selecting speed
	Gpio_Led.GPIO_PinConfig.GPIO_Pin0PType = GPIO_OP_TYPE_OD; // output type
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  //selecting pin push pull

	GPIO_PeriClockControl(GPIOA,ENABLE);  // enabling peripheral clk
	GPIO_Init(&Gpio_Led);

	// creating variable for button handling
	Gpio_Btn.pGPIOx = GPIOC ; // selecting the port of led PA5
	Gpio_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // selecting pin number
	Gpio_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;		// selecting pin mode
	Gpio_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Selecting speed
	Gpio_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  //selecting pin push pull

	GPIO_PeriClockControl(GPIOA,ENABLE);  // enabling peripheral clk
	GPIO_Init(&Gpio_Btn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
	return 0;
}
