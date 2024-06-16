/*
 * 004spi_tx_testing.c
 *
 *  Created on: Jun 16, 2024
 *      Author: Sachin K S
 */

#include "stm32f446xx.h"
#include <string.h>
//PB12 - NSS
//PB13 - SCLK
//PB14 - MISO
//PB15 - MOSI
// Alternate fun mode- 5


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}
void SPI2_GPIOInits(void)
{




	GPIO_Handle_t SPIPins;  //init base addr


	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_Pin0PType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}
void GPIO_ButtonInit(void)
{

		GPIO_Handle_t Gpio_Btn;

	// creating variable for button handling
		Gpio_Btn.pGPIOx = GPIOC ; // selecting the port of led PA5
		Gpio_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // selecting pin number
		Gpio_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;		// selecting pin mode
		Gpio_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Selecting speed
		Gpio_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  //selecting pin push pull


		GPIO_Init(&Gpio_Btn);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 2MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}
int main(void)
{

	char user_data[] = "Hello world";

	SPI2_GPIOInits();

	// this makes NSS signal internally high and avoids MODF error

	SPI2_Inits();

	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
				//wait until button is pressed
				while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

				delay();

				//enable the SPI2 peripheral
				SPI_PeripheralControl(SPI2,ENABLE);

				//first send length information
				uint8_t dataLen = strlen(user_data);
				SPI_SendData(SPI2,&dataLen,1);

				SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

				//lets confirm SPI is not busy
				while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

				//disable SPI2 Peripheral
				SPI_PeriClockControl(SPI2, DISABLE);




	}
	return 0;
}
