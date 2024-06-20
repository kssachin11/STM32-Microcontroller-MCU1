/*
 * stm32f446xx.h
 *
 *  Created on: Jun 2, 2024
 *      Author: Sachin K S
 */
// Contains MCU specific data

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>   // for defining uint32.....

#define __vo volatile



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  	4


// DEFINING BASE ADDRESS OF VARIOUS MEMORIES - FLASH AND SRAM

#define FLASH_BASEADDR			0x08000000U  // MAIN MEMORY
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM_BASEADDR			0x1FFF0000U  //SYSTEM MEMORY
#define SRAM 					SRAM1_BASEADDR

//DEFINING BASE ADDRESS OF VARIOUS BUS DOMAIN

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR // SAME AS OF PERIPHERAL BASE ADDRESS
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x40023C00U

// DEFINE BASE ADDRESS PF PERIPHERAL HANGING ON AHB1 BUS

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0000) // PEIHERAL BASE ADDR OF AHB1 BUS + OFFSET OF PERIPHERAL
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0800)
#define	GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1C00)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)
// DEFINE BASE ADDRESS PF PERIPHERAL HANGING ON APB1 BUS

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)


// DEFINE BASE ADDRESS OF PERIPHERAL HANGING ON APB2 BUS
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)


// CREATING PERIPHERAL STRUCTURE FOR GPIO

typedef struct{

	__vo uint32_t MODER;	//GPIO port mode register
	__vo uint32_t OTYPER;	//GPIO port output type register
	__vo uint32_t OSPEEDER;	//GPIO port output speed register
	__vo uint32_t PUPDR;	//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;		//GPIO port input data register
	__vo uint32_t ODR;		//GPIO port output data register
	__vo uint32_t BSRR;		//GPIO port bit set/reset register
	__vo uint32_t LCKR;		//GPIO port configuration lock register
	__vo uint32_t AFR[2];	/* created an array for alternate function register instead of writing the 2 registers
							AF[1];GPIO ALTERNATE FUNCTION HIGH REGISTER , AF[0]:GPIO ALTERNATE FUNCTION LOW REGISTER*/

}GPIO_RegDef_t;// gpio register definition structure


// creating register structure for RCC
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t Reserved1;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t Reserved2[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t Reserved3;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t Reserved4[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */


typedef struct
{
		__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
		__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
		__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
		__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
		__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
		__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
		__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
		__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
		__vo uint32_t I2SPR;      /*!< TODO,											Address offset: 0*/

}SPI_RegDef_t;


/*
 * peripheral register definition structure for USART
 */

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
		
			
}USART_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */
#define GPIOA 				 ((GPIO_RegDef_t*)GPIOA_BASEADDR) // to define macro, TYPE CASTING GPIOA TO GPIO
#define GPIOB 				 ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 				 ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 				 ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 				 ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				 ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 			   	 ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 				 ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				 	 ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				 ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASEADDR)


#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6				((USART_RegDef_t*)USART6_BASEADDR)
/*
 * Clock enable macros for GPIOx peripherals

 */
#define GPIOA_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PERI_CLOCK_ENABLE()		(RCC->AHB1ENR |=(1<<7))

/*
 * Clock enable macros for I2Cx peripherals

 */
#define I2C1_PCLK_EN()					(RCC->APB1ENR |=(1<<21)
#define I2C2_PCLK_EN()					(RCC->APB1ENR |=(1<<22)
#define I2C3_PCLK_EN()					(RCC->APB1ENR |=(1<<23)

/*
 * Clock enable macros for SPIx peripherals

 */
#define SPI1_PCLK_EN()					(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN()					(RCC->APB2ENR |=(1<<13))

/*
 * Clock enable macros for UARTx peripherals

 */
#define USART1_PCLK_EN()					(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |=(1<<17)
#define USART3_PCLK_EN()					(RCC->APB1ENR |=(1<<18)
#define UART4_PCLK_EN()						(RCC->APB1ENR |=(1<<19)
#define UART5_PCLK_EN()						(RCC->APB1ENR |=(1<<20)
#define USART6_PCLK_EN()					(RCC->APB2ENR |=(1<<5))

/*
 * Clock enable macros for SYSCFG peripherals

 */
#define SYSCFG_PCLK_EN()					(RCC->APB1ENR |=(1<<14))




/*
 * Clock disable macros for GPIOx peripherals

 */
#define GPIOA_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PERI_CLOCK_DISABLE()		(RCC->AHB1ENR &= ~(1<<7))



/*
 * Clock disable macros for I2Cx peripherals

 */
#define I2C1_PCLK_DN()					(RCC->APB1ENR &= ~(1<<21)
#define I2C2_PCLK_DN()					(RCC->APB1ENR &= ~(1<<22)
#define I2C3_PCLK_DN()					(RCC->APB1ENR &= ~(1<<23)


/*
 * Clock disable macros for SPIx peripherals

 */
#define SPI1_PCLK_DN()					(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DN()					(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DN()					(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DN()					(RCC->APB2ENR &= ~(1<<13))

/*
 * Clock disable macros for UARTx peripherals

 */
#define USART1_PCLK_DN()					(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DN()					(RCC->APB1ENR &= ~(1<<17)
#define USART3_PCLK_DN()					(RCC->APB1ENR &= ~(1<<18)
#define UART4_PCLK_DN()						(RCC->APB1ENR &= ~(1<<19)
#define UART5_PCLK_DN()						(RCC->APB1ENR &= ~(1<<20)
#define USART6_PCLK_DN()					(RCC->APB2ENR &= ~(1<<5))

/*
 * Clock disable macros for SYSCFG peripherals

 */
#define SYSCFG_PCLK_DN()					(RCC->APBENR &= ~(1<<14)



/*
 * MACROS TO RESET GPIOx PERIPHERALS
 *
 */
#define GPIOA_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<0));		(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<1));		(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<2));		(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<3));		(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<4));		(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<5));		(RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<6));		(RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()					do{(RCC->AHB1RSTR|=(1<<7));		(RCC->AHB1RSTR &= ~(1<<7));}while(0)


/*
 * MACROS TO RESET SPIx PERIPHERALS
 *
 */
#define SPI1_REG_RESET()					do{(RCC->APB2RSTR|=(1<<12));		(RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()					do{(RCC->APB1RSTR|=(1<<14));		(RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI4_REG_RESET()					do{(RCC->APB2RSTR|=(1<<13));		(RCC->APB2RSTR &= ~(1<<13));}while(0)
#define SPI3_REG_RESET()					do{(RCC->APB1RSTR|=(1<<15));		(RCC->APB1RSTR &= ~(1<<15));}while(0)
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)				((x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
										    (x == GPIOD)?3:\
										    (x == GPIOE)?4:\
										    (x == GPIOF)?5:\
										    (x == GPIOG)?6:\
										    (x == GPIOH)?7:0		)		// if x== GPIOA ,return 0,else check for
																						// C-conditional operators




/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15



//some generic macros

#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1 - values depends bit position on register CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15


/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

#endif /* INC_STM32F446XX_H_ */

