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
#define SPI1_PCLK_EN()					(RCC->APB2ENR |=(1<<12)
#define SPI2_PCLK_EN()					(RCC->APB1ENR |=(1<<14)
#define SPI3_PCLK_EN()					(RCC->APB1ENR |=(1<<15)
#define SPI4_PCLK_EN()					(RCC->APB2ENR |=(1<<13)

/*
 * Clock enable macros for UARTx peripherals

 */
#define USART2_PCLK_EN()					(RCC->APB1ENR |=(1<<17)
#define USART3_PCLK_EN()					(RCC->APB1ENR |=(1<<18)
#define UART4_PCLK_EN()						(RCC->APB1ENR |=(1<<19)
#define UART5_PCLK_EN()						(RCC->APB1ENR |=(1<<20)


/*
 * Clock enable macros for SYSCFG peripherals

 */
#define SYSCFG_PCLK_EN()					(RCC->APBENR |=(1<<14)




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
#define SPI1_PCLK_DN()					(RCC->APB2ENR &= ~(1<<12)
#define SPI2_PCLK_DN()					(RCC->APB1ENR &= ~(1<<14)
#define SPI3_PCLK_DN()					(RCC->APB1ENR &= ~(1<<15)
#define SPI4_PCLK_DN()					(RCC->APB2ENR &= ~(1<<13)

/*
 * Clock disable macros for UARTx peripherals

 */
#define USART2_PCLK_DN()					(RCC->APB1ENR &= ~(1<<17)
#define USART3_PCLK_DN()					(RCC->APB1ENR &= ~(1<<18)
#define UART4_PCLK_DN()						(RCC->APB1ENR &= ~(1<<19)
#define UART5_PCLK_DN()						(RCC->APB1ENR &= ~(1<<20)


/*
 * Clock disable macros for SYSCFG peripherals

 */
#define SYSCFG_PCLK_DN()					(RCC->APBENR &= ~(1<<14)




//some generic macros

#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

#endif /* INC_STM32F446XX_H_ */

