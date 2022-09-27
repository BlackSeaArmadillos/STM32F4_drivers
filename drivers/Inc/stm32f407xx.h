/*
 * stm32f407xx.h
 *
 *  Created on: Sep 18, 2022
 *      Author: TheGreekGod
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/* ********************* START: Processor Specific Details *********************** */
/*
 *	ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0					( (volatile uint32_t *)0xE000E100 )
#define NVIC_ISER1					( (volatile uint32_t *)0xE000E104 )
#define NVIC_ISER2					( (volatile uint32_t *)0xE000E108 )
#define NVIC_ISER3					( (volatile uint32_t *)0xE000E10C )

/*
 *	ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0					( (volatile uint32_t *)0XE000E180 )
#define NVIC_ICER1					( (volatile uint32_t *)0XE000E184 )
#define NVIC_ICER2					( (volatile uint32_t *)0XE000E188 )
#define NVIC_ICER3					( (volatile uint32_t *)0XE000E18C )

/*
 * ARM Cortex Mx Processor NVIC IPRx register Addresses
 */
#define NVIC_PR_BASE_ADDR			( (volatile uint32_t *)0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4


/* ********************* START: MCU Specific Details *********************** */
/*
 *	ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASE_ADDR				0x08000000UL
#define SRAM1_BASE_ADDR				0x20000000UL
#define SRAM2_BASE_ADDR				( SRAM1_BASE_ADDR + 1C000UL )
#define ROM							0x1FFF0000UL
#define SRAM						SRAM1_BASE_ADDR


#define PERIPH_BASE_ADDR			0x40000000UL
#define APB1_PERIPH_BASE_ADDR		PERIPH_BASE_ADDR
#define APB2_PERIPH_BASE_ADDR		0x40010000UL
#define AHB1_PERIPH_BASE_ADDR		0x40020000UL
#define AHB2_PERIPH_BASE_ADDR		0x50000000UL


/*
 * Base addresses for AHB1 peripherals
 */
#define GPIOA_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x0000 )
#define GPIOB_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x0400 )
#define GPIOC_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x0800 )
#define GPIOD_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x0C00 )
#define GPIOE_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x1000 )
#define GPIOF_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x1400 )
#define GPIOG_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x1800 )
#define GPIOH_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x1C00 )
#define GPIOI_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x2000 )

#define RCC_BASE_ADDR				( AHB1_PERIPH_BASE_ADDR + 0x3800 )

/*
 * Base addresses for APB1 peripherals
 */
#define SPI2_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x3800 )
#define SPI3_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x3C00 )
#define USART2_BASE_ADDR			( APB1_PERIPH_BASE_ADDR + 0x4400 )
#define USART3_BASE_ADDR			( APB1_PERIPH_BASE_ADDR + 0x4800 )
#define UART4_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x4C00 )
#define UART5_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x5000 )
#define I2C1_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x5400 )
#define I2C2_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x5800 )
#define I2C3_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x5C00 )
#define CAN1_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x6400 )
#define CAN2_BASE_ADDR				( APB1_PERIPH_BASE_ADDR + 0x6800 )


/*
 * Base addresses for APB2 peripherals
 */
#define USART1_BASE_ADDR			( APB2_PERIPH_BASE_ADDR + 0x1000 )
#define USART6_BASE_ADDR			( APB2_PERIPH_BASE_ADDR + 0x1400 )
#define SPI1_BASE_ADDR				( APB2_PERIPH_BASE_ADDR + 0x3000 )
#define SYSCFG_BASE_ADDR			( APB2_PERIPH_BASE_ADDR + 0x3800 )
#define EXTI_BASE_ADDR				( APB2_PERIPH_BASE_ADDR + 0x3C00 )


/*
 * Peripheral register definitions structures for GPIO
 */
typedef struct {
	volatile uint32_t MODER;						// GPIO port mode register
	volatile uint32_t OTYPER;						// GPIO port output type register
	volatile uint32_t OSPEEDR;						// GPIO port output speed register
	volatile uint32_t PUPDR;						// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;							// GPIO port input data register
	volatile uint32_t ODR;							// GPIO port output data register
	volatile uint32_t BSRR;							// GPIO port bit set/reset register
	volatile uint32_t LCKR;							// GPIO port configuration lock register
	volatile uint32_t AFR[2];						// AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register
} GPIO_RegDef_t;


/*
 * Peripheral register definitions structures for RCC
 */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;


/*
 * Peripheral register definitions structures EXTI
 */
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegDef_t;


/*
 * Peripheral register definitions structures SYSCFG
 */
typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;



/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 */
#define GPIOA						( (GPIO_RegDef_t *) GPIOA_BASE_ADDR )
#define GPIOB						( (GPIO_RegDef_t *) GPIOB_BASE_ADDR )
#define GPIOC						( (GPIO_RegDef_t *) GPIOC_BASE_ADDR )
#define GPIOD						( (GPIO_RegDef_t *) GPIOD_BASE_ADDR )
#define GPIOE						( (GPIO_RegDef_t *) GPIOE_BASE_ADDR )
#define GPIOF						( (GPIO_RegDef_t *) GPIOF_BASE_ADDR )
#define GPIOG						( (GPIO_RegDef_t *) GPIOG_BASE_ADDR )
#define GPIOH						( (GPIO_RegDef_t *) GPIOH_BASE_ADDR )
#define GPIOI						( (GPIO_RegDef_t *) GPIOI_BASE_ADDR )

#define RCC							( (RCC_RegDef_t *) RCC_BASE_ADDR )
#define EXTI						( (EXTI_RegDef_t *) EXTI_BASE_ADDR )
#define SYSCFG						( (SYSCFG_RegDef_t *) SYSCFG_BASE_ADDR )


/*
 * Clock Enable macros for GPIOx peripherals
 */
#define GPIOA_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 0) )
#define GPIOB_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 1) )
#define GPIOC_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 2) )
#define GPIOD_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 3) )
#define GPIOE_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 4) )
#define GPIOF_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 5) )
#define GPIOG_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 6) )
#define GPIOH_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 7) )
#define GPIOI_PCLCK_EN()			( RCC-> AHB1ENR |= (1 << 8) )


/*
 * Clock Enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()				( RCC-> APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()				( RCC-> APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()				( RCC-> APB1ENR |= (1 << 23) )


/*
 * Clock Enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()				( RCC-> APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()				( RCC-> APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()				( RCC-> APB1ENR |= (1 << 15) )


/*
 * Clock Enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()			( RCC->APB2ENR |= (1 << 4 ) )
#define USART2_PCLK_EN()			( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()			( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()			( RCC->APB2ENR |= (1 << 5 ) )
#define UART4_PCLK_EN()				( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()				( RCC->APB1ENR |= (1 << 20) )


/*
 * Clock Enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()			( RCC->APB2ENR |= (1 << 14) )



/*
 * Clock Disable macros for GPIOx peripherals
 */
#define GPIOA_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLCK_DI()			( RCC->AHB1ENR &= ~( 1 << 8 ) )


/*
 * Clock Disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 23) )


/*
 * Clock Disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()				( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 15) )


/*
 * Clock Disable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 4 ) )
#define USART2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 5 ) )
#define UART4_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()				( RCC->APB1ENR &= ~(1 << 20) )


/*
 * Clock Disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 14) )


/*
 * Reset GPIO peripherals
 */
#define GPIOA_RESET()				do {( RCC->AHB1RSTR |= (1 << 0) );	( RCC->AHB1RSTR &= ~(1 << 0) );} while(0)
#define GPIOB_RESET()				do {( RCC->AHB1RSTR |= (1 << 1) );	( RCC->AHB1RSTR &= ~(1 << 1) );} while(0)
#define GPIOC_RESET()				do {( RCC->AHB1RSTR |= (1 << 2) );	( RCC->AHB1RSTR &= ~(1 << 2) );} while(0)
#define GPIOD_RESET()				do {( RCC->AHB1RSTR |= (1 << 3) );	( RCC->AHB1RSTR &= ~(1 << 3) );} while(0)
#define GPIOE_RESET()				do {( RCC->AHB1RSTR |= (1 << 4) );	( RCC->AHB1RSTR &= ~(1 << 4) );} while(0)
#define GPIOF_RESET()				do {( RCC->AHB1RSTR |= (1 << 5) );	( RCC->AHB1RSTR &= ~(1 << 5) );} while(0)
#define GPIOG_RESET()				do {( RCC->AHB1RSTR |= (1 << 6) );	( RCC->AHB1RSTR &= ~(1 << 6) );} while(0)
#define GPIOH_RESET()				do {( RCC->AHB1RSTR |= (1 << 7) );	( RCC->AHB1RSTR &= ~(1 << 7) );} while(0)
#define GPIOI_RESET()				do {( RCC->AHB1RSTR |= (1 << 8) );	( RCC->AHB1RSTR &= ~(1 << 8) );} while(0)


/*
 * Returns port code for given GPIO base address
 */
#define GPIO_BASE_ADDR_TO_CODE(x)		( 	(x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOF) ? 5 :\
											(x == GPIOG) ? 6 :\
											(x == GPIOH) ? 7 : 8	)


/*
 * IRQ (Interrupt Request) Numbers of STM32F407x MCU
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40


/*
 * Priority levels
 */
#define NVIC_IRQ_PRIORITY0			0
#define NVIC_IRQ_PRIORITY1			1
#define NVIC_IRQ_PRIORITY2			2
#define NVIC_IRQ_PRIORITY3			3
#define NVIC_IRQ_PRIORITY4			4
#define NVIC_IRQ_PRIORITY5			5
#define NVIC_IRQ_PRIORITY6			6
#define NVIC_IRQ_PRIORITY7			7
#define NVIC_IRQ_PRIORITY8			8
#define NVIC_IRQ_PRIORITY9			9
#define NVIC_IRQ_PRIORITY10			10
#define NVIC_IRQ_PRIORITY11			11
#define NVIC_IRQ_PRIORITY12			12
#define NVIC_IRQ_PRIORITY13			13
#define NVIC_IRQ_PRIORITY14			14
#define NVIC_IRQ_PRIORITY15			15




// Generic macros
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET


#include "stm32f407xx_gpio.h"

#endif /* INC_STM32F407XX_H_ */
