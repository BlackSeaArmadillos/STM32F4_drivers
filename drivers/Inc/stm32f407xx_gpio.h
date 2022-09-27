/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Sep 20, 2022
 *      Author: TheGreekGod
 */

#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

#include <stdio.h>
#include "stm32f407xx.h"


/*
 * Handle structure for GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;				// values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;				// values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				// values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;		// values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;				// values from @GPIO_PIN_OTYPES
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;


typedef struct {
	GPIO_RegDef_t *pGPIOx;				// pointer that holds the address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	// holds GPIO pin config settings
} GPIO_Handle_t;



/********************************
 *	@GPIO_PIN_NUMBERS
 *	GPIO pin numbers
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15


/********************************
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_AF			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6


/********************************
 * @GPIO_PIN_OTYPES
 * GPIO pin output types
 */
#define GPIO_OTYPE_PP			0
#define GPIO_OTYPE_OD			1


/********************************
 * @GPIO_PIN_SPEED
 * GPIO pin speed
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3


/********************************
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up / pull-down
 */
#define GPIO_PUPD_NO		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2



/*
 * 		APIs supported by this driver
 *==========================================
 */


// Peripheral clock setup
void GPIO_PeriphClkCtrl (GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

// Init and DeInit
void GPIO_Init (GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx);

// Data read and write
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_Toggle (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ config and ISR handling
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandler (uint8_t PinNumber);


#endif /* STM32F407XX_GPIO_H_ */
