/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Sep 20, 2022
 *      Author: TheGreekGod
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"




/********************************************************************************
 * @fn				- GPIO_PeriphClkCtrl
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_PeriphClkCtrl (GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLCK_EN();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLCK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLCK_EN();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLCK_EN();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLCK_EN();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLCK_EN();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLCK_EN();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLCK_EN();
		}
		else if (pGPIOx == GPIOI) {
			GPIOI_PCLCK_EN();
		}
	}
	else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLCK_DI();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLCK_DI();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLCK_DI();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLCK_DI();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLCK_DI();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLCK_DI();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLCK_DI();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLCK_DI();
		}
		else if (pGPIOx == GPIOI) {
			GPIOI_PCLCK_DI();
		}
	}
}


/********************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Initialize GPIO peripheral
 *
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	// 1. Configure GPIO pin mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Non-interupt mode

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));		// Get the mode and the pin number
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );					// Clear bits
		pGPIOHandle->pGPIOx->MODER |= temp;																			// Store the value of temp in the mode register
		temp = 0;
	}
	else {
		// Interupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Clear corresponding RTSR bit
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1. Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// Clear correspongind FTSR bit
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t exti_temp1, exti_temp2;
		uint8_t portcode;

		exti_temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		exti_temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[exti_temp1] |= portcode << (4*exti_temp2);

		// 3. Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. Configure GPIO speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));		// Get the speed and pin number
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );						// Clear bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;																		// Store the value of temp in the speed register
	temp = 0;

	// 3. Configure GPIO pull-up/pull-down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));		// Get PuPd and pin number
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );							// Clear bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;																					// Store the value of temp in pull-up/pull-down register
	temp = 0;

	// 4. Configure output type register
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Get output type and pin number
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );					// Clear bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;																	// Store the values of temp in output type register
	temp = 0;

	// 5. Configure alternate function register
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF) {
		uint8_t temp1, temp2 = 0;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;												// Get the register 0 (AFL) or 1 (AFH)
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;												// Get the pin number
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4*temp2) );													// Clear bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));	// Store the values in temp2 in register[temp1]
	}
	else {
		// nothing to be done
	}
}

/********************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			-
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_RESET();
	}
	else if (pGPIOx == GPIOB) {
		GPIOB_RESET();
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_RESET();
	}
	else if (pGPIOx == GPIOD) {
		GPIOD_RESET();
	}
	else if (pGPIOx == GPIOE) {
		GPIOE_RESET();
	}
	else if (pGPIOx == GPIOF) {
		GPIOF_RESET();
	}
	else if (pGPIOx == GPIOG) {
		GPIOG_RESET();
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_RESET();
	}
	else if (pGPIOx == GPIOI) {
		GPIOI_RESET();
	}
}



/********************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			-
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @Note			- none
*/
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	uint8_t value;
	value = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/********************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- 16 bit value
 *
 * @Note			- none
*/
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx) {

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/********************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			-
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number to write to:		0, 1, 2, ... 15
 * @param[in]		- Value to write to the pin: 	0 or 1
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {

//	pGPIOx->ODR |= (Value << PinNumber);

	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/********************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- The function writes to a GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Value to write to output register
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value) {

	pGPIOx->ODR = Value;
}

/********************************************************************************
 * @fn				- GPIO_Toggle
 *
 * @brief			- The function toggles the pin
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin number
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_Toggle (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber);
}



/********************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
*/
void GPIO_IRQConfig (uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi) {

}

/********************************************************************************
 * @fn				- GPIO_IRQHandler
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void GPIO_IRQHandler (uint8_t PinNumber) {

}



