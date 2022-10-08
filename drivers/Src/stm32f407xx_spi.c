/*
 * stm32f407xx_spi.c
 *
 *  Created on: Oct 2, 2022
 *      Author: TheGreekGod
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"


static uint8_t SPI_GetFlagStatus (SPI_RegDef_t *pSPIx, uint32_t Flag);

/********************************************************************************
 * @fn				- SPI_PeriphClkCtrl
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
void SPI_PeriphClkCtrl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	}
	else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}


/********************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_Init (SPI_Handle_t *pSPIHandle) {

	uint32_t tempreg = 0;

	// Enable clock
	SPI_PeriphClkCtrl(pSPIHandle->pSPIx, ENABLE);

	// 1. Configure Device Mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2. Configure Bus Config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX) {
		tempreg &= ~(1 << 15); 						// Clear BIDIMODE
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX) {
		tempreg |= (1 << 15);						// Set BIDIMODE
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		tempreg &= ~(1 << 15);						// Clear BIDIMODE
		tempreg |= (1 << 10);						// Set RXONLY bit
	}

	// 3. Configure Serial Clock Speed (BaudRate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// 8. Configure CR1 register
	pSPIHandle->pSPIx->CR1 = tempreg;
}


/********************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_DeInit (SPI_RegDef_t *pSPIx) {

}


/********************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- Blocking mode API (polling API)
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while (Len > 0) {

		while (!SPI_GetFlagStatus(pSPIx, SPI_SR_TXE));			// Whait until TXE bit is set

		if ((pSPIx->CR1 >> SPI_CR1_DFF) & 0x01) {
			// 16bit DFF
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else {
			// 8bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


/********************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

	while (Len > 0) {

		while (!SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE));			// Whait until RXNE bit is set

		if ((pSPIx->CR1 >> SPI_CR1_DFF) & 0x01) {
			// 16bit DFF
			*(uint16_t *)pRxBuffer = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else {
			// 8bit DFF
			*pTxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/********************************************************************************
 * @fn				- SPI_SendDataIT
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
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t state = pSPI_Handle->TxState;

	if (state != SPI_TX_BUSY) {

		pSPI_Handle->pTxBuffer = pTxBuffer;
		pSPI_Handle->TxLen = Len;

		pSPI_Handle->TxState = SPI_TX_BUSY;						// State is busy in Tx mode. No other code can take over same SPI periph until transmission is over
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);		// Enable TXEIE bit to get interrupt whenever TXE flag is set in SR
	}

	return state;
}


/********************************************************************************
 * @fn				- SPI_ReceiveDataIT
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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPI_Handle->TxState;

	if (state != SPI_RX_BUSY) {

		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = Len;

		pSPI_Handle->RxState = SPI_RX_BUSY;						// State is busy in Tx mode. No other code can take over same SPI periph until transmission is over
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);		// Enable TXEIE bit to get interrupt whenever TXE flag is set in SR
	}

	return state;
}


/********************************************************************************
 * @fn				- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnOrDi) {

	if (EnOrDi ==ENABLE) {
		if (IRQNumber < 32) {
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 64) {
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else {
		if (IRQNumber < 32) {
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber < 64) {
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/********************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	// 1. Find IPR register
	uint8_t iprx, iprx_section, shift_amount;

	iprx = IRQNumber / 4;
	iprx_section = IRQNumber % 4;
	shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}


/********************************************************************************
 * @fn				- SPI_IRQHandler
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_IRQHandler (SPI_Handle_t *pHandle) {

}


/********************************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
static uint8_t SPI_GetFlagStatus (SPI_RegDef_t *pSPIx, uint32_t Flag) {

	if ((pSPIx->SR >> Flag) & 0x01)
		return FLAG_SET;

	return FLAG_RESET;
}


/********************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/********************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- The function enables or disables the peripheral clock for the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
*/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


