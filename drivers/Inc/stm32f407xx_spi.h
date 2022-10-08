/*
 * stm32f407xx_spi.h
 *
 *  Created on: Oct 2, 2022
 *      Author: TheGreekGod
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_


/*
 * Configuration structure for GPIO pin
 */
typedef struct {
	uint8_t SPI_DeviceMode;					// Values @SPI_DEVICE_MODES
	uint8_t SPI_BusConfig;					// Values @SPI_BUS_CONFIG
	uint8_t SPI_SclkSpeed;					// Values @SPI_SCLK_SPEED
	uint8_t SPI_DFF;						// Values @SPI_DFF
	uint8_t SPI_CPOL;						// Values @SPI_CPOL
	uint8_t SPI_CPHA;						// Values @SPI_CPHA
	uint8_t SPI_SSM;						// Values @SPI_SSM
} SPI_Config_t;


/*
 * Handle structure for GPIO pin
 */
typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;



/********************************
 *	@SPI_DEVICE_MODES
 *	SPI Device Modes (Master or Slave)
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/********************************
 *	@SPI_BUS_CONFIG
 *	SPI Bus Configurations (Full-Duplex, Half-Duplex, Simplex)
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX			1
#define SPI_BUS_CONFIG_HALF_DUPLEX			2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3


/********************************
 *	@SPI_SCLK_SPEED
 *	SPI Serial Clock Speed (fPCLK/2)
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


/********************************
 *	@SPI_DFF
 *	SPI Data Frame Format
 */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1


/********************************
 *	@SPI_CPOL
 *	SPI Clock Polarity
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0


/********************************
 *	@SPI_CPHA
 *	SPI Clock Phase
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0


/********************************
 *	@SPI_SSM
 *	SPI Software Slave Management
 */
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0


/*
 * SPI application states
 */
#define SPI_READY			0
#define SPI_RX_BUSY			1
#define SPI_TX_BUSY			2


/*******************************************************************************************************************************************
 * 		APIs supported by this driver
 *==========================================
 */

// Peripheral clock setup
void SPI_PeriphClkCtrl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// Init and DeInit
void SPI_Init (SPI_Handle_t *pSPIHandle);
void SPI_DeInit (SPI_RegDef_t *pSPIx);

// Data Send and Receive functions
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len);

// IRQ config and ISR handling
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler (SPI_Handle_t *pHandle);

// Other APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F407XX_SPI_H_ */
