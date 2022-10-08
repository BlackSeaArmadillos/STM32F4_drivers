/*
 * 006_SPI_Test.c
 *
 *  Created on: Oct 3, 2022
 *      Author: TheGreekGod
 */

/*
 * SPI2_MOSI --> PB15
 * SPI2_MISO --> PB14
 * SPI2_SCK  --> PB13
 * SPI2_NSS  --> PB12
 * AF: 5
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"

void SPI2_GPIOInit(void);
void SPI2_Init(void);

int main (void) {

	char txData[] = "Hello World";

	// 1. Initialize GPIO for SPI2
	SPI2_GPIOInit();

	SPI2_Init();

	SPI_SSIConfig(SPI2, ENABLE);		// pull-up to VCC the NSS signal and avoid MODF error

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t *)txData, strlen(txData));

	SPI_PeripheralControl(SPI2, DISABLE);

	while (1);
}


void SPI2_GPIOInit(void) {

	GPIO_Handle_t SPI_Pins;

	SPI_Pins.pGPIOx = GPIOB;

	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;		// Config NSS
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;		// Config SCK
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;		// Config MISO
	GPIO_Init(&SPI_Pins);

	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;		// Config MOSI
	GPIO_Init(&SPI_Pins);
}


void SPI2_Init(void) {

	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;

	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;			// 8 MHz
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2_Handle);
}





