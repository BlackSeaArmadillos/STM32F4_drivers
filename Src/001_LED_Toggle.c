/*
 * 001_LED_Toggle.c
 *
 *  Created on: Sep 24, 2022
 *      Author: TheGreekGod
 */

#include <stdio.h>
#include "stm32f407xx.h"

void delay(uint32_t value);

int main(void) {

	GPIO_Handle_t gpio_led;

	// Select GPIO port
	gpio_led.pGPIOx = GPIOD;

	// Pin configuration
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType =  GPIO_OTYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	// Enable peripheral clock
	GPIO_PeriphClkCtrl(GPIOD, ENABLE);

	// Initialize peripheral
	GPIO_Init(&gpio_led);

	// Infinite loop
	while(1) {
		GPIO_Toggle(GPIOD, GPIO_PIN_12);
		delay(250000);
	}
}



void delay(uint32_t value) {

	for (uint32_t i = 0; i < value; i++);
}
