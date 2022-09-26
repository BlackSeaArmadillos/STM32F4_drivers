/*
 * 002_LED_Button.c
 *
 *  Created on: Sep 25, 2022
 *      Author: TheGreekGod
 */


#include <stdio.h>
#include "stm32f407xx.h"

#define HIGH			1
#define BTN_PRESSED		HIGH

void delay(uint32_t value);

int main(void) {

	GPIO_Handle_t gpio_led, gpio_btn;

	// Configure LED
	gpio_led.pGPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType =  GPIO_OTYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_PeriphClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	// Configure btn
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_PeriphClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&gpio_btn);

	// Infinite loop
	while(1) {
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0 == BTN_PRESSED)) {
			delay(250000);
			GPIO_Toggle(GPIOD, GPIO_PIN_12);
		}
	}
}



void delay(uint32_t value) {

	for (uint32_t i = 0; i < value; i++);
}
