/*
 * 005_Button_interrupt.c
 *
 *  Created on: Sep 27, 2022
 *      Author: TheGreekGod
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define HIGH			1
#define BTN_PRESSED		HIGH

void delay(uint32_t value);

int main(void) {
	GPIO_Handle_t gpio_led, gpio_btn;

	memset(&gpio_led, 0, sizeof(gpio_led));
	memset(&gpio_btn, 0, sizeof(gpio_btn));

	// Configure LED
	gpio_led.pGPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType =  GPIO_OTYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_PeriphClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	// Configure Button with interrupt
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeriphClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&gpio_btn);

	// Configure IRQ
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIORITY15);



	while (1);
}

void delay(uint32_t value) {

	for (uint32_t i = 0; i < value; i++);
}


void EXTI0_IRQHandler(void) {
    // Handle the interrupt
	delay(250000);
    GPIO_IRQHandler(GPIO_PIN_0);
    GPIO_Toggle(GPIOD, GPIO_PIN_12);
}


