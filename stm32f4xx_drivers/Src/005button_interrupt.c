/*
 * 005button_interrupt.c
 *
 *  Created on: 20-Sep-2023
 *      Author: amanupadhyay
 */
#include "stm32f407xx.h"

#define HIGH			ENABLE
#define LOW				0
#define BTN_PRESSED 	LOW

void delay()
{
	for(uint32_t i = 0; i < 500000/2;i++);
}

int main()
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode	  = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

//this is an button gpio configuration
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode	  = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOBtn);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}

