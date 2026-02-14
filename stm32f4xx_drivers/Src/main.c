/*
 * main.c
 *
 *  Created on: 31-Aug-2023
 *      Author: amanupadhyay
 */
#include "stm32f407xx.h"


int main()
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	GPIO_IRQHandling(0);

}
