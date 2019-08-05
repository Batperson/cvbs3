/*
 * hardware.c
 *
 *  Created on: 6/07/2019
 */

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "macros.h"
#include "bitband.h"
#include "hardware.h"


void initLeds()
{
	/* Clock to GPIOF */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Initialize PF9, PF10 for LED blinking */
	GPIO_InitTypeDef		gpio;

	gpio.GPIO_Pin 	= GPIO_Pin_0 | GPIO_Pin_7 | GPIO_Pin_14;
	gpio.GPIO_Mode 	= GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	setLed1(OFF);
	setLed2(OFF);
	setLed3(OFF);
}
