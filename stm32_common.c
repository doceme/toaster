/**
 ******************************************************************************
 * @file       stm32_common.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @author     Michael Spradling Copyright (C) 2010
 * @brief      STM32 common implementation
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Includes */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>
#include "stm32_common.h"

#define DELAY_TIMER		TIM7
#define DELAY_PRESCALER_MS	((SystemCoreClock / 1000) - 1)
#define DELAY_PRESCALER_US	((SystemCoreClock / 1000000) - 1)

GPIO_TypeDef* gpio_port[GPIO_NUM_PORTS] =
{
	GPIOA,
	GPIOB,
	GPIOC,
	GPIOD,
	GPIOE,
	GPIOF,
	GPIOG
};

struct stm32_apb_periph gpio_apb[GPIO_NUM_PORTS] =
{
	{ /* GPIOA */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOA
	},
	{ /* GPIOB */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOB
	},
	{ /* GPIOC */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOC
	},
	{ /* GPIOD */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOD
	},
	{ /* GPIOE */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOE
	},
	{ /* GPIOF */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOF
	},
	{ /* GPIOG */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_GPIOG
	}
};

int common_init()
{
	TIM_TimeBaseInitTypeDef timer_init;
	TIM_TimeBaseStructInit(&timer_init);

	/* Configure timer */
	TIM_TimeBaseInit(DELAY_TIMER, &timer_init);

	return 0;
}

static inline void delay(uint16_t count, uint16_t prescaler)
{
	/* Disable timer counter */
	TIM_Cmd(DELAY_TIMER, DISABLE);

	/* Load prescaler for appropriate tick */
	TIM_SetAutoreload(DELAY_TIMER, count);
	TIM_PrescalerConfig(DELAY_TIMER, prescaler, TIM_PSCReloadMode_Immediate);

	/* Load timer count */
	//TIM_SetCounter(DELAY_TIMER, count);

	/* Clear status */
	TIM_ClearITPendingBit(DELAY_TIMER, TIM_IT_Update);

	/* Enable timer counter */
	TIM_Cmd(DELAY_TIMER, ENABLE);

	/* Wait for timer to expire */
	while(TIM_GetITStatus(DELAY_TIMER, TIM_IT_Update));

	/* Disable timer counter */
	TIM_Cmd(DELAY_TIMER, DISABLE);
}

void delay_us(uint16_t count)
{
	delay(count, DELAY_PRESCALER_US);
}
