/**
 ******************************************************************************
 *
 * @file       stm32_common.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      STM32 common header
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


#ifndef STM32_COMMON_H
#define STM32_COMMON_H

#define GPIO_NUM_PORTS	7

struct stm32_apb_periph
{
	void (*clock)(uint32_t RCC_APB1Periph, FunctionalState NewState);
	uint32_t periph;
};

extern GPIO_TypeDef* gpio_port[GPIO_NUM_PORTS];
extern struct stm32_apb_periph gpio_apb[GPIO_NUM_PORTS];

#endif /* STM32_COMMON_H */

