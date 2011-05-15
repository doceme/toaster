/**
 ******************************************************************************
 * @file       ssd2119.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @author     Michael Spradling Copyright (C) 2010
 * @brief      LCD implementation
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
#include "common.h"
#include "display.h"
#include "tprintf.h"

#ifdef FREERTOS
/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#endif

#define delay_ms vTaskDelay

#define SSD2119_GPIO_RESET	GPIOB
#define SSD2119_PIN_RESET	GPIO_Pin_9

int ssd2119_write_cmd(uint16_t cmd)
{
	LCD_REG = cmd;
	return 0;
}

int ssd2119_write_data(uint16_t data)
{
	LCD_RAM = data;
	return 0;
}

int ssd2119_write_reg(uint16_t addr, uint16_t data)
{
	ssd2119_write_cmd(addr);
	ssd2119_write_data(data);
	return 0;
}

int ssd2119_init(void)
{
	GPIO_InitTypeDef gpio_init;
	FSMC_NORSRAMInitTypeDef fsmc_init;
	FSMC_NORSRAMTimingInitTypeDef fsmc_timing_init;

	/* Enable FSMC AHB clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
				RCC_APB2Periph_GPIOD |
				RCC_APB2Periph_GPIOE |
				RCC_APB2Periph_AFIO, ENABLE);

	/* Configure LCD reset pin */
	gpio_init.GPIO_Pin = SSD2119_PIN_RESET;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SSD2119_GPIO_RESET, &gpio_init);

	/* Configure FSMC pins */
	gpio_init.GPIO_Pin = GPIO_Pin_0 |
			     GPIO_Pin_1 |
			     GPIO_Pin_4 |
			     GPIO_Pin_5 |
			     GPIO_Pin_7 |
			     GPIO_Pin_8 |
			     GPIO_Pin_9 |
			     GPIO_Pin_10 |
			     GPIO_Pin_11 |
			     GPIO_Pin_14 |
			     GPIO_Pin_15;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpio_init);

	gpio_init.GPIO_Pin = GPIO_Pin_7 |
			     GPIO_Pin_8 |
			     GPIO_Pin_9 |
			     GPIO_Pin_10 |
			     GPIO_Pin_11 |
			     GPIO_Pin_12 |
			     GPIO_Pin_13 |
			     GPIO_Pin_14 |
			     GPIO_Pin_15;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &gpio_init);

	/* FSMC_Bank1_NORSRAM1 timing configuration */
	fsmc_timing_init.FSMC_AddressSetupTime = 3;
	fsmc_timing_init.FSMC_AddressHoldTime = 0;
	fsmc_timing_init.FSMC_DataSetupTime = 3;
	fsmc_timing_init.FSMC_BusTurnAroundDuration = 0;
	fsmc_timing_init.FSMC_CLKDivision = 0;
	fsmc_timing_init.FSMC_DataLatency = 0;
	fsmc_timing_init.FSMC_AccessMode = FSMC_AccessMode_A;

	/* FSMC_Bank1_NORSRAM1 configured as follows:
	   - Data/Address MUX = Disable
	   - Memory Type = SRAM
	   - Data Width = 16bit
	   - Write Operation = Enable
	   - Extended Mode = Disable
	   - Asynchronous Wait = Disable */
	fsmc_init.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	fsmc_init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	fsmc_init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	fsmc_init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	fsmc_init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	fsmc_init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	fsmc_init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	fsmc_init.FSMC_WrapMode = FSMC_WrapMode_Disable;
	fsmc_init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	fsmc_init.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	fsmc_init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	fsmc_init.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
	fsmc_init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	fsmc_init.FSMC_ReadWriteTimingStruct = &fsmc_timing_init;
	fsmc_init.FSMC_WriteTimingStruct = &fsmc_timing_init;

	FSMC_NORSRAMInit(&fsmc_init);

	/* Enable FSMC_Bank1_NORSRAM1 */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);

	GPIO_WriteBit(SSD2119_GPIO_RESET, SSD2119_PIN_RESET, Bit_RESET);
	delay_ms(200);
	GPIO_WriteBit(SSD2119_GPIO_RESET, SSD2119_PIN_RESET, Bit_SET);
	delay_ms(500);

	return 0;
}
