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
#include "stm32_common.h"
#include "spi.h"
#include "display.h"
#include "tprintf.h"

#define SSD2119_PIN_RESET	GPIO_Pin_0
#define SSD2119_PIN_RS		GPIO_Pin_1

static struct spi_device ssd2119 =
{
	.data_bits = 8,
	.port = 1,
	.gpio_port = 1,
	.pin_clk = GPIO_Pin_13,
	.pin_mosi = GPIO_Pin_15,
	.pin_cs = GPIO_Pin_2,
	.mode = SPI_MODE_3,
	.speed = 4000000
};

int ssd2119_write_cmd(uint16_t cmd)
{
	GPIO_ResetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(cmd & 0xff);
	spi_cs(SPI_CS_DEACTIVATE);
	return 0;
}

int ssd2119_write_data(uint16_t data)
{
	GPIO_SetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data >> 8);
	spi_cs(SPI_CS_DEACTIVATE);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
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

	if (gpio_apb[ssd2119.gpio_port].clock)
	{
		gpio_apb[ssd2119.gpio_port].clock(gpio_apb[ssd2119.gpio_port].periph, ENABLE);
	}

	GPIO_SetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RS);
	GPIO_SetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RESET);

	/* Configure LCD reset and register select pins */
	gpio_init.GPIO_Pin = SSD2119_PIN_RESET | SSD2119_PIN_RS;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(gpio_port[ssd2119.gpio_port], &gpio_init);

	spi_init(&ssd2119);

	GPIO_ResetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RESET);
	delay_ms(200);
	GPIO_SetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RESET);
	delay_ms(500);

	return 0;
}
