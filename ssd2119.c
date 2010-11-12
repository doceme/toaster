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
#define SSD2119LCD_PIN_RS	GPIO_Pin_1

#define DISPLAY_NUM_TEST_BARS	6

/* Color definitions */
#define	BLACK			0x0000
#define	BLUE			0x001F
#define	RED			0xF800
#define	GREEN			0x07E0
#define CYAN			0x07FF
#define MAGENTA			0xF81F
#define YELLOW			0xFFE0
#define WHITE			0xFFFF

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

static uint16_t display_test_bars[DISPLAY_NUM_TEST_BARS] =
{
	WHITE,
	YELLOW,
	MAGENTA,
	RED,
	CYAN,
	GREEN
};

static int ssd2119_write_cmd(uint16_t cmd)
{
	GPIO_ResetBits(gpio_port[ssd2119.gpio_port], SSD2119_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(cmd & 0xff);
	spi_cs(SPI_CS_DEACTIVATE);
	return 0;
}

static int ssd2119_write_data(uint16_t data)
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

static int display_write_reg(uint16_t addr, uint16_t data)
{
	display.write_cmd(addr);
	display.write_data(data);
	return 0;
}

static int display_goto(uint16_t x, uint16_t y)
{
	display.write_reg(display->horz_reg, x & DISPLAY_MASK_WIDTH);
	display.write_reg(display->vert_reg, y & DISPLAY_MASK_HEIGHT);
	display.write_cmd(display->gram_reg);
	return 0;
}

static int display_window(const struct display_rect *rect)
{
	if (!rect)
	{
		tprintf("Invalid rect parameter!\r\n");
		return -EPARAM;
	}
	else
	{
		uint16_t vpos;
		uint16_t hpos;
		uint16_t vwnd;
		uint16_t hwnd1;
		uint16_t hwnd2;

#ifdef CONFIG_DISPLAY_PORTRAIT
		vpos = rect->x1;
		hpos = rect->y1;
		vwnd = (rect->y2 << 8) | rect->y1;
		hwnd1 = rect->x1;
		hwnd2 = rect->x2;
#else
		vpos = rect->y1;
		hpos = rect->x1;
		vwnd = (rect->x2 << 8) | rect->x1;
		hwnd1 = rect->y1;
		hwnd2 = rect->y2;
#endif

		display.write_reg(display.vpos_reg, vpos);
		display.write_reg(display.hpos_reg, hpos);
		display.write_reg(display.vwnd_reg, vwnd);
		display.write_reg(display.hwnd1_reg, hwnd1);
		display.write_reg(display.hwnd2_reg, hwnd2);
		display.write_cmd(display.gram_reg);
	}

	return 0;
}

static void display_test(void)
{
	uint16_t x;
	uint16_t y;
	uint16_t i = 0;

	const struct display_rect rect =
	{
		.x1 = 40,
		.y1 = 40,
		.x2 = 79,
		.y2 = 119
	};

	display_goto(0, 0);
	display_window(&screen_rect);

	for (y = 0; y < DISPLAY_PIXEL_HEIGHT; y++)
	{
		for (x = 0; x < DISPLAY_PIXEL_WIDTH; x++)
			display.write_data(ssd2119_test_bars[i]);

		if ((y + 1) % (DISPLAY_PIXEL_HEIGHT / DISPLAY_NUM_TEST_BARS) == 0)
			i++;
	}

	display_fill_rect(&rect, BLUE);
}

static int display_exec_op(struct display_op *op)
{
	if (!op)
	{
		tprintf("Invalid op parameter!\r\n");
		return -EPARAM;
	}

	switch (op->type)
	{
		case DISPLAY_OP_DELAY:
			delay_ms(op->data);
			break;
		case DISPLAY_OP_CMD:
			display.write_cmd(op->cmd);
			break;
		case DISPLAY_OP_DATA:
			display.write_data(op->cmd);
			break;
		case DISPLAY_OP_DATA:
			display.write_reg(op->cmd, op->data);
			break;
		default:
			tprintf("Invalid op type!\r\n");
			return -EPARAM;
	}

	return 0;
}

static int ssd2119_init(void)
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

int display_init(void)
{
	int i = 0;

	if (display.ctl_init)
		display_ctl_init();

	if (display.init_ops)
		while (display.init_ops[i] != DISPLAY_OP_DONE)
			display_exec_op(&display.init_ops[i++]);

	display_fill(BLACK);

	display_test();

	return 0;
}

int display_pixel(uint16_t x, uint16_t y, uint16_t color)
{
	uint16_t vpos;
	uint16_t hpos;

	if (x >= DISPLAY_PIXEL_WIDTH)
		x = DISPLAY_PIXEL_WIDTH - 1;
	if (y >= DISPLAY_PIXEL_HEIGHT)
		y = DISPLAY_PIXEL_HEIGHT - 1;

#ifdef CONFIG_DISPLAY_PORTRAIT
	vpos = x;
	hpos = y;
#else
	vpos = y;
	hpos = x;
#endif

	display.write_reg(display.vpos_reg, vpos);
	display.write_reg(display.hpos_reg, hpos);
	display.write_reg(display.gram_reg, color);

	return 0;
}

int display_fill(uint16_t color)
{
	display_fill_rect(&screen_rect, color);
	return 0;
}

int display_fill_rect(struct display_rect *rect, uint16_t color)
{
	uint16_t x;
	uint16_t y;

	display_window(rect);

	for (y = 0; y < (rect->y2 - rect->y1) + 1; y++)
		for (x = 0; x < (rect->x2 - rect->x1) + 1; x++)
			display.write_data(color);

	return 0;
}
