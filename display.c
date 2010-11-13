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

const struct display_panel display =
{
	.init_ops	= DISPLAY_INIT_OPS,
	.name		= DISPLAY_PANEL_NAME,
	.vres		= DISPLAY_VRES,
	.hres		= DISPLAY_HRES,
	.stride		= DISPLAY_STRIDE,
	.id		= DISPLAY_ID,
	.vpos_reg	= DISPLAY_VPOS_REG,
	.hpos_reg	= DISPLAY_HPOS_REG,
	.vwnd_reg	= DISPLAY_VWND_REG,
	.hwnd1_reg	= DISPLAY_HWND1_REG,
	.hwnd2_reg	= DISPLAY_HWND2_REG,
	.gram_reg	= DISPLAY_GRAM_REG,
	.ctl_init	= DISPLAY_CTL_INIT,
	.write_data	= DISPLAY_WRITE_DATA,
	.write_cmd	= DISPLAY_WRITE_CMD,
	.write_reg	= DISPLAY_WRITE_REG
};

static const struct display_rect screen_rect =
{
	.x1 = 0,
	.y1 = 0,
	.x2 = DISPLAY_PIXEL_WIDTH - 1,
	.y2 = DISPLAY_PIXEL_HEIGHT - 1
};

static int display_goto(uint16_t x, uint16_t y)
{
	display.write_reg(display.hpos_reg, x & DISPLAY_MASK_WIDTH);
	display.write_reg(display.vpos_reg, y & DISPLAY_MASK_HEIGHT);
	display.write_cmd(display.gram_reg);
	return 0;
}

static int display_window(const struct display_rect *rect)
{
	if (!rect)
	{
		tprintf("Invalid rect parameter!\r\n");
		return -ERR_PARAM;
	}
	else
	{
		uint16_t vpos;
		uint16_t hpos;
		uint16_t vwnd;
		uint16_t hwnd1;
		uint16_t hwnd2;

#ifdef CONFIG_DISPLAY_LANDSCAPE
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

static int display_exec_op(const struct display_op *op)
{
	if (!op)
	{
		tprintf("Invalid op parameter!\r\n");
		return -ERR_PARAM;
	}

	for (;;)
	{
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
			case DISPLAY_OP_REG:
				display.write_reg(op->cmd, op->data);
				break;
			case DISPLAY_OP_DONE:
				return 0;
			default:
				tprintf("Invalid op type!\r\n");
				return -ERR_PARAM;
		}

		op++;
	}

	return 0;
}

int display_init(void)
{
	if (display.ctl_init)
		display.ctl_init();

	if (display.init_ops)
		display_exec_op(display.init_ops);

	display_window(&screen_rect);
	display_goto(0, 0);

	display_fill(BLACK);

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

#ifdef CONFIG_DISPLAY_LANDSCAPE
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

int display_fill_rect(const struct display_rect *rect, uint16_t color)
{
	uint16_t x;
	uint16_t y;

	display_window(rect);

	for (y = 0; y < (rect->y2 - rect->y1) + 1; y++)
		for (x = 0; x < (rect->x2 - rect->x1) + 1; x++)
			display.write_data(color);

	return 0;
}
