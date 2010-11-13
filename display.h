/**
 ******************************************************************************
 *
 * @file       display.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      Display header
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef DISPLAY_H
#define DISPLAY_H

#include "ssd2119.h"

/* Color definitions */
#define	BLACK			0x0000
#define	BLUE			0x001F
#define	RED			0xF800
#define	GREEN			0x07E0
#define CYAN			0x07FF
#define MAGENTA			0xF81F
#define YELLOW			0xFFE0
#define WHITE			0xFFFF

struct display_rect
{
	uint16_t x1;
	uint16_t y1;
	uint16_t x2;
	uint16_t y2;
};

/**
 * @brief  Initializes the display
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int display_init(void);

/**
 * @brief  Set a pixel to a given color
 * @param  x The X coordinate
 * @param  y The Y coordinate
 * @param  color The color to draw
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int display_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief  Fill the display with a specified color
 * @param  color The color to fill
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int display_fill(uint16_t color);

/**
 * @brief  Fill a rectangle with a specified color
 * @param  rect The rectangle to fill
 * @param  color The color to fill
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int display_fill_rect(const struct display_rect *rect, uint16_t color);

enum display_op_type
{
	DISPLAY_OP_DONE,	/* Indicates no more operations */
	DISPLAY_OP_DELAY,	/* Delay (ms) */
	DISPLAY_OP_CMD,		/* Write command */
	DISPLAY_OP_DATA,	/* Write data */
	DISPLAY_OP_REG		/* Write register */
};

struct display_op
{
	enum display_op_type	type;
	uint16_t		cmd;
	uint16_t		data;
};

struct display_panel {
	const struct display_op *init_ops;
	const char *name;

	uint32_t vres;
	uint32_t hres;
	uint32_t stride;
	uint32_t flags;

	uint16_t id;
	uint16_t vpos_reg;
	uint16_t hpos_reg;
	uint16_t vwnd_reg;
	uint16_t hwnd1_reg;
	uint16_t hwnd2_reg;
	uint16_t gram_reg;

	int (*ctl_init)(void);
	int (*write_data)(uint16_t data);
	int (*write_cmd)(uint16_t cmd);
	int (*write_reg)(uint16_t addr, uint16_t data);
};

#ifdef CONFIG_DISPLAY_CFAF320240F
#define DISPLAY_CTL_SSD2119
#define DISPLAY_PANEL_NAME	"CFAF320240F"
#define DISPLAY_HRES		320
#define DISPLAY_VRES		240
#define DISPLAY_STRIDE		320

#define DISPLAY_HMASK		0x1FF
#define DISPLAY_VMASK		0xFF

#ifdef CONFIG_DISPLAY_LANDSCAPE
#define DISPLAY_PIXEL_WIDTH	DISPLAY_HRES
#define DISPLAY_PIXEL_HEIGHT	DISPLAY_VRES
#define DISPLAY_MASK_WIDTH	DISPLAY_HMASK
#define DISPLAY_MASK_HEIGHT	DISPLAY_VMASK
#define CFAF32240F_OUT_CTL	0x72EF
#define CFAF32240F_ENTRY_MODE	0x6870
#else
#define DISPLAY_PIXEL_WIDTH	DISPLAY_VRES
#define DISPLAY_PIXEL_HEIGHT	DISPLAY_HRES
#define DISPLAY_MASK_WIDTH	DISPLAY_VMASK
#define DISPLAY_MASK_HEIGHT	DISPLAY_HMASK
#define CFAF32240F_OUT_CTL	0x70EF
#define CFAF32240F_ENTRY_MODE	0x6878
#endif

static const struct display_op ssd2119_ops[] =
{
	/* Basic init */
	{DISPLAY_OP_REG, 0x0028, 0x0006}, /* VCOM OTP */
	{DISPLAY_OP_REG, 0x0000, 0x0001}, /* Start oscillator */
	{DISPLAY_OP_REG, 0x0010, 0x0000}, /* Exit sleep mode */
	{DISPLAY_OP_REG, 0x0001, CFAF32240F_OUT_CTL}, /* Driver output control */
	{DISPLAY_OP_REG, 0x0002, 0x0600}, /* LCD driving waveform control */
	{DISPLAY_OP_REG, 0x0003, 0x6a38}, /* Power control 1 */
	{DISPLAY_OP_REG, 0x0011, CFAF32240F_ENTRY_MODE}, /* Entry mode */
	{DISPLAY_OP_REG, 0X000f, 0x0000}, /* Gate scan position */
	{DISPLAY_OP_REG, 0X000b, 0x5308}, /* Frame cycle control */
	{DISPLAY_OP_REG, 0x000c, 0x0003}, /* Power control 2 */
	{DISPLAY_OP_REG, 0x000d, 0x000a}, /* Power control 3 */
	{DISPLAY_OP_REG, 0x000e, 0x2e00}, /* Power control 4 */
	{DISPLAY_OP_REG, 0x001e, 0x00be}, /* Power control 5 */
	{DISPLAY_OP_REG, 0x0025, 0x8000}, /* Frame frequency control */
	{DISPLAY_OP_REG, 0x0026, 0x7800}, /* Analog setting */
	{DISPLAY_OP_REG, 0x004e, 0x0000}, /* Ram address set */
	{DISPLAY_OP_REG, 0x004f, 0x0000}, /* Ram address set */
	{DISPLAY_OP_REG, 0x0012, 0x08D9}, /* Sleep mode */

	/* Gamma control */
	{DISPLAY_OP_REG, 0x0030, 0x0000},
	{DISPLAY_OP_REG, 0x0031, 0x0104},
	{DISPLAY_OP_REG, 0x0032, 0x0100},
	{DISPLAY_OP_REG, 0x0033, 0x0305},
	{DISPLAY_OP_REG, 0x0034, 0x0505},
	{DISPLAY_OP_REG, 0x0035, 0x0305},
	{DISPLAY_OP_REG, 0x0036, 0x0707},
	{DISPLAY_OP_REG, 0x0037, 0x0300},
	{DISPLAY_OP_REG, 0x003a, 0x1200},
	{DISPLAY_OP_REG, 0x003b, 0x0800},

	/* Turn on display */
	{DISPLAY_OP_REG, 0x0007, 0x0033},
	{DISPLAY_OP_DELAY, 0, 150},
	{DISPLAY_OP_DONE, 0, 0}
};
#endif

#ifdef DISPLAY_CTL_SSD2119
#define DISPLAY_VPOS_REG	0x4e
#define DISPLAY_HPOS_REG	0x4f
#define DISPLAY_VWND_REG	0x44
#define DISPLAY_HWND1_REG	0x45
#define DISPLAY_HWND2_REG	0x46
#define DISPLAY_GRAM_REG	0x22
#define DISPLAY_ID		0x9919
#define DISPLAY_INIT_OPS	ssd2119_ops
#define DISPLAY_CTL_INIT	ssd2119_init
#define DISPLAY_WRITE_DATA	ssd2119_write_data
#define DISPLAY_WRITE_CMD	ssd2119_write_cmd
#define DISPLAY_WRITE_REG	ssd2119_write_reg
#endif

#endif /* DISPLAY_H */

