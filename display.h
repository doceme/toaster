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
int display_fill_rect(struct display_rect *rect, uint16_t color);

#endif /* DISPLAY_H */

