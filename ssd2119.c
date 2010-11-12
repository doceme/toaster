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

#define LCD_PIN_RESET	GPIO_Pin_0
#define LCD_PIN_RS	GPIO_Pin_1

//#define LCD_ORIENTATION_PORTRAIT

#ifdef LCD_ORIENTATION_PORTRAIT
#define LCD_MASK_WIDTH		0x1FF
#define LCD_MASK_HEIGHT		0xFF
#define LCD_PIXEL_WIDTH		320
#define LCD_PIXEL_HEIGHT	240
#else
#define LCD_MASK_WIDTH		0xFF
#define LCD_MASK_HEIGHT		0x1FF
#define LCD_PIXEL_WIDTH		240
#define LCD_PIXEL_HEIGHT	320
#endif

#define LCD_NUM_TEST_BARS	6

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

static uint16_t ssd2119_test_bars[LCD_NUM_TEST_BARS] =
{
	WHITE,
	YELLOW,
	MAGENTA,
	RED,
	CYAN,
	GREEN
};

static void ssd2119_write_command(uint8_t data)
{
	GPIO_ResetBits(gpio_port[ssd2119.gpio_port], LCD_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
}

static void ssd2119_write_data(uint16_t data)
{
	GPIO_SetBits(gpio_port[ssd2119.gpio_port], LCD_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data >> 8);
	spi_cs(SPI_CS_DEACTIVATE);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
}

static inline void ssd2119_write_reg(uint8_t address, uint16_t data)
{
	ssd2119_write_command(address);
	ssd2119_write_data(data);
}

static void ssd2119_home()
{
	ssd2119_write_reg(0x44, 0xEF00); /* Vertical RAM address position */
	ssd2119_write_reg(0x45, 0x0000); /* Horizontal RAM address position */
	ssd2119_write_reg(0x46, 0x013F); /* Horizontal RAM address position */

	ssd2119_write_reg(0x4E, 0x0000); /* RAM address set */
	ssd2119_write_reg(0x4F, 0x0000); /* RAM address set */

	ssd2119_write_command(0x22); /* RAM data write/read */
}

static void ssd2119_window(struct display_rect *rect)
{
	if (!rect)
		return;

#ifdef LCD_ORIENTATION_PORTRAIT
	ssd2119_write_reg(0x44, (rect->y2 << 8) | (rect->y1 & LCD_MASK_HEIGHT)); /* Vertical RAM address position */
	ssd2119_write_reg(0x45, rect->x1); /* Horizontal RAM address position */
	ssd2119_write_reg(0x46, rect->x2); /* Horizontal RAM address position */

	ssd2119_write_reg(0x4E, rect->x1); /* RAM address set */
	ssd2119_write_reg(0x4F, rect->y1); /* RAM address set */
#else
	ssd2119_write_reg(0x44, (rect->x2 << 8) | (rect->x1 & LCD_MASK_HEIGHT)); /* Vertical RAM address position */
	ssd2119_write_reg(0x45, rect->y1); /* Horizontal RAM address position */
	ssd2119_write_reg(0x46, rect->y2); /* Horizontal RAM address position */

	ssd2119_write_reg(0x4E, rect->y1); /* RAM address set */
	ssd2119_write_reg(0x4F, rect->x1); /* RAM address set */
#endif

	ssd2119_write_command(0x22); /* RAM data write/read */
}

void ssd2119_test(void)
{
	uint16_t x;
	uint16_t y;
	uint16_t i = 0;

	struct display_rect rect =
	{
		.x1 = 40,
		.y1 = 40,
		.x2 = 79,
		.y2 = 119
	};

	ssd2119_home();

	for (y = 0; y < LCD_PIXEL_HEIGHT; y++)
	{
		for (x = 0; x < LCD_PIXEL_WIDTH; x++)
			ssd2119_write_data(ssd2119_test_bars[i]);

		if ((y + 1) % (LCD_PIXEL_HEIGHT / LCD_NUM_TEST_BARS) == 0)
			i++;
	}

	display_fill_rect(&rect, BLUE);
}

int display_init(void)
{
	GPIO_InitTypeDef gpio_init;

	if (gpio_apb[ssd2119.gpio_port].clock)
	{
		gpio_apb[ssd2119.gpio_port].clock(gpio_apb[ssd2119.gpio_port].periph, ENABLE);
	}

	GPIO_SetBits(gpio_port[ssd2119.gpio_port], LCD_PIN_RS);
	GPIO_SetBits(gpio_port[ssd2119.gpio_port], LCD_PIN_RESET);

	/* Configure LCD reset and register select pins */
	gpio_init.GPIO_Pin = LCD_PIN_RESET | LCD_PIN_RS;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(gpio_port[ssd2119.gpio_port], &gpio_init);

	spi_init(&ssd2119);

	GPIO_ResetBits(gpio_port[ssd2119.gpio_port], LCD_PIN_RESET);
	delay_ms(200);
	GPIO_SetBits(gpio_port[ssd2119.gpio_port], LCD_PIN_RESET);
	delay_ms(500);

	/* Initialize registers */
	ssd2119_write_reg(0x28, 0x0006);    // VCOM OTP
	ssd2119_write_reg(0x00, 0x0001);    // start Oscillator

	ssd2119_write_reg(0x10, 0x0000);    // Exit sleep mode
	delay_ms(30);

#ifdef LCD_ORIENTATION_PORTRAIT
	ssd2119_write_reg(0x01, 0x72EF);    // Driver Output Control
#else
	ssd2119_write_reg(0x01, 0x70EF);    // Driver Output Control
#endif
	ssd2119_write_reg(0x02, 0x0600);    // LCD Driving Waveform Control
	ssd2119_write_reg(0x03, 0x6A38);    // Power Control 1
#ifdef LCD_ORIENTATION_PORTRAIT
	ssd2119_write_reg(0x11, 0x6870);    // Entry Mode
#else
	ssd2119_write_reg(0x11, 0x6878);    // Entry Mode
#endif
	ssd2119_write_reg(0x0F, 0x0000);    // Gate Scan Position
	ssd2119_write_reg(0x0B, 0x5308);    // Frame Cycle Control
	ssd2119_write_reg(0x0C, 0x0003);    // Power Control 2
	ssd2119_write_reg(0x0D, 0x000A);    // Power Control 3
	ssd2119_write_reg(0x0E, 0x2E00);    // Power Control 4
	ssd2119_write_reg(0x1E, 0x00BE);    // Power Control 5
	ssd2119_write_reg(0x25, 0x8000);    // Frame Frequency Control
	ssd2119_write_reg(0x26, 0x7800);    // Analog setting
	ssd2119_write_reg(0x4E, 0x0000);    // Ram Address Set
	ssd2119_write_reg(0x4F, 0x0000);    // Ram Address Set
	ssd2119_write_reg(0x12, 0x08D9);    // Sleep mode

	/* Gamma control */
	ssd2119_write_reg(0x30, 0x0000);
	ssd2119_write_reg(0x31, 0x0104);
	ssd2119_write_reg(0x32, 0x0100);
	ssd2119_write_reg(0x33, 0x0305);
	ssd2119_write_reg(0x34, 0x0505);
	ssd2119_write_reg(0x35, 0x0305);
	ssd2119_write_reg(0x36, 0x0707);
	ssd2119_write_reg(0x37, 0x0300);
	ssd2119_write_reg(0x3A, 0x1200);
	ssd2119_write_reg(0x3B, 0x0800);
	ssd2119_write_reg(0x07, 0x0033);    // Display Control

	delay_ms(150);

	display_fill(BLACK);

	ssd2119_test();

	return 0;
}

int display_pixel(uint16_t x, uint16_t y, uint16_t color)
{
	if (x >= LCD_PIXEL_WIDTH)
		x = LCD_PIXEL_WIDTH - 1;
	if (y >= LCD_PIXEL_HEIGHT)
		y = LCD_PIXEL_HEIGHT - 1;

#ifdef LCD_ORIENTATION_PORTRAIT
	ssd2119_write_reg(0x4E, x);
	ssd2119_write_reg(0x4F, y);
#else
	ssd2119_write_reg(0x4E, y);
	ssd2119_write_reg(0x4F, x);
#endif
	ssd2119_write_reg(0x22, color);

	return 0;
}

int display_fill(uint16_t color)
{
	static const struct display_rect rect =
	{
		.x1 = 0,
		.y1 = 0,
		.x2 = LCD_PIXEL_WIDTH - 1,
		.y2 = LCD_PIXEL_HEIGHT - 1
	};

	display_fill_rect((struct display_rect *)&rect, color);

	return 0;
}

int display_fill_rect(struct display_rect *rect, uint16_t color)
{
	uint16_t x;
	uint16_t y;

	ssd2119_window(rect);

	for (y = 0; y < (rect->y2 - rect->y1) + 1; y++)
		for (x = 0; x < (rect->x2 - rect->x1) + 1; x++)
			ssd2119_write_data(color);

	return 0;
}
