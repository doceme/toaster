/**
 ******************************************************************************
 * @file       lcd.c
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
/* #include "lcd.h" */

#define LCD_PIN_RESET	GPIO_Pin_0
#define LCD_PIN_RS	GPIO_Pin_1

#define LCD_PIXEL_WIDTH		320
#define LCD_PIXEL_HEIGHT	240

/* Color definitions */
#define	BLACK			0x0000
#define	BLUE			0x001F
#define	RED			0xF800
#define	GREEN			0x07E0
#define CYAN			0x07FF
#define MAGENTA			0xF81F
#define YELLOW			0xFFE0
#define WHITE			0xFFFF

static struct spi_device lcd =
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

static void lcd_write_command(uint8_t data)
{
	GPIO_ResetBits(gpio_port[lcd.gpio_port], LCD_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
}

static void lcd_write_data(uint16_t data)
{
	GPIO_SetBits(gpio_port[lcd.gpio_port], LCD_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data >> 8);
	spi_cs(SPI_CS_DEACTIVATE);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
}

static inline void lcd_write_reg(uint8_t address, uint16_t data)
{
	lcd_write_command(address);
	lcd_write_data(data);
}

static void lcd_home()
{
	lcd_write_reg(0x4E, 0x0000); /* RAM address set */
	lcd_write_reg(0x4F, 0x0000); /* RAM address set */
	lcd_write_reg(0x44, 0xEF00); /* Vertical RAM address position */
	lcd_write_reg(0x45, 0x0000); /* Horizontal RAM address position */
	lcd_write_reg(0x46, 0x013F); /* Horizontal RAM address position */

	lcd_write_command(0x22); /* RAM data write/read */
}

void lcd_test(void)
{
	uint16_t x;
	uint16_t y;

	lcd_home();

	for (x = 0; x < LCD_PIXEL_WIDTH; x++)
		for (y = 0; y < LCD_PIXEL_HEIGHT; y++)
			lcd_write_data(BLACK);

	lcd_home();

	for (x = 0; x < LCD_PIXEL_WIDTH; x++)
	{
		for (y = 0; y < LCD_PIXEL_HEIGHT; y++)
		{
			if (x > 279)
				lcd_write_data(BLACK);
			else if (x > 239)
				lcd_write_data(BLUE);
			else if (x > 199)
				lcd_write_data(GREEN);
			else if (x > 159)
				lcd_write_data(CYAN);
			else if (x > 119)
				lcd_write_data(RED);
			else if (x > 79)
				lcd_write_data(MAGENTA);
			else if (x > 39)
				lcd_write_data(YELLOW);
			else
				lcd_write_data(WHITE);
		}
	}
}

int display_init(void)
{
	int result = -1;

	GPIO_InitTypeDef gpio_init;

	if (gpio_apb[lcd.gpio_port].clock)
	{
		gpio_apb[lcd.gpio_port].clock(gpio_apb[lcd.gpio_port].periph, ENABLE);
	}

	GPIO_SetBits(gpio_port[lcd.gpio_port], LCD_PIN_RS);
	GPIO_SetBits(gpio_port[lcd.gpio_port], LCD_PIN_RESET);

	/* Configure LCD reset and register select pins */
	gpio_init.GPIO_Pin = LCD_PIN_RESET | LCD_PIN_RS;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(gpio_port[lcd.gpio_port], &gpio_init);

	spi_init(&lcd);

	GPIO_ResetBits(gpio_port[lcd.gpio_port], LCD_PIN_RESET);
	delay_ms(200);
	GPIO_SetBits(gpio_port[lcd.gpio_port], LCD_PIN_RESET);
	delay_ms(500);

	/* Initialize registers */
	lcd_write_reg(0x28, 0x0006);    // VCOM OTP
	lcd_write_reg(0x00, 0x0001);    // start Oscillator

	lcd_write_reg(0x10, 0x0000);    // Exit sleep mode
	delay_ms(30);

	lcd_write_reg(0x01, 0x32EF);    // Driver Output Control
	lcd_write_reg(0x02, 0x0600);    // LCD Driving Waveform Control
	lcd_write_reg(0x03, 0x6A38);    // Power Control 1
	lcd_write_reg(0x11, 0x6870);    // Entry Mode
	lcd_write_reg(0x0F, 0x0000);    // Gate Scan Position
	lcd_write_reg(0x0B, 0x5308);    // Frame Cycle Control
	lcd_write_reg(0x0C, 0x0003);    // Power Control 2
	lcd_write_reg(0x0D, 0x000A);    // Power Control 3
	lcd_write_reg(0x0E, 0x2E00);    // Power Control 4
	lcd_write_reg(0x1E, 0x00BE);    // Power Control 5
	lcd_write_reg(0x25, 0x8000);    // Frame Frequency Control
	lcd_write_reg(0x26, 0x7800);    // Analog setting
	lcd_write_reg(0x4E, 0x0000);    // Ram Address Set
	lcd_write_reg(0x4F, 0x0000);    // Ram Address Set
	lcd_write_reg(0x12, 0x08D9);    // Sleep mode

	/* Gamma control */
	lcd_write_reg(0x30, 0x0000);
	lcd_write_reg(0x31, 0x0104);
	lcd_write_reg(0x32, 0x0100);
	lcd_write_reg(0x33, 0x0305);
	lcd_write_reg(0x34, 0x0505);
	lcd_write_reg(0x35, 0x0305);
	lcd_write_reg(0x36, 0x0707);
	lcd_write_reg(0x37, 0x0300);
	lcd_write_reg(0x3A, 0x1200);
	lcd_write_reg(0x3B, 0x0800);
	lcd_write_reg(0x07, 0x0033);    // Display Control 

	delay_ms(150);

	lcd_write_command(0x22);    // RAM data write/read

	lcd_test();

	result = 0;

	return result;
}
