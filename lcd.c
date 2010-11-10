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
#include "spi.h"
#include "lcd.h"

#define LCD_PIN_RESET	GPIO_Pin_0
#define LCD_PIN_RS	GPIO_Pin_1

static struct spi_device lcd =
{
	.data_bits = 8,
	.port = 1,
	.gpio_port = 1,
	.pin_clk = GPIO_Pin_13,
	.pin_mosi = GPIO_Pin_14,
	.pin_cs = GPIO_Pin_2,
	.mode = SPI_MODE_3,
	.speed = 4000000
};

static void lcd_write_command(uint8_t data)
{
	GPIO_ResetBits(gpio_port[lcd->gpio_port], LCD_PIN_RS);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
	GPIO_SetBits(gpio_port[lcd->gpio_port], LCD_PIN_RS);
}

static void lcd_write_data(uint16_t data)
{
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data >> 8);
	spi_cs(SPI_CS_DEACTIVATE);
	spi_cs(SPI_CS_ACTIVATE);
	spi_w8(data);
	spi_cs(SPI_CS_DEACTIVATE);
}

int lcd_init()
{
	int result = -1;

	GPIO_InitTypeDef gpio_init;

	if (gpio_apb[lcd->gpio_port].clock)
	{
		gpio_apb[lcd->gpio_port].clock(gpio_apb[uart->gpio_port].periph, ENABLE);
	}

	/* Configure LCD reset and register select pins */
	gpio_init.GPIO_Pin = LCD_PIN_RESET | LCD_PIN_RS;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(gpio_port[lcd->gpio_port], &gpio_init);

	GPIO_ResetBits(gpio_port[lcd->gpio_port], LCD_PIN_RESET);
	delay_ms(200);
	GPIO_SetBits(gpio_port[lcd->gpio_port], LCD_PIN_RESET);
	delay_ms(500);

	result = 0;

	return result;
}

int uart_request(struct uart_device *device)
{
	int result = -1;

	if (device)
	{
		uart = device;
		result = 0;
	}

	return result;
}

void uart_handler(void)
{
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	uart_handler();
}

/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	uart_handler();
}
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	uart_handler();
}
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)

/**
  * @brief  This function handles UART4 interrupt request.
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
	uart_handler();
}

/**
  * @brief  This function handles UART5 interrupt request.
  * @param  None
  * @retval None
  */
void UART5_IRQHandler(void)
{
	uart_handler();
}
#endif

int uart_read_char(uint8_t *data)
{
	return -ERR_NOIMP;
}

int uart_write_char(uint8_t data)
{
	while (USART_GetFlagStatus(uart_port[uart->port], USART_FLAG_TXE) == RESET);
	USART_SendData(uart_port[uart->port], data);
	//while (USART_GetFlagStatus(uart_port[uart->port], USART_FLAG_TC) == RESET);
}

int uart_read(uint8_t *data, uint32_t len)
{
	return -ERR_NOIMP;
}

int uart_write(uint8_t *data, uint32_t len)
{
	while (len-- > 0)
	{
		while (USART_GetFlagStatus(uart_port[uart->port], USART_FLAG_TXE) == RESET);
		USART_SendData(uart_port[uart->port], *data++);
	}
}

int uart_read_async(struct uart_xfer *request)
{
	return -ERR_NOIMP;
}

int uart_write_async(struct uart_xfer *request)
{
	return -ERR_NOIMP;
}
