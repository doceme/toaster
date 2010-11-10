/**
 ******************************************************************************
 *
 * @file       uart.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      UART header
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


#ifndef UART_H
#define UART_H

enum uart_parity
{
	UART_PARITY_NONE,
	UART_PARITY_ODD,
	UART_PARITY_EVEN
};

enum uart_flow_control
{
	UART_FLOW_NONE,
	UART_FLOW_RTS,
	UART_FLOW_CTS,
	UART_FLOW_BOTH
};

struct uart_device
{
	uint8_t port;
	uint8_t gpio_port;
	uint32_t pin_rx;
	uint32_t pin_tx;
	uint32_t speed; /* in Hz */
	uint8_t data_bits;
	enum uart_parity parity;
	uint16_t stop_bits;
	enum uart_flow_control flow_control;
};

struct uart_xfer
{
	uint8_t* buf;
	uint32_t len;
	void (*complete)(uint32_t len);
};

/**
 * @brief  Initializes the SPI device
 * @param  device The SPI device to initialize
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int uart_init(struct uart_device *device);

/**
 * @brief  Request access to the SPI bus
 * @param  device The requested SPI device
 * @retval On success, 0 is returned
 *         On failure, a negative number is returned
 */
int uart_request(struct uart_device *device);

/**
 * @brief  Read character over UART
 * @param  data The read character
 * @retval On success, 0 is returned
 *         On failure, a negative number is returned
 * @note   This function is synchronous
 */
int uart_read_char(uint8_t *data);

/**
 * @brief  Write character over UART
 * @param  data The character to write
 * @retval On success, 0 is returned
 *         On failure, a negative number is returned
 * @note   This function is synchronous
 */
int uart_write_char(uint8_t data);

/**
 * @brief  Read data over UART
 * @param  data The buffer to store the read data
 * @param  len The number of bytes to read
 * @retval On success, 0 is returned
 *         On failure, a negative number is returned
 * @note   This function is synchronous
 */
int uart_read(uint8_t *data, uint32_t len);

/**
 * @brief  Write data over UART
 * @param  data The buffer of data to write
 * @param  len The number of bytes to write
 * @retval On success, 0 is returned
 *         On failure, a negative number is returned
 * @note   This function is synchronous
 */
int uart_write(uint8_t *data, uint32_t len);

/**
 * @brief  Transfer UART data
 * @param  request UART read request data
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned
 * @note   This function is asynchronous
 */
int uart_read_async(struct uart_xfer *request);

/**
 * @brief  Transfer UART data
 * @param  request UART write request data
 * @retval On success, 0 is returned
 *         On failure, a negative number is returned
 * @note   This function is asynchronous
 */
int uart_write_async(struct uart_xfer *request);

#endif /* UART_H */

