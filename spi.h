/**
 ******************************************************************************
 *
 * @file       spi.h
 * @author     Stephen Caudle Copyright (C) 2010.
 * @brief      SPI header
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


#ifndef SPI_H
#define SPI_H

enum spi_cs_activation
{
	SPI_CS_ACTIVATE,
	SPI_CS_DEACTIVATE
};

enum spi_mode
{
	SPI_MODE_0,
	SPI_MODE_1,
	SPI_MODE_2,
	SPI_MODE_3
};

struct spi_device
{
	uint8_t slave; /* 0 = master, 1 = slave */
	uint8_t data_bits; /* in bits */
	uint8_t lsb; /* 0 = msb first, 1 = lsb first */
	uint8_t port;
	uint8_t gpio_port;
	uint32_t pin_clk;
	uint32_t pin_mosi;
	uint32_t pin_miso;
	uint32_t pin_cs;
	enum spi_mode mode;
	uint32_t speed; /* in Hz */
};

struct spi_xfer_8
{
	uint8_t* rx_buf;
	uint8_t* tx_buf;
	uint32_t rx_len;
	uint32_t tx_len;
	void (*complete)(uint32_t tx_len, uint32_t rx_len);
};

struct spi_xfer_16
{
	uint16_t* rx_buf;
	uint16_t* tx_buf;
	uint32_t rx_len;
	uint32_t tx_len;
	void (*complete)(uint32_t tx_len, uint32_t rx_len);
};

struct spi_xfer_32
{
	uint32_t* rx_buf;
	uint32_t* tx_buf;
	uint32_t rx_len;
	uint32_t tx_len;
	void (*complete)(uint32_t tx_len, uint32_t rx_len);
};

/**
 * @brief  Initializes the SPI device
 * @param  device The SPI device to initialize
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int spi_init(struct spi_device *device);

/**
 * @brief  Request access to the SPI bus
 * @param  device The requested SPI device
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int spi_request(struct spi_device *device);

/**
 * @brief  Activate or deactivate the chip select pin
 * @param  spi_cs_activation SPI_CS_ACTIVATE or SPI_CS_DEACTIVATE
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 */
int spi_cs(enum spi_cs_activation);

/**
 * @brief  Read 8 bits
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint8_t spi_r8(void);

/**
 * @brief  Read 16 bits
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint16_t spi_r16(void);

/**
 * @brief  Read 32 bits
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint32_t spi_r32(void);

/**
 * @brief  Write 8 bits
 * @param  data Data to write over SPI bus
 * @note   This function is synchronous
 */
void spi_w8(uint8_t data);

/**
 * @brief  Write 16 bits
 * @param  data Data to write over SPI bus
 * @note   This function is synchronous
 */
void spi_w16(uint16_t data);

/**
 * @brief  Write 32 bits
 * @param  data Data to write over SPI bus
 * @note   This function is synchronous
 */
void spi_w32(uint32_t data);

/**
 * @brief  Write 8 bits, then read 8 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint8_t spi_w8r8(uint8_t data);

/**
 * @brief  Write 8 bits, then read 16 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint16_t spi_w8r16(uint8_t data);

/**
 * @brief  Write 8 bits, then read 32 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint32_t spi_w8r32(uint8_t data);

/**
 * @brief  Write 16 bits, then read 8 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint8_t spi_w16r8(uint16_t data);

/**
 * @brief  Write 16 bits, then read 16 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint16_t spi_w16r16(uint16_t data);

/**
 * @brief  Write 16 bits, then read 32 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint32_t spi_w16r32(uint16_t data);

/**
 * @brief  Write 32 bits, then read 8 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint8_t spi_w32r8(uint32_t data);

/**
 * @brief  Write 32 bits, then read 16 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint16_t spi_w32r16(uint32_t data);

/**
 * @brief  Write 32 bits, then read 32 bits
 * @param  data Data to write over SPI bus
 * @retval Data read from SPI bus
 * @note   This function is synchronous
 */
uint32_t spi_w32r32(uint32_t data);

/**
 * @brief  Transfer 8-bit SPI data
 * @param  request SPI transfer request data
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 * @note   This function is asynchronous
 */
int spi_xfer_8(struct spi_xfer_8 *request);

/**
 * @brief  Transfer 16-bit SPI data
 * @param  request SPI transfer request data
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 * @note   This function is asynchronous
 */
int spi_xfer_16(struct spi_xfer_16 *request);

/**
 * @brief  Transfer 32-bit SPI data
 * @param  request SPI transfer request data
 * @retval On success, 0 is returned.
 *         On failure, a negative number is returned.
 * @note   This function is asynchronous
 */
int spi_xfer_32(struct spi_xfer_32 *request);

#endif /* SPI_H */

