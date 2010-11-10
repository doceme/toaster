/**
 ******************************************************************************
 * @file       stm32_spi.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @author     Michael Spradling Copyright (C) 2010
 * @brief      STM32 SPI Implementation
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

#if (defined(STM32F10X_LD) || defined(STM32F10X_LD_VL))
#define SPI_NUM_PORTS	1
#elif (defined(STM32F10X_MD) || defined(STM32F10X_MD_VL))
#define SPI_NUM_PORTS	2
#else
#define SPI_NUM_PORTS	3
#endif

static uint32_t spi_irq[SPI_NUM_PORTS] =
{
	SPI1_IRQn,
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	SPI2_IRQn,
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	SPI3_IRQn
#endif
};

static SPI_TypeDef* spi_port[SPI_NUM_PORTS] =
{
	SPI1,
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	SPI2,
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	SPI3
#endif
};

static struct stm32_apb_periph spi_apb[SPI_NUM_PORTS] =
{
	{ /* SPI1 */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_SPI1
	},
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	{ /* SPI2 */
		.clock = RCC_APB1PeriphClockCmd,
		.periph = RCC_APB1Periph_SPI2
	},
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	{ /* SPI3 */
		.clock = RCC_APB1PeriphClockCmd,
		.periph = RCC_APB1Periph_SPI3
	}
#endif
};

static struct spi_device *spi;

int spi_init(struct spi_device *device)
{
	int result = -1;

	if (device)
	{
		GPIO_InitTypeDef gpio_init;
		NVIC_InitTypeDef nvic_init;
		SPI_InitTypeDef spi_init;

		spi = device;

		/* Enable APB clocks */
		if (spi_apb[spi->port].clock)
		{
			spi_apb[spi->port].clock(spi_apb[spi->port].periph, ENABLE);
		}
		if (gpio_apb[spi->gpio_port].clock)
		{
			gpio_apb[spi->gpio_port].clock(gpio_apb[spi->gpio_port].periph, ENABLE);
		}

		/* Deactivate slave select */
		spi_cs(SPI_CS_DEACTIVATE);

		/* Configure SPI clock and data pins */
		gpio_init.GPIO_Pin = spi->pin_clk | spi->pin_mosi | spi->pin_miso;
		gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
		gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(gpio_port[spi->gpio_port], &gpio_init);

		/* Configure SPI chip select pins */
		gpio_init.GPIO_Pin = spi->pin_cs;
		gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(gpio_port[spi->gpio_port], &gpio_init);

		/* Enable and set SPI interrupt */
		nvic_init.NVIC_IRQChannel = spi_irq[spi->port];
		nvic_init.NVIC_IRQChannelPreemptionPriority = 0x0;
		nvic_init.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic_init);

		/* Configure SPI controller */
		spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

		if (spi->slave)
		{
			spi_init.SPI_Mode = SPI_Mode_Slave;
		}
		else
		{
			spi_init.SPI_Mode = SPI_Mode_Master;
		}

		if (spi->data_bits == 16)
		{
			spi_init.SPI_DataSize = SPI_DataSize_16b;
		}
		else
		{
			spi_init.SPI_DataSize = SPI_DataSize_8b;
		}

		switch (spi->mode)
		{
			case SPI_MODE_1:
				spi_init.SPI_CPOL = SPI_CPOL_Low;
				spi_init.SPI_CPHA = SPI_CPHA_2Edge;
				break;

			case SPI_MODE_2:
				spi_init.SPI_CPOL = SPI_CPOL_Low;
				spi_init.SPI_CPHA = SPI_CPHA_1Edge;
				break;

			case SPI_MODE_3:
				spi_init.SPI_CPOL = SPI_CPOL_High;
				spi_init.SPI_CPHA = SPI_CPHA_2Edge;
				break;

			default:
				spi_init.SPI_CPOL = SPI_CPOL_High;
				spi_init.SPI_CPHA = SPI_CPHA_1Edge;
				break;
		}

		spi_init.SPI_NSS = SPI_NSS_Soft;

		spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

		if (spi->lsb)
		{
			spi_init.SPI_FirstBit = SPI_FirstBit_LSB;
		}
		else
		{
			spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
		}

		spi_init.SPI_CRCPolynomial = 7;

		SPI_Init(spi_port[spi->port], &spi_init);

		/* Enable SPI */
		SPI_Cmd(spi_port[spi->port], ENABLE);

		result = 0;
	}

	return result;
}

int spi_request(struct spi_device *device)
{
	int result = -1;

	if (device)
	{
		spi = device;
		result = 0;
	}

	return result;
}

int spi_cs(enum spi_cs_activation deactivate)
{
	int result = -1;

	if (spi)
	{
		GPIO_WriteBit(gpio_port[spi->gpio_port], spi->pin_cs, deactivate & 0x1);
		result = 0;
	}

	return result;
}

void spi_handler(void)
{
}

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void)
{
	spi_handler();
}

#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	spi_handler();
}
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
	spi_handler();
}
#endif

uint8_t spi_r8(void)
{
	return spi_w16r16(0) & 0xff;
}

uint16_t spi_r16(void)
{
	return spi_w16r16(0);
}

uint32_t spi_r32(void)
{
	return spi_w16r16(0);
}

void spi_w8(uint8_t data)
{
	spi_w16(data);
}

void spi_w16(uint16_t data)
{
	if (spi)
	{
		/* Send data */
		SPI_I2S_SendData(spi_port[spi->port], data);

		/* Wait for data to be shifted out */
		while (SPI_I2S_GetFlagStatus(spi_port[spi->port], SPI_I2S_FLAG_TXE) == RESET);
	}
}

void spi_w32(uint32_t data)
{
	spi_w16(data);
}

uint8_t spi_w8r8(uint8_t data)
{
	return spi_w16r16(data) & 0xff;
}

uint16_t spi_w8r16(uint8_t data)
{
	return spi_w16r16(data);
}

uint32_t spi_w8r32(uint8_t data)
{
	return spi_w16r16(data);
}

uint8_t spi_w16r8(uint16_t data)
{
	return spi_w16r16(data) & 0xff;
}

uint16_t spi_w16r16(uint16_t data)
{
	uint16_t result = 0;

	if (spi)
	{
		/* Send data */
		SPI_I2S_SendData(spi_port[spi->port], data);

		/* Wait for data */
		while (SPI_I2S_GetFlagStatus(spi_port[spi->port], SPI_I2S_FLAG_RXNE) == RESET);

		/* Read data */
		result = SPI_I2S_ReceiveData(spi_port[spi->port]);
	}

	return result;
}

uint32_t spi_w16r32(uint16_t data)
{
	return spi_w16r16(data);
}

uint8_t spi_w32r8(uint32_t data)
{
	return spi_w16r16(data) & 0xff;
}

uint16_t spi_w32r16(uint32_t data)
{
	return spi_w16r16(data);
}

uint32_t spi_w32r32(uint32_t data)
{
	return spi_w16r16(data);
}

int spi_xfer_8(struct spi_xfer_8 *request)
{
	return -ERR_NOIMP;
}

int spi_xfer_16(struct spi_xfer_16 *request)
{
	return -ERR_NOIMP;
}

int spi_xfer_32(struct spi_xfer_32 *request)
{
	return -ERR_NOIMP;
}
