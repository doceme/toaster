/**
 ******************************************************************************
 * @file       stm32_uart.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @author     Michael Spradling Copyright (C) 2010
 * @brief      STM32 UART Implementation
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
#include "uart.h"

#if (defined(STM32F10X_LD) || defined(STM32F10X_LD_VL))
#define UART_NUM_PORTS	2
#elif (defined(STM32F10X_MD) || defined(STM32F10X_MD_VL))
#define UART_NUM_PORTS	3
#else
#define UART_NUM_PORTS	5
#endif

static uint32_t uart_irq[UART_NUM_PORTS] =
{
	USART1_IRQn,
	USART2_IRQn,
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	USART3_IRQn,
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	UART4_IRQn,
	UART5_IRQn
#endif
};

static USART_TypeDef* uart_port[UART_NUM_PORTS] =
{
	USART1,
	USART2,
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	USART3,
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	UART4,
	UART5
#endif
};

static struct stm32_apb_periph uart_apb[UART_NUM_PORTS] =
{
	{ /* USART1 */
		.clock = RCC_APB2PeriphClockCmd,
		.periph = RCC_APB2Periph_USART1
	},
	{ /* USART2 */
		.clock = RCC_APB1PeriphClockCmd,
		.periph = RCC_APB1Periph_USART2
	},
#if defined(STM32F10X_MD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	{ /* USART3 */
		.clock = RCC_APB1PeriphClockCmd,
		.periph = RCC_APB1Periph_USART3
	},
#endif
#if defined(STM32F10X_HD) || defined(STM32F10X_XD) || defined(STM32F10X_CL)
	{ /* UART4 */
		.clock = RCC_APB1PeriphClockCmd,
		.periph = RCC_APB1Periph_UART4
	},
	{ /* UART5 */
		.clock = RCC_APB1PeriphClockCmd,
		.periph = RCC_APB1Periph_UART5
	}
#endif
};

static struct uart_device *uart;

int uart_init(struct uart_device *device)
{
	int result = -1;

	if (device)
	{
		GPIO_InitTypeDef gpio_init;
		NVIC_InitTypeDef nvic_init;
		USART_InitTypeDef usart_init;

		uart = device;

		/* Enable APB clocks */
		if (uart_apb[uart->port].clock)
		{
			uart_apb[uart->port].clock(uart_apb[uart->port].periph, ENABLE);
		}
		if (gpio_apb[uart->gpio_port].clock)
		{
			gpio_apb[uart->gpio_port].clock(gpio_apb[uart->gpio_port].periph, ENABLE);
		}

		/* Configure UART tx pin */
		gpio_init.GPIO_Pin = uart->pin_tx;
		gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
		gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(gpio_port[uart->gpio_port], &gpio_init);

		/* Configure UART rx pin */
		gpio_init.GPIO_Pin = uart->pin_rx;
		gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(gpio_port[uart->gpio_port], &gpio_init);

		/* Enable and set SPI interrupt */
		nvic_init.NVIC_IRQChannel = uart_irq[uart->port];
		nvic_init.NVIC_IRQChannelPreemptionPriority = 0x0;
		nvic_init.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic_init);

		/* Configure the USART */
		usart_init.USART_BaudRate = uart->speed;

		if (uart->data_bits == 9)
		{
			usart_init.USART_WordLength = USART_WordLength_9b;
		}
		else
		{
			usart_init.USART_WordLength = USART_WordLength_8b;
		}

		usart_init.USART_StopBits = uart->stop_bits;


		switch (uart->parity)
		{
			case UART_PARITY_ODD:
				usart_init.USART_Parity = USART_Parity_Odd;
				break;

			case UART_PARITY_EVEN:
				usart_init.USART_Parity = USART_Parity_Even;
				break;

			default:
				usart_init.USART_Parity = USART_Parity_No;
				break;
		}

		switch (uart->flow_control)
		{
			case UART_FLOW_RTS:
				usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;
				break;

			case UART_FLOW_CTS:
				usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
				break;

			case UART_FLOW_BOTH:
				usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
				break;

			default:
				usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
				break;
		}

		usart_init.USART_Mode = USART_Mode_Tx;

		USART_Init(uart_port[uart->port], &usart_init);

		/* Enable the USART */
		USART_Cmd(uart_port[uart->port], ENABLE);

		result = 0;
	}

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
	return 0;
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

	return 0;
}

int uart_read_async(struct uart_xfer *request)
{
	return -ERR_NOIMP;
}

int uart_write_async(struct uart_xfer *request)
{
	return -ERR_NOIMP;
}
