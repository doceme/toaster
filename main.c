/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @author     Michael Spradling Copyright (C) 2010
 * @brief      Main toaster application
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
#include "tprintf.h"

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void USART_Configuration(void);
static void EXTI_Configuration(void);
static void RTC_Configuration(void);
static void NVIC_Configuration(void);
static void SPI_Configuration(void);
static void main_noreturn(void) NORETURN;

static BitAction led_user = Bit_SET;
static BitAction led_rtc = Bit_SET;

static uint16_t tc_temp = 0;
static uint8_t tc_open = 1;
static uint32_t elapsed_seconds = 0;

#define USART		USART1
#define USART_GPIO	GPIOA
#define USART_PIN_TX	GPIO_Pin_9

#define SPI_MASTER	SPI2
#define SPI_GPIO	GPIOB
#define SPI_PIN_SCK	GPIO_Pin_13
#define SPI_PIN_MISO	GPIO_Pin_14
#define SPI_PIN_NSS	GPIO_Pin_12

#define MAX6675_MASK_STATE	0x1
#define MAX6675_MASK_DEVICE_ID	0x2
#define MAX6675_MASK_TC_INPUT	0x4
#define MAX6675_MASK_TC_TEMP	0x7ff8
#define MAX6675_MASK_DUMMY	0x8000

void assert_failed(uint8_t *function, uint32_t line)
{
	while (1);
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int outbyte(int ch) {
	while (USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
	USART_SendData(USART, (uint8_t)ch);
	while (USART_GetFlagStatus(USART, USART_FLAG_TC) == RESET);
	return ch;
}

/**
 * Main function
 */
int main(void)
{
	main_noreturn();
}

inline void main_noreturn(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	tprintf("\r\nBooting Toaster Application Version 1.0...\r\n");
	EXTI_Configuration();
	RTC_Configuration();
	NVIC_Configuration();
	SPI_Configuration();
	tprintf("Init Complete.\r\n");
	tprintf("Time (s),Temp (c)\r\n");

	while (1);
}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP | RCC_APB1Periph_SPI2, ENABLE);

	/* Enable GPIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC5 (ADC Channel15) as analog input */
	GPIO_WriteBit(GPIOC, GPIO_Pin_8 | GPIO_Pin_9, Bit_SET);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure button input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Deactivate slave select */
	GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_SET);

	/* Configure SPI_MASTER pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_PIN_NSS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	/* Configure pinout for UART1 */
	GPIO_InitStructure.GPIO_Pin = USART_PIN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART_GPIO, &GPIO_InitStructure);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
}

/**
 * @brief  Configures the USART.
 * @param  None
 * @retval None
 */
void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	/* Configure the USART */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;

	USART_Init(USART, &USART_InitStructure);

	/* Enable the USART */
	USART_Cmd(USART, ENABLE);
}

/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configures RTC clock source and prescaler
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
	/* RTC clock source configuration ------------------------------------------*/
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Set the RTC time base to 1min */
	RTC_SetPrescaler(32767);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	//RTC_ITConfig(RTC_IT_ALR, ENABLE);

	/* Enable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	//NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set Button EXTI interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set SPI interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the SPI controller.
  * @param  None
  * @retval None
  */
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;

	/* Initialize SPI Master */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_MASTER, &SPI_InitStructure);

	/* Enable SPI interrupt */
	SPI_I2S_ITConfig(SPI_MASTER, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable SPI_MASTER */
	SPI_Cmd(SPI_MASTER, ENABLE);
}

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* Clear the Key Button EXTI line pending bit */
	EXTI_ClearITPendingBit(EXTI_Line0);

	GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_user);
	led_user ^= 1;
}

/**
  * @brief  This function handles RTC interrupt request
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
	{
		uint32_t counter = RTC_GetCounter();

		/* Activate slave select */
		GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_RESET);

		/* Start temperature read */
		SPI_I2S_SendData(SPI_MASTER, 0);

		/* Toggle LED */
		GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_rtc);
		led_rtc ^= 1;

		elapsed_seconds++;

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_SEC);

		/* Reset RTC Counter when Time is 23:59:59 */
		if (counter == 0x00015180)
		{
			RTC_SetCounter(0);
			/* Wait until last write operation on RTC registers has finished */
			RTC_WaitForLastTask();
		}
	}
}

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	uint16_t data;

	/* Deactivate slave select */
	GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_SET);
	data = SPI_I2S_ReceiveData(SPI_MASTER);
	tc_temp = ((data & MAX6675_MASK_TC_TEMP) >> 3) >> 2;
	tc_open = ((data & MAX6675_MASK_TC_INPUT) >> 2);

	if (!tc_open)
	{
		tprintf("%d,%d\r\n", elapsed_seconds, tc_temp);
	}
	else
	{
		tprintf("TC open!\r\n");
	}
}
