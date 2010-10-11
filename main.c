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
#include "stm32_dsp.h"

#define LED_GPIO	GPIOC
#define LED_APB1	0
#define LED_APB2	RCC_APB2Periph_GPIOC
#define LED_PIN_BLUE	GPIO_Pin_8
#define LED_PIN_GREEN	GPIO_Pin_9

#define BTN_GPIO	GPIOA
#define BTN_IRQ	EXTI0_IRQn
#define BTN_APB1	0
#define BTN_APB2	RCC_APB2Periph_GPIOA
#define BTN_PIN	GPIO_Pin_0
#define BTN_PORT_SRC	GPIO_PortSourceGPIOA
#define BTN_PIN_SRC	GPIO_PinSource0
#define BTN_EXTI_LINE	EXTI_Line0
#define BTN_EXTI_MODE	EXTI_Mode_Interrupt
#define BTN_EXTI_TRG	EXTI_Trigger_Falling

#define RTC_APB1	(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP)
#define RTC_APB2	0

#define USART		USART1
#define USART_APB1	0
#define USART_APB2	RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1
#define USART_GPIO	GPIOA
#define USART_PIN_TX	GPIO_Pin_9
#define USART_BAUDRATE	115200
#define USART_WORD_LEN	USART_WordLength_8b
#define USART_STOP_BITS	USART_StopBits_1
#define USART_PARITY	USART_Parity_No
#define USART_HW_FLOW	USART_HardwareFlowControl_None
#define USART_MODE	USART_Mode_Tx

#define SPI		SPI2
#define SPI_IRQ		SPI2_IRQn
#define SPI_APB1	RCC_APB1Periph_SPI2
#define SPI_APB2	RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB
#define SPI_DIRECTION	SPI_Direction_2Lines_FullDuplex
#define SPI_MODE	SPI_Mode_Master
#define SPI_DATA_SIZE	SPI_DataSize_16b
#define SPI_CLK_POL	SPI_CPOL_Low
#define SPI_CLK_PHASE	SPI_CPHA_2Edge
#define SPI_SLAVE_MODE	SPI_NSS_Soft
#define SPI_GPIO	GPIOB
#define SPI_PIN_SCK	GPIO_Pin_13
#define SPI_PIN_MISO	GPIO_Pin_14
#define SPI_PIN_NSS	GPIO_Pin_12

#define DMA		DMA1_Channel4

#define TIMER		TIM7
#define TIMER_IRQ	TIM7_IRQn
#define TIMER_APB1	RCC_APB1Periph_TIM7
#define TIMER_APB2	0

#define MAX6675_MASK_STATE	0x1
#define MAX6675_MASK_DEVICE_ID	0x2
#define MAX6675_MASK_TC_INPUT	0x4
#define MAX6675_MASK_TC_TEMP	0x7ff8
#define MAX6675_MASK_DUMMY	0x8000

enum thermal_state
{
	OFF,
	WARMUP,
	PREHEAT_RAMPUP,
	PREHEAT,
	REFLOW_RAMPUP,
	REFLOW,
	COOLDOWN
};

struct thermocouple
{
	uint16_t temp;
	uint8_t open;
};

struct toaster
{
	enum thermal_state state;
	struct thermocouple tc;
	uint32_t time_elapsed; /* in milliseconds */
	BitAction led_blue;
};

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void USART_Configuration(void);
static void EXTI_Configuration(void);
static void RTC_Configuration(void);
static void NVIC_Configuration(void);
static void SPI_Configuration(void);
static void DMA_Configuration(void);
static void TIM_Configuration(void);
static void main_noreturn(void) NORETURN;

static struct toaster oven;
static uint16_t dma;

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
	DMA_Configuration();
	TIM_Configuration();
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
	/* Enable APB1 peripherals */
	RCC_APB1PeriphClockCmd(
		(LED_APB1 |
			BTN_APB1 |
			RTC_APB1 |
			USART_APB1 |
			SPI_APB1 |
			TIMER_APB1),
		ENABLE);

	RCC_APB2PeriphClockCmd(
		(LED_APB2 |
			BTN_APB2 |
			RTC_APB2 |
			USART_APB2 |
			SPI_APB2 |
			TIMER_APB2),
		ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure LED pins */
	GPIO_InitStructure.GPIO_Pin = LED_PIN_BLUE | LED_PIN_GREEN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);

	/* Configure button pins */
	GPIO_InitStructure.GPIO_Pin = BTN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BTN_GPIO, &GPIO_InitStructure);

	/* Configure SPI pins */
	/* Deactivate slave select */
	GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_SET);
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_NSS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	/* Configure USART pins */
	GPIO_InitStructure.GPIO_Pin = USART_PIN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USART_GPIO, &GPIO_InitStructure);

	/* Connect button EXTI Line to button GPIO pin */
	GPIO_EXTILineConfig(BTN_PORT_SRC, BTN_PIN_SRC);
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
	USART_InitStructure.USART_BaudRate = USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WORD_LEN;
	USART_InitStructure.USART_StopBits = USART_STOP_BITS;
	USART_InitStructure.USART_Parity = USART_PARITY;
	USART_InitStructure.USART_HardwareFlowControl = USART_HW_FLOW;
	USART_InitStructure.USART_Mode = USART_MODE;

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

	/* Configure the external interrupt */
	EXTI_ClearITPendingBit(BTN_EXTI_LINE);
	EXTI_InitStructure.EXTI_Line = BTN_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = BTN_EXTI_MODE;
	EXTI_InitStructure.EXTI_Trigger = BTN_EXTI_TRG;
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

	/* Set the RTC time base to 1 second */
	RTC_SetPrescaler(32767);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	//RTC_ITConfig(RTC_IT_ALR, ENABLE);

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
	NVIC_InitStructure.NVIC_IRQChannel = BTN_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set SPI interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = SPI_IRQ;
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

	/* Configure SPI controller */
	SPI_InitStructure.SPI_Direction = SPI_DIRECTION;
	SPI_InitStructure.SPI_Mode = SPI_MODE;
	SPI_InitStructure.SPI_DataSize = SPI_DATA_SIZE;
	SPI_InitStructure.SPI_CPOL = SPI_CLK_POL;
	SPI_InitStructure.SPI_CPHA = SPI_CLK_PHASE;
	SPI_InitStructure.SPI_NSS = SPI_SLAVE_MODE;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI, &SPI_InitStructure);

	/* Enable SPI interrupt */
	//SPI_I2S_ITConfig(SPI, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable SPI */
	SPI_Cmd(SPI, ENABLE);
}

/**
  * @brief  Configures the DMA controller.
  * @param  None
  * @retval None
  */
void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	/* Configure DMA channel */
	DMA_DeInit(DMA);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SPI2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dma;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA, &DMA_InitStructure);

	/* Enable DMA channel */
	DMA_Cmd(DMA, ENABLE);
}

/**
  * @brief  Configure the timer controller.
  * @param  None
  * @retval None
  */
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Configure timer */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000 ) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 2;
	TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);

	/* Enable SPI interrupt */
	TIM_ITConfig(TIMER, TIM_IT_Update, ENABLE);

	/* Timer DMA request enable */
	TIM_DMACmd(TIMER, TIM_DMA_Update, ENABLE);

	/* Enable timer counter */
	TIM_Cmd(TIMER, ENABLE);
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

	switch (oven.state)
	{
		case OFF:
		{
			tprintf(">WARMUP\r\n");
			oven.state = WARMUP;

			/* Turn on green LED */
			GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_SET);

			/* Enable the RTC second interrupt */
			RTC_ITConfig(RTC_IT_SEC, ENABLE);
		} break;

		default:
		{
			tprintf(">OFF\r\n");
			oven.state = OFF;

			/* Turn off green LED */
			GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_RESET);

			/* Turn off blue LED */
			GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, Bit_RESET);
			oven.led_blue = 0;

			/* Disable the RTC second interrupt */
			RTC_ITConfig(RTC_IT_SEC, DISABLE);
		} break;
	}
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
		//SPI_I2S_SendData(SPI, 0);

		/* Toggle blue LED */
		GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, oven.led_blue);
		oven.led_blue ^= 1;

		oven.time_elapsed+=1000;

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

#if 0
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
	data = SPI_I2S_ReceiveData(SPI);
	oven.tc.temp = ((data & MAX6675_MASK_TC_TEMP) >> 3) >> 2;
	oven.tc.open = ((data & MAX6675_MASK_TC_INPUT) >> 2);

	if (!oven.tc.open)
	{
		tprintf("%d,%d\r\n", oven.time_elapsed, oven.tc.temp);
	}
	else
	{
		tprintf("Thermocouple is open!\r\n");
	}
}
#endif

/**
  * @brief  This function handles timer interrupt request.
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
	/* Clear the timer update pending bit */
	TIM_ClearITPendingBit(TIMER, TIM_IT_Update);
}
