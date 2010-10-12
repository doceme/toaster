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
#define BTN_IRQ		EXTI0_IRQn
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

#define TIMER		TIM7
#define TIMER_IRQ	TIM7_IRQn
#define TIMER_APB1	RCC_APB1Periph_TIM7
#define TIMER_APB2	0

#define MAX6675_MASK_STATE	0x1
#define MAX6675_MASK_DEVICE_ID	0x2
#define MAX6675_MASK_TC_INPUT	0x4
#define MAX6675_MASK_TC_TEMP	0x7ff8
#define MAX6675_MASK_DUMMY	0x8000

#define DEBOUNCE_DELAY		40

/* All time data is in milliseconds unless otherwise stated */
/* All temperature data is in degrees Celcius unless otherwise stated */
/* All rate data is in degrees Celcius per second unless otherwise stated */

enum thermal_state
{
	OFF,
	PREHEAT_RAMPUP,
	PREHEAT,
	REFLOW_RAMPUP,
	REFLOW,
	COOLDOWN
};

struct thermocouple
{
	uint16_t temp;
	uint16_t temp_prev;
	uint8_t open;
};

struct limits
{
	uint16_t preheat_temp_min;
	uint16_t preheat_temp_max;
	uint32_t preheat_time_min;
	uint32_t preheat_time_max;
	float preheat_rampup_rate;
	float preheat_rate;
	uint16_t reflow_temp_min;
	uint16_t reflow_temp_max;
	uint32_t reflow_time_min;
	uint32_t reflow_time_max;
	float reflow_rampup_rate;
	float reflow_rate;
	uint16_t cooldown_temp_min;
	uint32_t cooldown_time_max;
	float cooldown_rate;
};

struct toaster
{
	enum thermal_state state;
	struct thermocouple tc;
	uint32_t elapsed;
	uint32_t counter;
	uint8_t max_preheat_reached;
	uint8_t max_reflow_reached;
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
static void TIM_Configuration(void);
static void main_noreturn(void) NORETURN;

static void toaster_off();
static void toaster_preheat();
static void toaster_reflow();
static void toaster_cooldown();

static uint8_t debounce;
static struct toaster oven;
static struct limits profile =
{
	.preheat_temp_min = 100,
	.preheat_temp_max = 150,
	.preheat_time_min = 60 * 1000,
	.preheat_time_max = 120 * 1000,
	.reflow_temp_min = 183,
	.reflow_temp_max = 235,
	.reflow_time_min = 10 * 1000,
	.reflow_time_max = 60 * 1000,
	.preheat_rampup_rate = 2,
	.preheat_rate = 0.5,
	.reflow_rampup_rate = 2.5,
	.reflow_rate = 3,
	.cooldown_temp_min = 50,
	.cooldown_time_max = 10 * 60 * 1000,
	.cooldown_rate = 5,
};

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
	/* Enable APB1 clocks */
	RCC_APB1PeriphClockCmd(
		(LED_APB1 |
			BTN_APB1 |
			RTC_APB1 |
			USART_APB1 |
			SPI_APB1 |
			TIMER_APB1),
		ENABLE);

	/* Enable APB2 clocks */
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

	/* Enable RTC interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable button EXTI interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = BTN_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xf;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set SPI interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SPI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable timer interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
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
	SPI_I2S_ITConfig(SPI, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable SPI */
	SPI_Cmd(SPI, ENABLE);
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

	/* Enable timer interrupt */
	TIM_ITConfig(TIMER, TIM_IT_Update, ENABLE);
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

	if (!debounce)
	{
		debounce = DEBOUNCE_DELAY;

		/* Enable timer counter */
		TIM_Cmd(TIMER, ENABLE);
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

		/* Toggle blue LED */
		GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, oven.led_blue);
		oven.led_blue ^= 1;

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

	/* Get SPI data */
	data = SPI_I2S_ReceiveData(SPI);

	/* Set thermocouple data */
	oven.tc.temp_prev = oven.tc.temp;
	oven.tc.temp = ((data & MAX6675_MASK_TC_TEMP) >> 3) >> 2;
	oven.tc.open = ((data & MAX6675_MASK_TC_INPUT) >> 2);

	if (oven.tc.open)
	{
		toaster_off();
		tprintf("Thermocouple is open!\r\n");
	}
}

/**
  * @brief  This function handles timer interrupt request.
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
	/* Clear the timer update pending bit */
	TIM_ClearITPendingBit(TIMER, TIM_IT_Update);

	if (!debounce)
	{
		oven.elapsed++;
		oven.counter++;

		/* Activate slave select */
		GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_RESET);

		/* Start temperature read */
		SPI_I2S_SendData(SPI, 0);

		if (oven.elapsed % 3000 == 0)
		{
			tprintf("%d,%d\r\n", oven.elapsed, oven.tc.temp);
		}

		switch (oven.state)
		{
			case OFF:
				break;
			case PREHEAT_RAMPUP:
			case PREHEAT:
			{
				toaster_preheat();
			} break;

			case REFLOW_RAMPUP:
			case REFLOW:
			{
				toaster_reflow();
			} break;

			case COOLDOWN:
			{
				toaster_cooldown();
			} break;

			default:
			{
				toaster_off();
				tprintf("Invalid state! %d\r\n", oven.state);
			} break;
		}
	}
	else
	{
		debounce--;

		if (!debounce)
		{
			if (GPIO_ReadInputDataBit(BTN_GPIO, BTN_PIN) == RESET)
			{
				switch (oven.state)
				{
					case OFF:
					{
						toaster_preheat();
					} break;

					default:
					{
						toaster_cooldown();
					} break;
				}
			}
			else /* False positive */
			{
				/* Disable timer counter */
				TIM_Cmd(TIMER, DISABLE);
			}
		}
	}
}

void toaster_goto(enum thermal_state state)
{
	switch (state)
	{
		case OFF:
		{
			tprintf(">OFF\r\n");
		} break;

		case PREHEAT_RAMPUP:
		{
			tprintf(">PREHEAT_RAMPUP\r\n");
		} break;

		case PREHEAT:
		{
			tprintf(">PREHEAT\r\n");
		} break;

		case REFLOW_RAMPUP:
		{
			tprintf(">REFLOW_RAMPUP\r\n");
		} break;

		case REFLOW:
		{
			tprintf(">REFLOW\r\n");
		} break;

		case COOLDOWN:
		{
			tprintf(">COOLDOWN\r\n");
		} break;

		default:
		{
			tprintf("Invalid state!\r\n");
			toaster_off();
		} return;
	}

	oven.counter = 0;
	oven.state = state;
}

void toaster_off()
{
	/* Disable timer counter */
	TIM_Cmd(TIMER, DISABLE);

	/* Disable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, DISABLE);

	/* Turn off green LED */
	GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_RESET);

	/* Turn off blue LED */
	GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, Bit_RESET);
	oven.led_blue = 0;

	toaster_goto(OFF);
}

void toaster_preheat()
{
	if (oven.state == OFF)
	{
		/* Turn on green LED */
		GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_SET);

		/* Enable the RTC second interrupt */
		RTC_ITConfig(RTC_IT_SEC, ENABLE);

		/* Enable timer counter */
		TIM_Cmd(TIMER, ENABLE);

		/* Reset toaster variables */
		oven.elapsed = 0;
		oven.max_preheat_reached = 0;
		oven.max_reflow_reached = 0;

		toaster_goto(PREHEAT_RAMPUP);
	}
	else if (oven.state == PREHEAT_RAMPUP)
	{
		if (oven.tc.temp >= profile.preheat_temp_min)
		{
			toaster_goto(PREHEAT);
		}
		else if (oven.counter >= profile.preheat_time_max)
		{
			toaster_reflow();
		}
	}
	else if (oven.state == PREHEAT)
	{
		if (oven.counter >= profile.preheat_time_max)
		{
			toaster_reflow();
		}
	}
	else
	{
		toaster_off();
		tprintf("Invalid state!\r\n");
	}
}

void toaster_reflow()
{
	if ((oven.state == PREHEAT) || (oven.state == PREHEAT_RAMPUP))
	{
		toaster_goto(REFLOW_RAMPUP);
	}
	else if (oven.state != REFLOW_RAMPUP)
	{
		if (oven.tc.temp >= profile.reflow_temp_min)
		{
			toaster_goto(REFLOW);
		}
		else if (oven.counter >= profile.reflow_time_max)
		{
			toaster_cooldown();
		}
	}
	else if (oven.state != REFLOW)
	{
		if (oven.counter >= profile.reflow_time_max)
		{
			toaster_cooldown();
		}
	}
	else
	{
		toaster_off();
		tprintf("Invalid state!\r\n");
	}
}

void toaster_cooldown()
{
	if (oven.state != OFF)
	{
		if (oven.state != COOLDOWN)
		{
			toaster_goto(COOLDOWN);
		}
		else
		{
			if (oven.tc.temp <= profile.cooldown_temp_min ||
				oven.counter >= profile.cooldown_time_max)
			{
				toaster_off();
			}
		}
	}
}
