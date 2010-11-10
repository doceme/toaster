/**
 ******************************************************************************
 * * @file       main.c
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

/* STM32 standard peripheral library */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* Other includes */
#include <stdlib.h>
#include "common.h"
#include "spi.h"
#include "uart.h"
#include "tprintf.h"

#define LED_GPIO	GPIOC
#define LED_APB1	0
#define LED_APB2	RCC_APB2Periph_GPIOC
#define LED_PIN_BLUE	GPIO_Pin_8
#define LED_PIN_GREEN	GPIO_Pin_9

#define BTN_GPIO	GPIOA
#define BTN_IRQ		EXTI0_IRQn
#define BTN_APB1	0
#define BTN_APB2	RCC_APB2Periph_GPIOA
#define BTN_PIN		GPIO_Pin_0
#define BTN_PORT_SRC	GPIO_PortSourceGPIOA
#define BTN_PIN_SRC	GPIO_PinSource0
#define BTN_EXTI_LINE	EXTI_Line0
#define BTN_EXTI_MODE	EXTI_Mode_Interrupt
#define BTN_EXTI_TRG	EXTI_Trigger_Falling

#define RTC_APB1	(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP)
#define RTC_APB2	0

#define TIMER		TIM7
#define TIMER_IRQ	TIM7_IRQn
#define TIMER_APB1	RCC_APB1Periph_TIM7
#define TIMER_APB2	0

#define OVN_PWM_TIM	TIM3
#define OVN_PWM_APB1	RCC_APB1Periph_TIM3
#define OVN_PWM_APB2	RCC_APB2Periph_GPIOA
#define OVN_PWM_GPIO	GPIOA
#define OVN_PWM_PIN	GPIO_Pin_6
#define OVN_PWM_MS	((SystemCoreClock / 100000) - 1)

#define MAX6675_MASK_STATE	0x1
#define MAX6675_MASK_DEVICE_ID	0x2
#define MAX6675_MASK_TC_INPUT	0x4
#define MAX6675_MASK_TC_TEMP	0x7ff8
#define MAX6675_MASK_DUMMY	0x8000

#define DEBOUNCE_DELAY		40

#define MAX_OUTPUT		100
#define MAX_ERROR		50

#define TEST_PREHEAT		150

#define MS_PER_SEC		1000

/* All time data is in milliseconds unless otherwise stated */
/* All temperature data is in degrees Celcius unless otherwise stated */
/* All rate data is in degrees Celcius per second unless otherwise stated */

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
	uint16_t temp_prev;
	uint8_t open;
};

struct limits
{
	uint16_t warmup_temp_min;
	uint8_t warmup_output;
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
	int output;
	uint8_t max_preheat_reached;
	uint8_t max_reflow_reached;
	BitAction led_blue;
};

struct pid_params
{
	float kp;
	float ki;
	float kd;
	float integral;
	int error_prev;
};

/* Function Prototypes */
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
static void RTC_Configuration(void);
static void NVIC_Configuration(void);
static void TIM_Configuration(void);

static void main_task(void *pvParameters) NORETURN;
static void main_noreturn(void) NORETURN;

static void toaster_off();
#if 0
static void toaster_warmup();
static void toaster_preheat();
static void toaster_reflow();
static void toaster_cooldown();
#endif

static xTaskHandle xMainTask = NULL;

static void toaster_set_output(int output);

static uint8_t debounce;
static struct toaster oven;
#if 0
static struct limits profile =
{
	.warmup_temp_min = 50,
	.warmup_output = 40,
	.preheat_temp_min = 100,
	.preheat_temp_max = 150,
	.preheat_time_min = 100 * 1000,
	.preheat_time_max = 180 * 1000,
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

static struct pid_params pid =
{
	.kp = 4,
	.ki = 0.035,
	.kd = 45,
	.integral = 30,
	.error_prev = 0
};
#endif

static struct spi_device max6675 =
{
	.data_bits = 16,
	.port = 1,
	.gpio_port = 1,
	.pin_clk = GPIO_Pin_13,
	.pin_mosi = GPIO_Pin_14,
	.pin_cs = GPIO_Pin_12,
	.mode = SPI_MODE_1,
	.speed = 4000000
};

static struct uart_device debug =
{
	.port = 0,
	.gpio_port = 0,
	.pin_tx = GPIO_Pin_9,
	.speed = 115200,
	.data_bits = 8,
	.stop_bits = 1
};

#if 0
/**
  * @brief  PID in C, Error computed inside the routine
  * @param : Input (measured value)
  *   Ref: reference (target value)
  *   Coeff: pointer to the coefficient table
  * @retval : PID output (command)
  */
static float pid_calc(int error, struct pid_params *p)
{
	float result = 0;
	float dterm;
	float ferror = (float)error;
	float pterm = p->kp * ferror;

	if (pterm > MAX_ERROR || pterm < -MAX_ERROR)
	{
		p->integral = 0;
	}
	else
	{
		p->integral += p->ki * ferror;

		if (p->integral > MAX_ERROR)
		{
			p->integral = MAX_ERROR;
		}
		else if (p->integral < -MAX_ERROR)
		{
			p->integral = 0;
		}
	}

	dterm = p->kd * ((float)(error - p->error_prev));
	result = pterm + p->integral + dterm;

	p->error_prev = error;

	return result;
}
#endif

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
	uart_write_char(ch);
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
	uart_init(&debug);
	tprintf("\r\n=== Toaster Reflow Oven ===\r\n");
	EXTI_Configuration();
	RTC_Configuration();
	NVIC_Configuration();
	TIM_Configuration();
	spi_init(&max6675);
	tprintf("Init Complete.\r\n");
	tprintf("Time (s),Temp (c)\r\n");

	/* Create a FreeRTOS task */
	xTaskCreate(main_task, (signed portCHAR *)"main", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &xMainTask);
	assert_param(xMainTask);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

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
			TIMER_APB1 |
			OVN_PWM_APB1),
		ENABLE);

	/* Enable APB2 clocks */
	RCC_APB2PeriphClockCmd(
		(LED_APB2 |
			BTN_APB2 |
			RTC_APB2 |
			TIMER_APB2 |
			OVN_PWM_APB2),
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

	/* Configure the main PWM to control the oven */
	GPIO_InitStructure.GPIO_Pin = OVN_PWM_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(OVN_PWM_GPIO, &GPIO_InitStructure);

	/* Connect button EXTI Line to button GPIO pin */
	GPIO_EXTILineConfig(BTN_PORT_SRC, BTN_PIN_SRC);
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

	/* Enable timer interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure the timer controller.
  * @param  None
  * @retval None
  */
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* Configure timer */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000 ) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 2;
	TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);

	/* Set Period */
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 2000) - 1;
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInit(OVN_PWM_TIM, &TIM_TimeBaseStructure);

	/* Enable timer counter */
	TIM_Cmd(TIMER, ENABLE);

	/* Set Duty Cycle */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(OVN_PWM_TIM, &TIM_OCInitStructure);

	/* Enable oven PWM timer */
	TIM_OC1PreloadConfig(OVN_PWM_TIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(OVN_PWM_TIM, ENABLE);
	TIM_Cmd(OVN_PWM_TIM, ENABLE);
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

		/* Enable timer interrupt */
		TIM_ITConfig(TIMER, TIM_IT_Update, ENABLE);
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

#if 0
		if ((oven.elapsed % 250) == 0)
		{
			/* Activate slave select */
			GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_RESET);

			/* Start temperature read */
			SPI_I2S_SendData(SPI, 0);
		}
#endif
	}
	else
	{
		debounce--;

		if (!debounce)
		{
			if (GPIO_ReadInputDataBit(BTN_GPIO, BTN_PIN) == RESET)
			{
				uint16_t data = 0;
				uint16_t temp = 0;

				tprintf("reading...\r\n", data);

				spi_request(&max6675);
				spi_cs(SPI_CS_ACTIVATE);
				data = spi_r16();
				spi_cs(SPI_CS_DEACTIVATE);

				temp = ((data & MAX6675_MASK_TC_TEMP) >> 3) >> 2;
				tprintf("data=0x%04x\r\n", data);
				tprintf("temp=%d\r\n", temp);

				/* Disable timer interrupt */
				TIM_ITConfig(TIMER, TIM_IT_Update, DISABLE);
#if 0
				switch (oven.state)
				{
					case OFF:
					{
						toaster_warmup();
					} break;

					default:
					{
						toaster_off();
					} break;
				}
#endif
			}
			else /* False positive */
			{
				/* Disable timer interrupt */
				TIM_ITConfig(TIMER, TIM_IT_Update, DISABLE);
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

		case WARMUP:
		{
			tprintf(">WARMUP\r\n");
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
	/* Disable timer interrupt */
	TIM_ITConfig(TIMER, TIM_IT_Update, DISABLE);

	/* Disable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, DISABLE);

	/* Turn off green LED */
	GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_RESET);

	/* Turn off blue LED */
	GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, Bit_RESET);
	oven.led_blue = 0;

	/* Turn off the oven */
	toaster_set_output(0);

	toaster_goto(OFF);
}

#if 0
void toaster_warmup()
{
	if (oven.state != WARMUP)
	{
		/* Reset toaster variables */
		oven.elapsed = 0;
		oven.max_preheat_reached = 0;
		oven.max_reflow_reached = 0;

		/* Update state */
		toaster_goto(WARMUP);

		/* Turn on green LED */
		GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_SET);

		/* Enable the RTC second interrupt */
		RTC_ITConfig(RTC_IT_SEC, ENABLE);

		toaster_set_output(profile.warmup_output);
	}

	if (oven.tc.temp >= profile.warmup_temp_min)
	{
		toaster_preheat();
	}
}

void toaster_preheat()
{
	if (oven.state == WARMUP)
	{
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
#ifdef TEST_PREHEAT
			toaster_off();
#else
			toaster_reflow();
#endif
		}
	}
	else if (oven.state == PREHEAT)
	{
		if (oven.counter >= profile.preheat_time_max)
		{
#ifdef TEST_PREHEAT
			toaster_off();
#else
			toaster_reflow();
#endif
		}
	}
	else
	{
		toaster_off();
		tprintf("Invalid state!\r\n");
	}

	if (oven.state == PREHEAT_RAMPUP || oven.state == PREHEAT)
	{
#ifdef TEST_PREHEAT
		oven.output = TEST_PREHEAT;
#endif
		toaster_set_output(pid_calc(oven.output - oven.tc.temp, &pid));
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
#endif

void toaster_set_output(int output)
{
	uint8_t duty;

	if (output > MAX_OUTPUT)
	{
		duty = MAX_OUTPUT;
	}
	else if (output < 0)
	{
		duty = 0;
	}
	else
	{
		duty = (uint8_t)output;
	}

	tprintf("output=%d\r\n", output);
	tprintf("duty=%d%%\r\n", duty);

	TIM_SetCompare1(OVN_PWM_TIM, duty * 10);
}

void main_task(void *pvParameters)
{
	portTickType xLastWakeTime;

	/* Initialise the xLastExecutionTime variable on task entry */
	xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		/* Toggle green LED */
		GPIO_SetBits(LED_GPIO, LED_PIN_GREEN);
		vTaskDelay((1 * MS_PER_SEC) / portTICK_RATE_MS);
		GPIO_ResetBits(LED_GPIO, LED_PIN_GREEN);
		vTaskDelay((1 * MS_PER_SEC) / portTICK_RATE_MS);
	}
}
