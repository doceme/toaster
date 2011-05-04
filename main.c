/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Stephen Caudle Copyright (C) 2010
 * @brief      Main implementation
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

/* STM32 includes */
#include <stm32f10x.h>
#include <stm32f10x_conf.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* Includes */
#include "common.h"
#include "tprintf.h"

#define MS_PER_SEC		1000
#define DEBOUNCE_DELAY		40
#define TPRINTF_QUEUE_SIZE	512

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
#define BTN_EXTI_TRG	EXTI_Trigger_Rising_Falling

#define SSR_GPIO	GPIOA
#define SSR_APB1	0
#define SSR_APB2	RCC_APB2Periph_GPIOA
#define SSR_PIN		GPIO_Pin_6

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

#define DEBOUNCE_TIMER		TIM4
#define DEBOUNCE_TIMER_IRQ	TIM4_IRQn
#define DEBOUNCE_TIMER_APB1	RCC_APB1Periph_TIM4
#define DEBOUNCE_TIMER_APB2	0

#define PROFILE_TIMER		TIM2
#define PROFILE_TIMER_IRQ	TIM2_IRQn
#define PROFILE_TIMER_APB1	RCC_APB1Periph_TIM2
#define PROFILE_TIMER_APB2	0

#define MAX6675_MASK_STATE	0x1
#define MAX6675_MASK_DEVICE_ID	0x2
#define MAX6675_MASK_TC_INPUT	0x4
#define MAX6675_MASK_TC_TEMP	0x7ff8
#define MAX6675_MASK_DUMMY	0x8000

#define PDM_NUM_BITS		30
#define PDM_TABLE_SIZE		32

#define MAX_OUTPUT		PDM_TABLE_SIZE - 1
#define MAX_ERROR		MAX_OUTPUT
#define MIN_ERROR		0

#define OVN_TEMP_TOLERANCE	1

//#define DISABLE_OVEN

/* All time data is in milliseconds unless otherwise stated */
/* All temperature data is in degrees Celcius unless otherwise stated */
/* All rate data is in degrees Celcius per second unless otherwise stated */

enum thermal_state
{
	OFF,
	WARMUP,
	PREHEAT,
	SOAK,
	RAMPUP,
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
	uint8_t running;
	uint16_t warmup_temp;
	uint8_t warmup_output;
	uint16_t soak_temp;
	uint32_t soak_time;
	uint16_t reflow_temp;
	uint32_t reflow_time;
	uint16_t cooldown_temp;
	uint32_t cooldown_time;
};

struct toaster
{
	enum thermal_state state;
	struct thermocouple tc;
	uint32_t elapsed_total;
	uint32_t elapsed_warmup;
	uint32_t elapsed_preheat;
	uint32_t elapsed_soak;
	uint32_t elapsed_reflow_rampup;
	uint32_t elapsed_reflow;
	uint32_t elapsed_cooldown;
	uint32_t counter;
	uint8_t output;
	uint8_t max_temp_reached;
	uint8_t soak_reached;
	uint8_t reflow_reached;
	BitAction led_blue;
	BitAction led_green;
};

struct pid_params
{
	float kp;
	float ki;
	float kd;
	float integral;
	float derror_prev;
};

enum button_state
{
	BUTTON_STATE_UP,
	BUTTON_STATE_DOWN
};

/* Function Prototypes */
static void setup(void);
static void setup_rcc(void);
static void setup_gpio(void);
static void setup_exti(void);
static void setup_rtc(void);
static void setup_timer(void);
static void setup_spi(void);
static void setup_usart(void);
static void setup_nvic(void);
static void main_noreturn(void) NORETURN;

static void blink_toggle_green(void);

static void profile_task(void *pvParameters) NORETURN;
static void debug_task(void *pvParameters) NORETURN;

static int pid_calc(int error, struct pid_params *p, float derror);

static void toaster_goto(enum thermal_state state);
static void toaster_update();
static void toaster_off();
static void toaster_warmup();
static void toaster_soak();
static void toaster_reflow();
static void toaster_cooldown();
static void toaster_set_output(int output);

static xTaskHandle profile_task_handle;
static xQueueHandle tprintf_queue;
static xSemaphoreHandle debug_sem;

static uint8_t debounce;
static enum button_state button_state;

static struct toaster oven;
static struct limits profile =
{
	.warmup_temp = 50,
	.warmup_output = 13,
	.soak_temp = 160,
	.soak_time = 120 * 1000,
	.reflow_temp = 255,
	.reflow_time = 30 * 1000,
	.cooldown_temp = 50,
	.cooldown_time = 10 * 60 * 1000,
};

static struct pid_params soak_pid =
{
	.kp = 1.0,
	.ki = 0.003,
	.kd = 100,
	.integral = 0,
	.derror_prev = 0
};

static struct pid_params reflow_pid =
{
	.kp = 4.1,
	.ki = 0.1,
	.kd = 100,
	.integral = 0,
	.derror_prev = 0
};

/* Only the first 30 bits are significant @ 60Hz */
static const uint32_t pdm_table_60hz[PDM_TABLE_SIZE] =
{
	0x00000000, // 0000 0000 0000 0000 0000 0000 0000 0000
	0x00000002, // 0000 0000 0000 0000 0000 0000 0000 0010
	0x00008002, // 0000 0000 0000 0000 1000 0000 0000 0010
	0x00100802, // 0000 0000 0001 0000 0000 1000 0000 0010
	0x00808202, // 0000 0000 1000 0000 1000 0010 0000 0010
	0x01042082, // 0000 0001 0000 0100 0010 0000 1000 0010
	0x02108842, // 0000 0010 0001 0000 1000 1000 0100 0010
	0x04444444, // 0000 0100 0100 0100 0100 0100 0100 0100
	0x0888a222, // 0000 1000 1000 1000 1010 0010 0010 0010
	0x09120925, // 0000 1001 0001 0010 0000 1001 0010 0101
	0x0924a492, // 0000 1001 0010 0100 1010 0100 1001 0010
	0x12521495, // 0001 0010 0101 0010 0001 0100 1001 0101
	0x1294ca52, // 0001 0010 1001 0100 1100 1010 0101 0010
	0x152a2955, // 0001 0101 0010 1010 0010 1001 0101 0101
	0x15555554, // 0001 0101 0101 0101 0101 0101 0101 0100
	0x15552aab, // 0001 0101 0101 0101 0010 1010 1010 1011
	0x2aaad554, // 0010 1010 1010 1010 1101 0101 0101 0100
	0x2aaaaaab, // 0010 1010 1010 1010 1010 1010 1010 1011
	0x2ad5d6aa, // 0010 1010 1101 0101 1101 0110 1010 1010
	0x2b5b2d6b, // 0010 1011 0101 1011 0010 1101 0110 1011
	0x2dadeb6c, // 0010 1101 1010 1101 1110 1011 0110 1100
	0x2db6b6dd, // 0010 1101 1011 0110 1011 0110 1101 1011
	0x36ddf6dc, // 0011 0110 1101 1101 1111 0110 1101 1100
	0x37775ddd, // 0011 0111 0111 0111 0101 1101 1101 1101
	0x3bbbbbbb, // 0011 1011 1011 1011 1011 1011 1011 1011
	0x3bdeef7b, // 0011 1011 1101 1110 1110 1111 0111 1011
	0x3dfbdf7d, // 0011 1101 1111 1011 1101 1111 0111 1101
	0x3f7f7dfd, // 0011 1111 0111 1111 0111 1101 1111 1101
	0x3feff7fd, // 0011 1111 1110 1111 1111 0111 1111 1101
	0x3fff7ffb, // 0011 1111 1111 1111 0111 1111 1111 1011
	0x3ffffffb, // 0011 1111 1111 1111 1111 1111 1111 1011
	0x3fffffff, // 0011 1111 1111 1111 1111 1111 1111 1111
};

static uint8_t pdm_index;
static uint32_t pdm_bit_count;
static uint32_t pdm_bit_slice;

static inline void pdm_reset()
{
	pdm_bit_count = 0;
	pdm_bit_slice = pdm_table_60hz[pdm_index];
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch The character to print
 * @retval The character printed
 */
int outbyte(int ch)
{
	/* Enable USART TXE interrupt */
	xQueueSendToBack(tprintf_queue, &ch, 0);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	return ch;
}

static inline uint16_t spi_r16()
{
	/* Send data */
	SPI->DR = 0;

	/* Wait for data */
	while ((SPI->SR & SPI_I2S_FLAG_RXNE) == 0);

	/* Read data */
	return SPI->DR;
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
	/* Create the blink task */
	xTaskCreate(profile_task, (signed portCHAR *)"blink", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &profile_task_handle);
	assert_param(profile_task_handle);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();
	assert_param(NULL);

	while (1);
}

static inline void setup()
{
	setup_rcc();
	setup_gpio();
	setup_exti();
	setup_rtc();
	setup_timer();
	setup_spi();
	setup_usart();
	setup_nvic();

	tprintf("stm32_template\r\n");
}

/**
 * @brief  Configures the peripheral clocks
 * @param  None
 * @retval None
 */
void setup_rcc(void)
{
	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR |
			SSR_APB1 |
			DEBOUNCE_TIMER_APB1 |
			PROFILE_TIMER_APB1 |
			LED_APB1 |
			SPI_APB1 |
			RCC_APB1Periph_BKP, ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(SSR_APB2 |
			RCC_APB2Periph_USART1 |
			LED_APB2 |
			SPI_APB2 |
			RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports
 * @param  None
 * @retval None
 */
void setup_gpio(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Configure UART tx pin */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure PC5 (ADC Channel15) as analog input */
	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpio_init);

	/* Deactivate SPI chip select pin */
	GPIO_WriteBit(SPI_GPIO, SPI_PIN_NSS, Bit_SET);

	/* Configure SPI clock and data pins */
	gpio_init.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MISO;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &gpio_init);

	/* Configure SPI chip select pins */
	gpio_init.GPIO_Pin = SPI_PIN_NSS;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &gpio_init);

	/* Configure solid state relay pin */
	gpio_init.GPIO_Pin = SSR_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SSR_GPIO, &gpio_init);

	/* Configure button input floating */
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
}

/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void setup_exti(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configures RTC clock source and prescaler
  * @param  None
  * @retval None
  */
void setup_rtc(void)
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

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Configures timer controller
  * @param  None
  * @retval None
  */
void setup_timer(void)
{
	TIM_TimeBaseInitTypeDef tim_init;

	/* Configure debounce timer */
	tim_init.TIM_Prescaler = DEBOUNCE_DELAY - 1;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_Period = (SystemCoreClock / 1000 ) - 1;
	tim_init.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(DEBOUNCE_TIMER, &tim_init);

	/* Configure profile timer */
	tim_init.TIM_Prescaler = 1000 - 1;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_Period = (SystemCoreClock / 120 / 1000 ) - 1;
	tim_init.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(PROFILE_TIMER, &tim_init);

	/* Clear the debouce timer update pending bit */
	TIM_ClearITPendingBit(PROFILE_TIMER, TIM_IT_Update);

	/* Enable profile timer interrupt */
	TIM_ITConfig(PROFILE_TIMER, TIM_IT_Update, ENABLE);

	/* Enable the timer */
	TIM_Cmd(PROFILE_TIMER, ENABLE);

}

/**
  * @brief  Configures SPI controller
  * @param  None
  * @retval None
  */
void setup_spi(void)
{
	SPI_InitTypeDef spi_init;

	/* SPI configuration */
	spi_init.SPI_Direction = SPI_DIRECTION;
	spi_init.SPI_Mode = SPI_MODE;
	spi_init.SPI_DataSize = SPI_DATA_SIZE;
	spi_init.SPI_CPOL = SPI_CLK_POL;
	spi_init.SPI_CPHA = SPI_CLK_PHASE;
	spi_init.SPI_NSS = SPI_SLAVE_MODE;
	spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	spi_init.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init(SPI, &spi_init);

	/* Enable SPI */
	SPI_Cmd(SPI, ENABLE);
}

/**
  * @brief  Configures USART controller
  * @param  None
  * @retval None
  */
void setup_usart(void)
{
	USART_InitTypeDef usart_init;

	usart_init.USART_BaudRate = 115200;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = 1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &usart_init);

	/* Enable the USART */
	USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  Configure the nested vectored interrupt controller
  * @param  None
  * @retval None
  */
void setup_nvic(void)
{
	NVIC_InitTypeDef nvic_init;

	/* Set the Vector Table base address as specified in .ld file */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	/* 4 bits for Interupt priorities so no sub priorities */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	/* Configure USART interrupt */
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xf;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure RTC interrupt */
	nvic_init.NVIC_IRQChannel = RTC_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xe;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure debounce timer interrupt */
	nvic_init.NVIC_IRQChannel = DEBOUNCE_TIMER_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure profile timer interrupt */
	nvic_init.NVIC_IRQChannel = PROFILE_TIMER_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xd;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure EXTI interrupt */
	nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0x1;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void usart1_isr(void)
{
	portBASE_TYPE task_woken;

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		unsigned char ch;

		if (xQueueReceiveFromISR(tprintf_queue, &ch, &task_woken))
			USART_SendData(USART1, ch);
		else
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}

	portEND_SWITCHING_ISR(task_woken);
}

/**
  * @brief  This function handles RTC interrupt request
  * @param  None
  * @retval None
  */
void rtc_isr(void)
{
	if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
	{
		uint32_t counter = RTC_GetCounter();

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_SEC);

		blink_toggle_green();

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
  * @brief  This function handles timer interrupt request
  * @param  None
  * @retval None
  */
void tim4_isr(void)
{
	volatile uint8_t button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

	if (button_state == BUTTON_STATE_UP && button)
	{
		button_state = BUTTON_STATE_DOWN;
		tprintf("button press\r\n");
		vTaskResume(profile_task_handle);
	}
	else if (button_state == BUTTON_STATE_DOWN && !button)
	{
		button_state = BUTTON_STATE_UP;
		tprintf("button release\r\n");
	}

	/* Clear the debouce timer update pending bit */
	TIM_ClearITPendingBit(DEBOUNCE_TIMER, TIM_IT_Update);

	/* Disable the timer */
	TIM_Cmd(DEBOUNCE_TIMER, DISABLE);

	debounce = 0;
}

/**
  * @brief  This function handles timer interrupt request
  * @param  None
  * @retval None
  */
void tim2_isr(void)
{
	static signed portBASE_TYPE task_woken;
	uint32_t slice = pdm_bit_slice;

	/* Clear the profile timer update pending bit */
	TIM_ClearITPendingBit(PROFILE_TIMER, TIM_IT_Update);

	if (++pdm_bit_count >= PDM_NUM_BITS)
	{
		volatile uint16_t data;

		/* Read thermocouple temperature */
		SPI_GPIO->BRR = SPI_PIN_NSS;
		data = spi_r16();
		SPI_GPIO->BSRR = SPI_PIN_NSS;

		oven.tc.temp = ((data & MAX6675_MASK_TC_TEMP) >> 3) >> 2;
		toaster_update();

		oven.elapsed_total += 250;
		oven.counter += 250;
		pdm_reset();

		xSemaphoreGiveFromISR(debug_sem, &task_woken);
	}
	else
	{
		pdm_bit_slice >>= 1;
	}

	if (profile.running && (slice & 0x1))
		SSR_GPIO->BSRR = SSR_PIN;
	else
		SSR_GPIO->BRR = SSR_PIN;

	portEND_SWITCHING_ISR(task_woken);
}

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void exti0_isr(void)
{
	/* Clear pending bit */
	EXTI_ClearITPendingBit(EXTI_Line0);

	/* Disable timer interrupt */
	TIM_ITConfig(DEBOUNCE_TIMER, TIM_IT_Update, DISABLE);

	/* Reset timer counter */
	TIM_SetCounter(DEBOUNCE_TIMER, 0);

	/* Clear the debouce timer update pending bit */
	TIM_ClearITPendingBit(DEBOUNCE_TIMER, TIM_IT_Update);

	/* Enable timer interrupt */
	TIM_ITConfig(DEBOUNCE_TIMER, TIM_IT_Update, ENABLE);

	if (!debounce)
	{
		debounce = 1;

		/* Enable the timer */
		TIM_Cmd(DEBOUNCE_TIMER, ENABLE);
	}
}

void blink_toggle_green()
{
	oven.led_green ^= 1;
	GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, oven.led_green);
}

void profile_task(void *pvParameters)
{
	xTaskHandle task_handle;

	/*
	 * If using FreeRTOS, tprintf must not be called until after this queue
	 * has been created
	 */
	tprintf_queue = xQueueCreate(TPRINTF_QUEUE_SIZE, sizeof(unsigned char));
	assert_param(tprintf_queue);

	vSemaphoreCreateBinary(debug_sem);
	assert_param(debug_sem);

	/* Create the debug task */
	xTaskCreate(debug_task, (signed portCHAR *)"debug", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &task_handle);
	assert_param(task_handle);

	setup();

	for (;;)
	{
		vTaskSuspend(NULL);

		if (!profile.running)
		{
			profile.running = 1;
			toaster_warmup();
		}
		else
		{
			profile.running = 0;
			toaster_off();
		}
	}
}

void debug_task(void *pvParameters)
{
	static uint32_t prev_total = 0xffffffff;

	for (;;)
	{
		xSemaphoreTake(debug_sem, 250);
		if (oven.tc.temp > 0 && oven.elapsed_total != prev_total)
		{
			tprintf("pdm=%08x\r\n", pdm_bit_slice);
			tprintf(":%d,%d\r\n", oven.elapsed_total - 250, oven.tc.temp);
			prev_total = oven.elapsed_total;
		}
	}
}

/**
  * @brief  PID in C, Error computed inside the routine
  * @param : Input (measured value)
  *   Ref: reference (target value)
  *   Coeff: pointer to the coefficient table
  * @retval : PID output (command)
  */
int pid_calc(int error, struct pid_params *p, float derror)
{
	float ferror = (float)error;
	float pterm;
	float iterm;
	float dterm;

	pterm = p->kp * ferror;
	p->integral += ferror;

	if (p->integral > MAX_ERROR)
		p->integral = MAX_ERROR;
	else if (p->integral < MIN_ERROR)
		p->integral = 0;

	iterm = p->ki * p->integral;
	dterm = p->kd * (derror - p->derror_prev);

	p->derror_prev = derror;

	return (int)(pterm + iterm - dterm);
}

void toaster_goto(enum thermal_state state)
{
	switch (state)
	{
		case OFF:
		{
			oven.output = 0;
			tprintf(">OFF\r\n");
		} break;

		case WARMUP:
		{
			oven.output = profile.warmup_output;
			oven.elapsed_warmup = oven.counter;
			tprintf(">WARMUP\r\n");
		} break;

		case PREHEAT:
		{
			oven.output = profile.soak_temp;
			oven.elapsed_preheat = oven.counter;
			tprintf(">PREHEAT\r\n");
		} break;

		case SOAK:
		{
			oven.output = profile.soak_temp;
			oven.elapsed_soak = oven.counter;
			tprintf(">SOAK\r\n");
		} break;

		case RAMPUP:
		{
			oven.output = profile.reflow_temp;
			oven.elapsed_reflow_rampup = oven.counter;
			tprintf(">RAMPUP\r\n");
		} break;

		case REFLOW:
		{
			oven.output = profile.reflow_temp;
			oven.elapsed_reflow = oven.counter;
			tprintf(">REFLOW\r\n");
		} break;

		case COOLDOWN:
		{
			oven.output = 0;
			oven.elapsed_cooldown = oven.counter;
			tprintf(">COOLDOWN\r\n");
		} break;

		default:
		{
			oven.output = 0;
			tprintf("Invalid state!\r\n");
			toaster_off();
		} return;
	}

	oven.counter = 0;
	oven.state = state;
}

void toaster_update()
{
	switch (oven.state)
	{
		case WARMUP:
			toaster_warmup();
			break;

		case PREHEAT:
		case SOAK:
			toaster_soak();
			break;

		case RAMPUP:
		case REFLOW:
			toaster_reflow();
			break;

		case COOLDOWN:
			toaster_cooldown();
			break;

		default:
			break;
	}
}

void toaster_off()
{
	/* Disable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, DISABLE);

	/* Turn off green LED */
	GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_RESET);
	oven.led_green = 0;

	/* Turn off blue LED */
	GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, Bit_RESET);
	oven.led_blue = 0;

	/* Turn off the oven */
	toaster_set_output(0);

	toaster_goto(OFF);

	tprintf("Reflow process complete\r\n");
	tprintf("Total time elapsed: %d\r\n", oven.elapsed_total);
	tprintf("Max temp reached: %d\r\n", oven.max_temp_reached);
	tprintf("Soak reached: %d\r\n", oven.soak_reached);
	tprintf("Reflow reached: %d\r\n", oven.reflow_reached);
}

void toaster_warmup()
{
	if (oven.state != WARMUP)
	{
		/* Reset toaster variables */
		oven.elapsed_total = 0;
		oven.elapsed_warmup = 0;
		oven.elapsed_preheat = 0;
		oven.elapsed_soak = 0;
		oven.elapsed_reflow_rampup = 0;
		oven.elapsed_reflow = 0;
		oven.elapsed_cooldown = 0;
		oven.max_temp_reached = 0;
		oven.soak_reached = 0;
		oven.reflow_reached = 0;

		/* Update state */
		toaster_goto(WARMUP);

		/* Turn on green LED */
		GPIO_WriteBit(LED_GPIO, LED_PIN_GREEN, Bit_SET);
		oven.led_green = 1;

		/* Turn on blue LED */
		GPIO_WriteBit(LED_GPIO, LED_PIN_BLUE, Bit_SET);
		oven.led_blue = 1;

		/* Enable the RTC second interrupt */
		RTC_ITConfig(RTC_IT_SEC, ENABLE);

		pdm_reset();

		toaster_set_output(oven.output);
	}

	if (oven.tc.temp >= profile.warmup_temp)
	{
		toaster_soak();
	}
}

void toaster_soak()
{
	if (oven.state == WARMUP)
	{
		toaster_goto(PREHEAT);
	}
	else if (oven.state == PREHEAT)
	{
		if (oven.tc.temp >= (profile.soak_temp - OVN_TEMP_TOLERANCE))
		{
			oven.soak_reached = 1;
			toaster_goto(SOAK);
		}
	}
	else if (oven.state == SOAK)
	{
		if (oven.counter >= profile.soak_time)
		{
			toaster_reflow();
		}
	}
	else
	{
		toaster_off();
		tprintf("Invalid state!\r\n");
	}

	if (oven.state == PREHEAT || oven.state == SOAK)
	{
		toaster_set_output(pid_calc(profile.soak_temp - oven.tc.temp, &soak_pid, oven.tc.temp));
	}
}

void toaster_reflow()
{
	if ((oven.state == SOAK) || (oven.state == PREHEAT))
	{
		toaster_goto(RAMPUP);
	}
	else if (oven.state == RAMPUP)
	{
		if (oven.tc.temp >= (profile.reflow_temp - OVN_TEMP_TOLERANCE))
		{
			oven.reflow_reached = 1;
			toaster_goto(REFLOW);
		}
	}
	else if (oven.state == REFLOW)
	{
		if (oven.counter >= profile.reflow_time)
		{
			toaster_cooldown();
		}
	}
	else
	{
		toaster_off();
		tprintf("Invalid state!\r\n");
	}

	if (oven.state == RAMPUP || oven.state == REFLOW)
	{
		toaster_set_output(pid_calc(profile.reflow_temp - (float)oven.tc.temp, &reflow_pid, oven.tc.temp));
	}
}

void toaster_cooldown()
{
	if (oven.state != OFF)
	{
		if (oven.state != COOLDOWN)
		{
			toaster_goto(COOLDOWN);
			toaster_set_output(oven.output);
		}
		else
		{
			if (oven.tc.temp <= profile.cooldown_temp ||
				oven.counter >= profile.cooldown_time)
			{
				toaster_off();
			}
		}
	}
}

void toaster_set_output(int output)
{
#ifndef DISABLE_OVEN
	if (output > MAX_OUTPUT)
	{
		pdm_index = MAX_OUTPUT;
	}
	else if (output < 0)
	{
		pdm_index = 0;
	}
	else
	{
		pdm_index = (uint8_t)output;
	}

	tprintf("output=%d\r\n", output);
	tprintf("index=%d\r\n", pdm_index);
#endif
}
