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
#include "display.h"

#define MS_PER_SEC		1000
#define DEBOUNCE_DELAY		40
#define TPRINTF_QUEUE_SIZE	256

#define PWM_TIMER	TIM2
#define PWM_APB1	RCC_APB1Periph_TIM2

#define SPI		SPI1
#define SPI_IRQ		SPI1_IRQn
#define SPI_APB1	0
#define SPI_APB2	RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1
#define SPI_DIRECTION	SPI_Direction_2Lines_FullDuplex
#define SPI_MODE	SPI_Mode_Master
#define SPI_DATA_SIZE	SPI_DataSize_16b
#define SPI_CLK_POL	SPI_CPOL_Low
#define SPI_CLK_PHASE	SPI_CPHA_2Edge
#define SPI_SLAVE_MODE	SPI_NSS_Soft
#define SPI_GPIO	GPIOA
#define SPI_PIN_SCK	GPIO_Pin_5
#define SPI_PIN_MISO	GPIO_Pin_6
#define SPI_PIN_NSS	GPIO_Pin_4

/* Function Prototypes */
static void setup_rcc(void);
static void setup_gpio(void);
#if 0
static void setup_exti(void);
#endif
static void setup_rtc(void);
static void setup_timer(void);
static void setup_usart(void);
static void setup_spi(void);
static void setup_nvic(void);
static void main_noreturn(void) NORETURN;

static void blink_task(void *pvParameters) NORETURN;
static void debounce_task(void *pvParameters) NORETURN;
static void setup(void);
static void blink_toggle_green1(void);
static void blink_toggle_green2(void);

enum button_state
{
	BUTTON_STATE_UP,
	BUTTON_STATE_DOWN
};

static xQueueHandle tprintf_queue;
static xSemaphoreHandle debounce_sem;

static enum button_state button_state;
static uint8_t led_green1 = 1;
static uint8_t led_green2 = 1;

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

/**
 * Main function
 */
int main(void)
{
	main_noreturn();
}

inline void main_noreturn(void)
{
	xTaskHandle task;

	/* Create the blink task */
	xTaskCreate(blink_task, (signed portCHAR *)"blink", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 1, &task);
	assert_param(task);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();
	assert_param(NULL);

	while (1);
}

static inline uint16_t spi_w16r16(uint16_t data)
{
	/* Send data */
	SPI->DR = data;

	/* Wait for data */
	while ((SPI->SR & SPI_I2S_FLAG_RXNE) == 0);

	/* Read data */
	return SPI->DR;
}

static inline uint16_t spi_r16()
{
	return spi_w16r16(0);
}

static inline void setup()
{
	setup_rcc();
	setup_gpio();
#if 0
	setup_exti();
#endif
	setup_rtc();
	setup_timer();
	setup_usart();
	setup_spi();
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
			PWM_APB1 |
			SPI_APB1 |
			RCC_APB1Periph_BKP, ENABLE);

	/* Enable GPIOA and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
			SPI_APB2 |
			RCC_APB2Periph_GPIOB |
			RCC_APB2Periph_GPIOD |
			RCC_APB2Periph_USART1 |
			RCC_APB2Periph_AFIO, ENABLE);

	/* Enable FSMC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports
 * @param  None
 * @retval None
 */
void setup_gpio(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Configure PA9 as output */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	/* Configure PD12 & PD13 as output */
	gpio_init.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpio_init);

	/* Configure buzzer pin */
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
	gpio_init.GPIO_Pin = GPIO_Pin_7;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_init);

	/* Configure PWM pin */
	gpio_init.GPIO_Pin = GPIO_Pin_8;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_init);

	/* Configure LCD backlight pin */
	gpio_init.GPIO_Pin = GPIO_Pin_10;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_init);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

#if 0
	/* Configure touchscreen interrupt pin */
	gpio_init.GPIO_Pin = GPIO_Pin_11;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_init);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_Pin_11);
#endif

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
}

#if 0
/**
  * @brief  Configures EXTI Lines
  * @param  None
  * @retval None
  */
void setup_exti(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_InitStructure.EXTI_Line = GPIO_EXTILineConfig;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
#endif

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
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC second interrupt */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Set the RTC time base to 1sec */
	RTC_SetPrescaler(32767);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Clear the RTC second interrupt */
	RTC_ClearITPendingBit(RTC_IT_SEC);

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
	TIM_OCInitTypeDef tim_oc_init;
	RCC_ClocksTypeDef RCC_ClockFreq;
	uint16_t ccr_value;

	/* Set CCR value based on PCLK1 frequency and desired buzzer frequency */
	RCC_GetClocksFreq(&RCC_ClockFreq);
	ccr_value = (RCC_ClockFreq.PCLK1_Frequency / 5000);

	/* Configure timer */
	tim_init.TIM_Prescaler = 0;
	tim_init.TIM_Period = (SystemCoreClock / 5000) - 1;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(PWM_TIMER, &tim_init);

	/* Channel 1 configuration in PWM mode */
	tim_oc_init.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
	tim_oc_init.TIM_OutputNState = TIM_OutputNState_Enable;
	tim_oc_init.TIM_Pulse = (SystemCoreClock / 5000 / 2) - 1;
	tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_Low;
	tim_oc_init.TIM_OCNPolarity = TIM_OCNPolarity_High;
	tim_oc_init.TIM_OCIdleState = TIM_OCIdleState_Set;
	tim_oc_init.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC3Init(PWM_TIMER, &tim_oc_init);

	TIM_OC3PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(PWM_TIMER, ENABLE);

	/* Timer main output enable */
	TIM_CtrlPWMOutputs(PWM_TIMER, ENABLE);

	/* Timer counter enable */
	TIM_Cmd(PWM_TIMER, ENABLE);
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
	spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	spi_init.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init(SPI, &spi_init);

	/* Enable SPI */
	SPI_Cmd(SPI, ENABLE);
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
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	/* Configure RTC interrupt */
	nvic_init.NVIC_IRQChannel = RTC_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

#if 0
	/* Configure EXTI interrupt */
	nvic_init.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0xc;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
#endif
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

#if 0
		blink_toggle_green2();
#endif

		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_SEC);

		/* Reset RTC Counter when Time is 23:59:59 */
		if (counter == 0x00015180)
		{
			RTC_SetCounter(0);
		}

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
	}
}

#if 0
/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void exti15_10_isr(void)
{
	static signed portBASE_TYPE task_woken;

	/* Clear pending bit */
	EXTI_ClearITPendingBit(EXTI_Line11);

	xSemaphoreGiveFromISR(debounce_sem, &task_woken);

	portEND_SWITCHING_ISR(task_woken);
}
#endif

void blink_toggle_green1()
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, led_green1);
	led_green1 ^= 1;
}

void blink_toggle_green2()
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_13, led_green2);
	led_green2 ^= 1;
}

#define DISPLAY_NUM_TEST_BARS 6

static uint16_t display_test_bars[DISPLAY_NUM_TEST_BARS] =
{
	WHITE,
	YELLOW,
	MAGENTA,
	RED,
	CYAN,
	GREEN
};

static void display_test(void)
{
	uint16_t i = 0;

	struct display_rect rect =
	{
		.x1 = 0,
		.y1 = 0,
		.x2 = DISPLAY_PIXEL_WIDTH - 1,
		.y2 = (DISPLAY_PIXEL_HEIGHT / DISPLAY_NUM_TEST_BARS) - 1
	};

	for (i = 0; i < DISPLAY_NUM_TEST_BARS; i++)
	{
		display_fill_rect(&rect, display_test_bars[i]);
		rect.y1 += DISPLAY_PIXEL_HEIGHT / DISPLAY_NUM_TEST_BARS;
		rect.y2 += DISPLAY_PIXEL_HEIGHT / DISPLAY_NUM_TEST_BARS;
	}
}


void blink_task(void *pvParameters)
{
	portTickType last_wake = xTaskGetTickCount();
	xTaskHandle task;

	/*
	 * If using FreeRTOS, tprintf must not be called until after this queue
	 * has been created
	 */
	tprintf_queue = xQueueCreate(TPRINTF_QUEUE_SIZE, sizeof(unsigned char));
	assert_param(tprintf_queue);

	vSemaphoreCreateBinary(debounce_sem);
	assert_param(debounce_sem);

	setup();

	/* Create the button task */
	xTaskCreate(debounce_task, (signed portCHAR *)"debounce", configMINIMAL_STACK_SIZE , NULL, tskIDLE_PRIORITY + 2, &task);
	assert_param(task);

	display_init();
	display_test();

	for (;;)
	{
		//vTaskDelayUntil(&last_wake, (1 * MS_PER_SEC) / portTICK_RATE_MS);
		vTaskDelayUntil(&last_wake, 500);
		blink_toggle_green1();
#if 0
		tprintf("tick=%d\r\n", last_wake);
#endif
	}
}

void debounce_task(void *pvParameters)
{
	portTickType delay = portMAX_DELAY;
	uint8_t debounce = 0;

	for (;;)
	{
		if (xSemaphoreTake(debounce_sem, delay) == pdTRUE)
		{
			if (!debounce)
			{
				debounce = 1;
				delay = DEBOUNCE_DELAY;
			}
		}
		else
		{
			volatile uint8_t button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

			if (button_state == BUTTON_STATE_UP && button)
			{
				button_state = BUTTON_STATE_DOWN;
				blink_toggle_green2();
				tprintf("button press\r\n");
			}
			else if (button_state == BUTTON_STATE_DOWN && !button)
			{
				button_state = BUTTON_STATE_UP;
				tprintf("button release\r\n");
			}

			debounce = 0;
			delay = portMAX_DELAY;
		}
	}
}
