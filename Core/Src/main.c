/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtc.h"
#include "LiquidCrystal_I2C.h"
#include "Button.h"
#include "DFPLAYER.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
	alarm_mode,
	idle_mode,
	adjust_mode,
	timer_mode,
	select_ringtone,
}clock_mode;
clock_mode Mode = idle_mode;

typedef enum
{
	backlight_on,
	backlight_off,
}backlight_state;
backlight_state lcd_backlight = backlight_on;

typedef enum
{
	no_adjust,
	second_adjust,
	minute_adjust,
	hour_adjust,
	day_adjust,
	month_adjust,
	year_adjust,
}time_adjust;
time_adjust adjust_state = no_adjust;

typedef struct
{
	uint8_t hour;
	uint8_t minute;

}Time_alarm;
Time_alarm Time_to_alarm;

typedef enum
{
	alarm_not_set,
	alarm_setted,

}alarm_state;
alarm_state state_alarm = alarm_not_set;

typedef enum
{
	alarm_is_ringing,
	alarm_is_not_ring,
}alarm_has_ring_or_not;
alarm_has_ring_or_not alarm_flag = alarm_is_not_ring;

typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t milisec;
}Timer_time;
Timer_time display_timer;

bool timer_state = false;

LiquidCrystal_I2C hlcd;
RTC_Typedef ds3231;
DFPLAYER_Name mini_player;
Button_Typdef Button3;
Button_Typdef Button4;
Button_Typdef Button5;
uint32_t temp = 0;

uint8_t demo_track = 1;
uint8_t track_num = 1;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim2.Instance)
  {
	  if(display_timer.sec == 59)
	  {

		  display_timer.sec = 0;
		  if(display_timer.min == 59)
		  {
			  display_timer.min = 0;
			  display_timer.hour++;
		  }
		  else{
		  display_timer.min++;
		  }
	  }
	  else
	  {
		  display_timer.sec++;
	  }

  }
}


void btn_press_short_callback(Button_Typdef *ButtonX )
{
	if(ButtonX == &Button3)
	{
		if (Mode == idle_mode)
		{
			Mode = adjust_mode;
			lcd_clear_display(&hlcd);
			HAL_Delay(1);
		}
		else if (Mode == alarm_mode)
		{
			state_alarm = alarm_setted;
			Mode = idle_mode;
			lcd_clear_display(&hlcd);
			HAL_Delay(1);
		}
		else
		{
			adjust_state = no_adjust;
			Mode = idle_mode;
			lcd_clear_display(&hlcd);
			HAL_Delay(1);
		}
	}
	if(ButtonX == &Button4)
	{
		if (Mode == adjust_mode)
		{
			switch(adjust_state)
			{
			case no_adjust:
				adjust_state = second_adjust;
				break;
			case second_adjust:
				adjust_state = minute_adjust;
				break;
			case minute_adjust:
				adjust_state = hour_adjust;
				break;
			case hour_adjust:
				adjust_state = day_adjust;
				break;
			case day_adjust:
				adjust_state = month_adjust;
				break;
			case month_adjust:
				adjust_state = year_adjust;
				break;
			case year_adjust:
				adjust_state = no_adjust;
				break;
			}
		}
		else if (Mode == alarm_mode)
		{
			switch(adjust_state)
			{
			case no_adjust:
				adjust_state = minute_adjust;
				break;
			case minute_adjust:
				adjust_state = hour_adjust;
				break;
			case hour_adjust:
				adjust_state = no_adjust;
				break;
			default:
				break;
			}
		}
		else if (Mode == timer_mode)
		{
			// start/stop  timer
			if(timer_state == false)
			{
				HAL_TIM_Base_Start_IT(&htim2);
				timer_state = true;
			}
			else
			{
				HAL_TIM_Base_Stop_IT(&htim2);
				timer_state = false;
			}

		}
		else if (Mode == select_ringtone)
		{
			if(demo_track <9)
			{
				demo_track++;
			}
			else
			{
				demo_track = 1;
			}
			DFPLAYER_PlayTrack(&mini_player, demo_track);
		}
	}
	if(ButtonX == &Button5)
	{
		if(state_alarm == alarm_setted && alarm_flag == alarm_is_ringing)
		{
			state_alarm = alarm_not_set;
			alarm_flag = alarm_is_not_ring;
			DFPLAYER_Stop(&mini_player);
		}
		else{
			if(Mode == idle_mode)
			{
				if(lcd_backlight == backlight_on)
				{
					lcd_backlight_off(&hlcd);
					lcd_backlight = backlight_off;
				}
				else if(lcd_backlight == backlight_off)
				{
					lcd_backlight_on(&hlcd);
					lcd_backlight = backlight_on;
				}
			}
			if (Mode == alarm_mode)
			{
				// adjust time to alarm
				switch (adjust_state) {
					case minute_adjust:
						if(Time_to_alarm.minute == 59)
						{
							Time_to_alarm.minute = 0;
						}
						else
						{
							Time_to_alarm.minute++;
						}
						break;
					case hour_adjust:
						if(Time_to_alarm.hour == 23)
						{
							Time_to_alarm.hour = 0;
						}
						else
						{
							Time_to_alarm.hour++;
						}
						break;
					default:
						break;
				}
			}
			if(Mode == adjust_mode)
			{
				// adjust time
				switch(adjust_state)
				{
				case second_adjust:
					RTC_ReadTime(&ds3231, ds3231.time);
					if(ds3231.time->sec == 59)
					{
						ds3231.time->sec = 0;
					}
					else
					{
						ds3231.time->sec++;
					}
					RTC_WriteTime(&ds3231, ds3231.time);
					break;
				case minute_adjust:
					RTC_ReadTime(&ds3231, ds3231.time);
					if(ds3231.time->min == 59)
					{
						ds3231.time->min = 0;
					}
					else
					{
						ds3231.time->min++;
					}
					RTC_WriteTime(&ds3231, ds3231.time);
					break;
				case hour_adjust:
					RTC_ReadTime(&ds3231, ds3231.time);
					if(ds3231.time->hour == 23)
					{
						ds3231.time->hour = 0;
					}
					else
					{
						ds3231.time->hour++;
					}
					RTC_WriteTime(&ds3231, ds3231.time);
					break;
				case day_adjust:
					RTC_ReadTime(&ds3231, ds3231.time);
					if(ds3231.time->date == 31)
					{
						ds3231.time->date = 1;
					}
					else
					{
						ds3231.time->date++;
					}
					RTC_WriteTime(&ds3231, ds3231.time);
					break;
				case month_adjust:
					RTC_ReadTime(&ds3231, ds3231.time);
					if(ds3231.time->month == 12)
					{
						ds3231.time->month = 1;
					}
					else
					{
						ds3231.time->month++;
					}
					RTC_WriteTime(&ds3231, ds3231.time);
					break;
				case year_adjust:
					RTC_ReadTime(&ds3231, ds3231.time);
					if(ds3231.time->year == 99)
					{
						ds3231.time->year = 1;
					}
					else
					{
						ds3231.time->year++;
					}
					RTC_WriteTime(&ds3231, ds3231.time);
					break;
				case no_adjust:
					break;
				}
			}
			if(Mode == timer_mode)
			{
				// reset timer
				HAL_TIM_Base_Stop(&htim2);
				display_timer.hour = 0;
				display_timer.min = 0;
				display_timer.sec = 0;
				htim2.Instance->CNT = 0;
				timer_state = false;
			}
			if(Mode == select_ringtone)
			{
				track_num = demo_track;
				DFPLAYER_Stop(&mini_player);
				Mode = idle_mode;
				lcd_clear_display(&hlcd);
				HAL_Delay(1);
			}
		}
	}
}

void btn_press_timeout1s_callback(Button_Typdef *ButtonX)
{
	if(ButtonX == &Button3)
	{
		Mode = select_ringtone;
		lcd_clear_display(&hlcd);
		HAL_Delay(1);
	}
}

void btn_press_timeout2s_callback(Button_Typdef *ButtonX)
{
	if(ButtonX == &Button3)
	{
		adjust_state = no_adjust;
		Mode = alarm_mode;
		lcd_clear_display(&hlcd);
		HAL_Delay(1);
//		temp++;
	}
	if(ButtonX == &Button4)
	{
//		if(Mode == alarm_mode)
//		{
//			state_alarm = alarm_setted;
//		}
	}
}

void btn_press_timeout3s_callback(Button_Typdef *ButtonX)
{
	if(ButtonX == &Button3)
	{
		adjust_state = no_adjust;
		Mode = timer_mode;
		lcd_clear_display(&hlcd);
		HAL_Delay(1);
	}
}

void alarm_handle()
{
	  if(state_alarm == alarm_setted && alarm_flag == alarm_is_not_ring)
	  {
		  if(Time_to_alarm.hour == ds3231.time->hour && Time_to_alarm.minute == ds3231.time->min)
		  {
			  alarm_flag = alarm_is_ringing;
			  DFPLAYER_PlayTrack(&mini_player, track_num);
		  }
	  }
	  if(state_alarm == alarm_setted && alarm_flag == alarm_is_ringing)
	  {

		  if(HAL_GetTick() - temp >= 1000){
			  if(lcd_backlight ==  backlight_on){
				  lcd_backlight_off(&hlcd);
				  lcd_backlight = backlight_off;
			  }
			  else{
				  lcd_backlight_on(&hlcd);
				  lcd_backlight = backlight_on;
			  }
			  temp = HAL_GetTick();
		  }
	  }
}
void display_handle()
{
	switch(Mode)
	{
	case idle_mode:
		RTC_ReadTime(&ds3231,ds3231.time);
		lcd_set_cursor(&hlcd, 1,4);
		lcd_printf(&hlcd, "%02d:%02d:%02d",ds3231.time->hour,
				ds3231.time->min, ds3231.time->sec);
		lcd_set_cursor(&hlcd, 0,4);
		lcd_printf(&hlcd, "%02d-%02d-%02d",ds3231.time->date,
				ds3231.time->month,ds3231.time->year);
		break;
	case adjust_mode:
		RTC_ReadTime(&ds3231,ds3231.time);
		lcd_set_cursor(&hlcd, 1,4);
		lcd_printf(&hlcd, "%02d:%02d:%02d",ds3231.time->hour,
				ds3231.time->min, ds3231.time->sec);
		lcd_set_cursor(&hlcd, 0,4);
		lcd_printf(&hlcd, "%02d-%02d-%02d",ds3231.time->date,
				ds3231.time->month,ds3231.time->year);
		break;
	case timer_mode:
		lcd_set_cursor(&hlcd, 0, 2);
		lcd_printf(&hlcd, "Sport Timer");
		lcd_set_cursor(&hlcd, 1, 2);
		lcd_printf(&hlcd, "%02d:%02d:%02d.%03d",display_timer.hour, display_timer.min, display_timer.sec, htim2.Instance->CNT);
		break;
	case alarm_mode:
		lcd_set_cursor(&hlcd, 0, 2);
		lcd_printf(&hlcd, "Alarm Mode");
		lcd_set_cursor(&hlcd, 1,2);
		lcd_printf(&hlcd, "%02d-%02d",Time_to_alarm.hour, Time_to_alarm.minute);
		break;
	case select_ringtone:
		lcd_set_cursor(&hlcd, 0, 2);
		lcd_printf(&hlcd, "Select ringtone");
		lcd_set_cursor(&hlcd, 1,0);
		lcd_printf(&hlcd, "Track number:%02d",demo_track);
		break;

	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	DateTime date1;
 	date1.date = 1;
	date1.day = 31;
	date1.month = 11;
	date1.year = 24;
	date1.hour = 10;
	date1.min = 30;
	date1.sec = 0;
	RTC_WriteTime(&ds3231, &date1);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  button_init(&Button3, GPIOA, GPIO_PIN_3);
  button_init(&Button4, GPIOA, GPIO_PIN_4);
  button_init(&Button5, GPIOA, GPIO_PIN_5);
  RTC_Init(&ds3231, &hi2c1, &date1);
  DFPLAYER_Init(&mini_player, &huart1);
  lcd_init(&hlcd, &hi2c1, LCD_ADDR_DEFAULT);
  DFPLAYER_SetVolume(&mini_player, 27);

//  lcd_printf(&hlcd, "Ngu_vl");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  lcd_backlight_off(&hlcd);
//	  HAL_Delay(500);
//	  lcd_backlight_on(&hlcd);
//	  HAL_Delay(500);
	  button_handle(&Button3);
	  button_handle(&Button4);
	  button_handle(&Button5);
	  display_handle();
	  alarm_handle();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
