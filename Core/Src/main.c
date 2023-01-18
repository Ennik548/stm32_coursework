/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c-lcd.h"
#include "stdio.h"
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
 TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

#define stepsperrev 4096

void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000/stepsperrev/rpm);
}

void stepper_half_drive (int step)
{
	switch (step){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case 3:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case 4:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case 5:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			break;

		case 6:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			break;

		case 7:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		}
}
void stepper_step_angle (float angle, int direction, int rpm)
{
	float anglepersequence = 0.703125;
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)
		{
			for (int step=7; step>=0; step--)
			{
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}

		else if (direction == 1)
		{
			for (int step=0; step<8; step++)
			{
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */
  lcd_init();
  int BTN1_Left = 0;
  int BTN3_OK = 0;
  int EXPOSTION_TIME = 0;
  int NUMBER_OF_SAMPLES = 0;
  int FIRE = 0;

  char EXPOSTION_TIME_str[4];
  char NUMBER_OF_SAMPLES_str[4];

  /* Infinite loop */
  stepper_step_angle(80, 0, 12);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */
  	  /* ПЕРЕКЛЮЧЕНИЕ ПО МЕНЮ НАЧАЛО */
  	  if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 1 && BTN3_OK == 0) {
  		  BTN1_Left += 1;
  	  }
  	  else if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 1 && BTN3_OK == 0) {
  		  BTN1_Left -= 1;
  	  }
  	  /* ПЕРЕКЛЮЧЕНИЕ ПО МЕНЮ КОНЕЦ */
  	  /* КРУГОВОЕ МЕНЮ НАЧАЛО */
  	  if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 1 && BTN1_Left == 3) {
  		  BTN1_Left = 0; // НАСТРОЙКА ПРАВОЙ ГРАНИЦЫ
  	  };
  	  if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 1 && BTN1_Left == -1) {
  		  BTN1_Left = 1; // НАСТРОЙКА ЛЕВОЙ ГРАНИЦЫ
  	  };
  	  /* КРУГОВОЕ МЕНЮ КОНЕЦ */

  	  /* ОБРАБОТКА ОК НАЧАЛО */
  	  if (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == 1) {
  	  		  BTN3_OK += 1;
  	  };
  	  if (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == 1 && BTN3_OK == 2) {
  	  		  BTN3_OK = 0;
  	  };
  	  /* ОБРАБОТКА ОК КОНЕЦ */

  	  /* ИНКРЕМЕНТАЦИЯ НАЧАЛО */


  	  /* ИНКРЕМЕНТАЦИЯ КОНЕЦ */
  	  sprintf(EXPOSTION_TIME_str, "%d", EXPOSTION_TIME);
  	  sprintf(NUMBER_OF_SAMPLES_str, "%d", NUMBER_OF_SAMPLES);


  	  /* ЭЛЕМЕНТЫ КРУГОВОГО МЕНЮ НАЧАЛО */
  	  if (BTN1_Left == 0) {
  		  lcd_put_cur(0, 0);
  		  lcd_send_string("<- Exposition ->");
  		  lcd_put_cur(1, 8);
  		  lcd_send_string(EXPOSTION_TIME_str);
  		  if (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 1 && BTN3_OK == 1 && EXPOSTION_TIME != 9999) {
  			  EXPOSTION_TIME += 1;
  		  };
  		  if (HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin) == 1 && BTN3_OK == 1 && EXPOSTION_TIME != 0) {
  			  EXPOSTION_TIME -= 1;
  		  };
  	  };

  	  if (BTN1_Left == 1) {
  		  lcd_put_cur(0, 0);
  		  lcd_send_string("<- N. Samples ->");
  		  lcd_put_cur(1, 8);
  		  lcd_send_string(NUMBER_OF_SAMPLES_str);
  		  if (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 1 && BTN3_OK == 1 && NUMBER_OF_SAMPLES != 9999) {
  			  NUMBER_OF_SAMPLES += 1;
  		  };
  		  if (HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin) == 1 && BTN3_OK == 1 && NUMBER_OF_SAMPLES != 0) {
  			  NUMBER_OF_SAMPLES -= 1;
  		  };
  	  };
  	  if (BTN1_Left == 2) {
  		  	  if (BTN3_OK == 0 && FIRE == 0) {
  	  		  	  lcd_put_cur(0, 0);
  	  		  	  lcd_send_string("<-   FIRE ?   ->");
  	  		  	  lcd_put_cur(1, 5);
  	  		  	  lcd_send_string("YES");
  	  		  	  lcd_put_cur(1, 8);
  	  		  	  lcd_send_string("/");
  	  		  	  lcd_put_cur(1, 9);
  	  		  	  lcd_send_string("NO");
  		  	  }
  		  	  if (BTN3_OK == 1) {
  		  		  if (FIRE == 0) {
  		  		  	  lcd_put_cur(0, 0);
  		  		  	  lcd_send_string("<-   FIRE ?   ->");
  	  	  		  	  lcd_put_cur(1, 7);
  	  	  		  	  lcd_send_string("NO");
  		  		  }
  		  		  else if (FIRE == 1) {
  		  		  	  lcd_put_cur(0, 0);
  		  		  	  lcd_send_string("<-   FIRE ?   ->");
  	  	  		  	  lcd_put_cur(1, 6);
  	  	  		  	  lcd_send_string("YES!");
  		  		  }
  	  		  	  if (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 1 && FIRE != 1)  {
  	  		  		  FIRE += 1;
  	  		  	  }
  	  		  	  else if (HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin) == 1 && FIRE != 0) {
  	  		  		  FIRE -= 1;
  	  		  	  }
  		  	  };
		  if (FIRE == 1 && BTN3_OK == 0) {
			  lcd_put_cur(0, 0);
			  lcd_send_string("working....");

		  }
  	  };
  	/* ЭЛЕМЕНТЫ КРУГОВОГО МЕНЮ КОНЕЦ */

  	  HAL_Delay(150);
  	  lcd_clear();
      /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN5_Pin BTN4_Pin BTN3_Pin BTN2_Pin
                           BTN1_Pin */
  GPIO_InitStruct.Pin = BTN5_Pin|BTN4_Pin|BTN3_Pin|BTN2_Pin
                          |BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
