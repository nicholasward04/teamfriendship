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
#include "Motors.h"
#include "Floodfill.h"
#include "IR.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float motor_speed = 0.1; // Speed of motors in terms of duty cycle. Global as to be accessible by button interrupts

unsigned armed = 0; // Arming button assigned to SW1 (false = NOT ARMED true = ARMED)
mouse_dir prev_action = BACKWARD; // Previous action taken by mouse, used for turn distance calculation

unsigned run_count = 0; // Number of completed runs

extern bool moving;
extern bool wall_front;
extern bool wall_left;
extern bool wall_right;

extern float global_angle;
float mouse_angle = 0;

bool initial_wall_L = false;
bool initial_wall_R = false;
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Start timer 2 for PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL); // Start timer 3 and 4 for encoders
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 1);
  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 1);
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, 1);

  Maze maze;

  maze.mouse_pos = (Coord){0, 0};
  maze.mouse_dir = NORTH;
  init_maze(&maze);

  setGoalCell(&maze, 4); // Set initial goal

  set_speed(0, MOTOR_LEFT);
  set_speed(0, MOTOR_RIGHT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 while (!armed) {
		 poll_sensors();
//		 scanWalls(&maze);
	 }

	 if (armed) {
		 scanWalls(&maze);
		 Floodfill(&maze); // call Floodfill to update distance values

		 initial_wall_L = wall_left;
		 initial_wall_R = wall_right;

		 Cell best_cell = bestCell(&maze);
		 Direction dir_of_cell = best_cell.dir;

		 if (dir_of_cell == (Direction)((maze.mouse_dir + 1) % 4)) // Right turn
		 {
			 moveForward(0.1, 114.5754);
			 turnRight(0.1);
			 prev_action = RIGHT;
			 initial_wall_L = wall_front;
			 initial_wall_R = false;
			 mouse_angle -= 90;
			 maze.mouse_dir = (Direction)((maze.mouse_dir + 1) % 4);
		 }
		 else if (dir_of_cell == (Direction)((maze.mouse_dir + 3) % 4)) // Left turn
		 {
			 moveForward(0.1, 114.5754);
			 turnLeft(0.1);
			 prev_action = LEFT;
			 initial_wall_L = false;
			 initial_wall_R = wall_front;
			 global_angle = 0;
			 mouse_angle += 90;
			 maze.mouse_dir = (Direction)((maze.mouse_dir + 3) % 4);
		 }
		 else if (dir_of_cell == (Direction)((maze.mouse_dir + 2) % 4)) // Behind
		 {
			 if (wall_front) {
				 moveForward(0.1, 82.5);
				 turnLeft(0.1);
				 turnLeft(0.1);
				 global_angle = 0;
				 moveBackward(0.1, 113);
				 maze.mouse_dir = (Direction)((maze.mouse_dir + 2) % 4);
				 prev_action = BACKWARD;
			 }
			 else {
				 turnLeft(0.1);
				 turnLeft(0.1);
				 maze.mouse_dir = (Direction)((maze.mouse_dir + 2) % 4);
				 prev_action = NONE;
			 }
			 poll_sensors();
			 initial_wall_L = wall_right;
			 initial_wall_R = wall_left;
			 mouse_angle += 180;
		 }

		 if (prev_action == FORWARD) {
			 moveForward(motor_speed, 183);
		 }
		 else if (prev_action == BACKWARD) {
			 moveForward(0.1, 123.827); // 27.92mm back to mouse rotational center
		 }
		 else if (prev_action == NONE) {

		 }
		 else {
			 poll_sensors();
			 moveForward(motor_speed, 82); // Distance from mouse in center of cell to start of next cell after turn
		 }

		 prev_action = FORWARD;

		 updateMousePos(&maze.mouse_pos, maze.mouse_dir);

		 // Check if mouse is in goal, if so change goal back to start location
		 if (maze.distances[maze.mouse_pos.y][maze.mouse_pos.x] == 0)
		 {
			 if (!((maze.goalPos[0].x == 0) && (maze.goalPos[0].y == 0)))
			 {
				 setGoalCell(&maze, 1); // Change goal cell back to origin
				 // If best direction is behind mouse, go forward one step
				 best_cell = bestCell(&maze);
				 dir_of_cell = best_cell.dir;

				 if (dir_of_cell == ((maze.mouse_dir + 2) % 4)) { // If best cell is behind
					 moveForward(0.1, 183);
					 updateMousePos(&maze.mouse_pos, maze.mouse_dir);
				 }
			 }
			 else
			 {
				 armed = false;
				 run_count++;
				 setGoalCell(&maze, 4); // Change goal cell back to center of maze
				 // Reset mouse position to base position
				 // 180 then back up
				 // Set prev_action as BACKWARD
				 maze.mouse_dir = NORTH;
				 turnLeft(0.1);
				 turnLeft(0.1);
				 moveBackward(0.1, 113);
				 prev_action = BACKWARD;
			 }

		 }
	 }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInit = {0};
  RCC_ClkInitTypeDef RCC_ClkInit = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef ure.
  */
  RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInit.HSEState = RCC_HSE_ON;
  RCC_OscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInit.HSIState = RCC_HSI_ON;
  RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInit.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2047;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_Init = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EMIT_FR_Pin|EMIT_FL_Pin|EMIT_L_Pin|ML_BWD_Pin
                          |MR_BWD_Pin|ML_FWD_Pin|EMIT_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MR_FWD_Pin|LED_Red_Pin|LED_Blue_Pin|LED_Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_Init.Pin = GPIO_PIN_13;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_Init);

  /*Configure GPIO pin : SPEED_SW2_Pin */
  GPIO_Init.Pin = SPEED_SW2_Pin;
  GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
  GPIO_Init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPEED_SW2_GPIO_Port, &GPIO_Init);

  /*Configure GPIO pin : RECIV_R_Pin */
  GPIO_Init.Pin = RECIV_R_Pin;
  GPIO_Init.Mode = GPIO_MODE_INPUT;
  GPIO_Init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RECIV_R_GPIO_Port, &GPIO_Init);

  /*Configure GPIO pin : RECIV_L_Pin */
  GPIO_Init.Pin = RECIV_L_Pin;
  GPIO_Init.Mode = GPIO_MODE_INPUT;
  GPIO_Init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RECIV_L_GPIO_Port, &GPIO_Init);

  /*Configure GPIO pins : EMIT_FR_Pin EMIT_FL_Pin EMIT_L_Pin ML_BWD_Pin
                           MR_BWD_Pin ML_FWD_Pin EMIT_R_Pin */
  GPIO_Init.Pin = EMIT_FR_Pin|EMIT_FL_Pin|EMIT_L_Pin|ML_BWD_Pin
                          |MR_BWD_Pin|ML_FWD_Pin|EMIT_R_Pin;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);

  /*Configure GPIO pins : MR_FWD_Pin LED_Red_Pin LED_Blue_Pin LED_Green_Pin */
  GPIO_Init.Pin = MR_FWD_Pin|LED_Red_Pin|LED_Blue_Pin|LED_Green_Pin;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);

  /*Configure GPIO pin : ARM_SW1_Pin */
  GPIO_Init.Pin = ARM_SW1_Pin;
  GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
  GPIO_Init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARM_SW1_GPIO_Port, &GPIO_Init);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ARM_SW1_Pin) { // Arm mouse to start run after 10 second delay
		armed = 1;
		unsigned LED = 0;
		for (int x = 0; x < 10; x++) {
			HAL_Delay(250);
			LED = (LED + 1) % 2;
			HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, LED);

		}
	}
	else if (GPIO_Pin == SPEED_SW2_Pin) { // Increase motor speed
		unsigned LED_2 = 0;
		if (motor_speed < 0.15) {
			motor_speed = 0.2; // 0.2
			LED_2 = 0;
		}
		else {
			motor_speed = 0.1;
			LED_2 = 1;
		}
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, LED_2); // Red LED ON when on high speed, OFF when on low speed
		HAL_Delay(500);
	}
	else {
		return;
	}
}
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
