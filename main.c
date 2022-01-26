/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "float.h"
#include "../../Library/lcd.h"
#include "roshandler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FR_ChanA HAL_GPIO_ReadPin(FR_CHA_GPIO_Port,FR_CHA_Pin)
#define FR_ChanB HAL_GPIO_ReadPin(FR_CHB_GPIO_Port,FR_CHB_Pin)

#define FL_ChanA HAL_GPIO_ReadPin(FL_CHA_GPIO_Port,FL_CHA_Pin)
#define FL_ChanB HAL_GPIO_ReadPin(FL_CHB_GPIO_Port,FL_CHB_Pin)

#define BR_ChanA HAL_GPIO_ReadPin(BR_CHA_GPIO_Port,BR_CHA_Pin)
#define BR_ChanB HAL_GPIO_ReadPin(BR_CHB_GPIO_Port,BR_CHB_Pin)

#define BL_ChanA HAL_GPIO_ReadPin(BL_CHA_GPIO_Port,BL_CHA_Pin)
#define BL_ChanB HAL_GPIO_ReadPin(BL_CHB_GPIO_Port,BL_CHB_Pin)

#define FR_ChanA_High (FR_ChanA==1)
#define FR_ChanA_Low (FR_ChanA==0)

#define FR_ChanB_High (FR_ChanB==1)
#define FR_ChanB_Low (FR_ChanB==0)

#define FL_ChanA_High (FL_ChanA==1)
#define FL_ChanA_Low (FL_ChanA==0)

#define FL_ChanB_High (FL_ChanB==1)
#define FL_ChanB_Low (FL_ChanB==0)

#define BR_ChanA_High (BR_ChanA==1)
#define BR_ChanA_Low (BR_ChanA==0)

#define BR_ChanB_High (BR_ChanB==1)
#define BR_ChanB_Low (BR_ChanB==0)

#define BL_ChanA_High (BL_ChanA==1)
#define BL_ChanA_Low (BL_ChanA==0)

#define BL_ChanB_High (BL_ChanB==1)
#define BL_ChanB_Low (BL_ChanB==0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId motor_taskHandle;
/* USER CODE BEGIN PV */
Lcd_HandleTypeDef lcd;
Lcd_PortType port[] = { d4_GPIO_Port, d5_GPIO_Port, d6_GPIO_Port, d7_GPIO_Port };
Lcd_PinType pin[] = { d4_Pin, d5_Pin, d6_Pin, d7_Pin };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
void motor_thread(void const * argument);

/* USER CODE BEGIN PFP */
void transmit_motor(int flpwm, int frpwm, int blpwm, int brpwm);
void init_motor(void);
void menu_motor(void);
void menu_encoder(void);
void read_encoder(int pos);
void init_interupt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool play = false;
char buff[33];
int pos;
int row, col;
int pos_motor = 0;
int row_motor, col_motor;
int pos_encoder;
uint32_t dir_enc[4 * 2];
// Kp Motor 1 = 1.
// Kp Motor 2 = 1.025
// Kp Motor 3 = 1.35
// Kp Motor 4 = 0.6

float ppr[] = { 434., 434., 434., 434. };
float pid[4][3] = { { 1.0, 0.001, 2.0 }, { 1.0, 0.001, 0.0 },
		{ 1.0, 0.02, 0.0 }, { 1.0, 0.001, 1.57 } };
float error[4] = { 0., 0., 0., 0. };

float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
float error_kalkulasi[4] = { 0.0, 0.0, 0.0, 0.0 };
float delta_error[4] = { 0.0, 0.0, 0.0, 0.0 };
float last_error[4] = { 0.0, 0.0, 0.0, 0.0 };
float sigma_error[4] = { 0.0, 0.0, 0.0, 0.0 };
float ts = 0.1;
float max_output = 1024.0;
float min_output = 1024.0;
float last_setpoint[4] = { 0.0, 0.0, 0.0, 0.0 };
float last_kp[4] = { 0.0, 0.0, 0.0, 0.0 };
int count = 0;
int inc = 0;

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of motor_task */
  osThreadDef(motor_task, motor_thread, osPriorityNormal, 0, 512);
  motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 10;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1024;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4200-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ena_Pin|rs_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, d4_Pin|d5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, d6_Pin|d7_Pin|anoda_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN4_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ena_Pin rs_Pin */
  GPIO_InitStruct.Pin = ena_Pin|rs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN2_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BR_CHB_Pin BR_CHA_Pin BL_CHB_Pin */
  GPIO_InitStruct.Pin = BR_CHB_Pin|BR_CHA_Pin|BL_CHB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BL_CHA_Pin */
  GPIO_InitStruct.Pin = BL_CHA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BL_CHA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FR_CHA_Pin FR_CHB_Pin FL_CHA_Pin FL_CHB_Pin */
  GPIO_InitStruct.Pin = FR_CHA_Pin|FR_CHB_Pin|FL_CHA_Pin|FL_CHB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN3_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN3_Pin|BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : d4_Pin d5_Pin */
  GPIO_InitStruct.Pin = d4_Pin|d5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : d6_Pin d7_Pin anoda_Pin */
  GPIO_InitStruct.Pin = d6_Pin|d7_Pin|anoda_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Clock Wise

	// FL ....
	if (GPIO_Pin == FL_CHA_Pin) {
		if ((FL_ChanA_High && FL_ChanB_Low)
				|| (FL_ChanA_Low && FL_ChanB_High)) {
			dir_enc[0]++;
		} else if ((FL_ChanA_High && FL_ChanB_High)
				|| (FL_ChanA_Low && FL_ChanB_Low)) {
			dir_enc[1]++;
		}
	}

	if (GPIO_Pin == FL_CHB_Pin) {
		if ((FL_ChanA_High && FL_ChanB_High)
				|| (FL_ChanA_Low && FL_ChanB_Low)) {
			dir_enc[0]++;
		} else if ((FL_ChanA_High && FL_ChanB_Low)
				|| (FL_ChanA_Low && FL_ChanB_High)) {
			dir_enc[1]++;
		}
	}

	// FR ....
	if (GPIO_Pin == FR_CHA_Pin) {
		if ((FR_ChanA_High && FR_ChanB_Low)
				|| (FR_ChanA_Low && FR_ChanB_High)) {
			dir_enc[2]++;
		} else if ((FR_ChanA_High && FR_ChanB_High)
				|| (FR_ChanA_Low && FR_ChanB_Low)) {
			dir_enc[3]++;
		}
	}

	if (GPIO_Pin == FR_CHB_Pin) {
		if ((FR_ChanA_High && FR_ChanB_High)
				|| (FR_ChanA_Low && FR_ChanB_Low)) {
			dir_enc[2]++;
		} else if ((FR_ChanA_High && FR_ChanB_Low)
				|| (FR_ChanA_Low && FR_ChanB_High)) {
			dir_enc[3]++;
		}
	}

	// BL ....
	if (GPIO_Pin == BL_CHA_Pin) {
		if ((BL_ChanA_High && BL_ChanB_Low)
				|| (BL_ChanA_Low && BL_ChanB_High)) {
			dir_enc[4]++;
		} else if ((FR_ChanA_High && FR_ChanB_High)
				|| (BL_ChanA_Low && BL_ChanB_Low)) {
			dir_enc[5]++;
		}
	}

	if (GPIO_Pin == BL_CHB_Pin) {
		if ((BL_ChanA_High && BL_ChanB_High)
				|| (BL_ChanA_Low && BL_ChanB_Low)) {
			dir_enc[4]++;
		} else if ((BL_ChanA_High && BL_ChanB_Low)
				|| (BL_ChanA_Low && BL_ChanB_High)) {
			dir_enc[5]++;
		}
	}

	// BR ....
	if (GPIO_Pin == BR_CHA_Pin) {
		if ((BR_ChanA_High && BR_ChanB_Low)
				|| (BR_ChanA_Low && BR_ChanB_High)) {
			dir_enc[6]++;
		} else if ((BR_ChanA_High && BR_ChanB_High)
				|| (BR_ChanA_Low && BR_ChanB_Low)) {
			dir_enc[7]++;
		}
	}

	if (GPIO_Pin == BR_CHB_Pin) {
		if ((BR_ChanA_High && BR_ChanB_High)
				|| (BR_ChanA_Low && BR_ChanB_Low)) {
			dir_enc[6]++;
		} else if ((BR_ChanA_High && BR_ChanB_Low)
				|| (BR_ChanA_Low && BR_ChanB_High)) {
			dir_enc[7]++;
		}
	}

}

void init_motor() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

void init_interupt(void) {
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim14);
}

void read_encoder(int pos) {
	while (1) {
		if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 0) {
			};
			Lcd_clear(&lcd);
			break;
		}

		char buff[33];

		switch (pos) {
		case 0:
			Lcd_cursor(&lcd, 0, 0);
			Lcd_string(&lcd, "FR:");
			sprintf(buff, "%.3f   ", rpm[1]);
			Lcd_string(&lcd, buff);
			Lcd_cursor(&lcd, 1, 0);
			Lcd_string(&lcd, "BR:");
			sprintf(buff, "%.3f   ", rpm[3]);
			Lcd_string(&lcd, buff);
			break;

		case 1:
			Lcd_cursor(&lcd, 0, 0);
			Lcd_string(&lcd, "FL:");
			sprintf(buff, "%.3f   ", rpm[0]);
			Lcd_string(&lcd, buff);
			Lcd_cursor(&lcd, 1, 0);
			Lcd_string(&lcd, "BL:");
			sprintf(buff, "%.3f   ", rpm[2]);
			Lcd_string(&lcd, buff);
			break;
		}
	}
}

void menu_encoder(void) {
	while (play) {
		Lcd_cursor(&lcd, pos_encoder, 0);
		Lcd_string(&lcd, ">");
		Lcd_cursor(&lcd, 0, 1);
		Lcd_string(&lcd, "RPM. Right");
		Lcd_cursor(&lcd, 1, 1);
		Lcd_string(&lcd, "RPM. Left");

		if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 0) {
			};
			Lcd_clear(&lcd);
			play = false;
		} else if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0) {
			};
			Lcd_clear(&lcd);
			if (++pos_encoder > 1) {
				pos_encoder = 0;
			}
		}
		if (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 0) {
			};
			Lcd_clear(&lcd);
			read_encoder(pos_encoder);
		}
	}
}

void transmit_motor(int flpwm, int frpwm, int blpwm, int brpwm) {
	float pwm[] = { abs(flpwm), abs(frpwm), abs(blpwm), abs(brpwm) };

	// FL
	if (flpwm > 0) {
		TIM2->CCR4 = pwm[0];
		TIM3->CCR4 = 0;
	} else if (flpwm < 0) {
		TIM2->CCR4 = 0;
		TIM3->CCR4 = pwm[0];
	} else {
		TIM2->CCR4 = 0;
		TIM3->CCR4 = 0;
	}

	// FR
	if (frpwm > 0) {
		TIM8->CCR2 = pwm[1];
		TIM8->CCR1 = 0;
	} else if (frpwm < 0) {
		TIM8->CCR2 = 0;
		TIM8->CCR1 = pwm[1];
	} else {
		TIM8->CCR1 = 0;
		TIM8->CCR2 = 0;
	}

	// BL
	if (blpwm > 0) {
		TIM3->CCR1 = 0;
		TIM3->CCR2 = pwm[2];
	} else if (blpwm < 0) {
		TIM3->CCR1 = pwm[2];
		TIM3->CCR2 = 0;
	} else {
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
	}

	// BR
	if (brpwm > 0) {
		TIM2->CCR2 = 0;
		TIM3->CCR3 = pwm[3];
	} else if (brpwm < 0) {
		TIM2->CCR2 = pwm[3];
		TIM3->CCR3 = 0;
	} else {
		TIM2->CCR2 = 0;
		TIM3->CCR3 = 0;
	}

}

void menu_motor(void) {
	bool cond = false;
	while (1) {
		if (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 0) {
			};
			Lcd_clear(&lcd);
			cond = !cond;
			HAL_GPIO_WritePin(anoda_GPIO_Port, anoda_Pin, (int) !cond);
			if (cond) {
				Lcd_cursor(&lcd, 0, 0);
				Lcd_string(&lcd, "Play Robot....");
			} else {
				for (int i = 0; i < 4; i++) {
					setpoint[i] = 0;
				}
				pos = 0;
				play = false;
				break;
			}
		}

		switch (cond) {
		case false:
			Lcd_cursor(&lcd, row_motor, col_motor);
			Lcd_string(&lcd, ">");
			Lcd_cursor(&lcd, 0, 1);
			Lcd_string(&lcd, "FL:");
			Lcd_int(&lcd, setpoint[0]);
			Lcd_cursor(&lcd, 1, 1);
			Lcd_string(&lcd, "FR:");
			Lcd_int(&lcd, setpoint[1]);

			Lcd_cursor(&lcd, 0, 9);
			Lcd_string(&lcd, "BL:");
			Lcd_int(&lcd, setpoint[2]);
			Lcd_cursor(&lcd, 1, 9);
			Lcd_string(&lcd, "BR:");
			Lcd_int(&lcd, setpoint[3]);

			if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 0) {
				while (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == 0) {
				};
				Lcd_clear(&lcd);
				if (setpoint[pos_motor] <= -1000) {
					setpoint[pos_motor] = -1000;
				}
				setpoint[pos_motor] -= 50;
			} else if (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == 0) {
				while (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == 0) {
				};
				Lcd_clear(&lcd);
				if (setpoint[pos_motor] >= 1000) {
					setpoint[pos_motor] = 1000;
				}
				setpoint[pos_motor] += 50;
			} else if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0) {
				while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0) {
				};
				Lcd_clear(&lcd);
				if (++pos_motor > 3) {
					pos_motor = 0;
				}
				if (pos_motor == 2) {
					row_motor = 0;
					col_motor = 8;
				} else if (pos_motor == 3) {
					row_motor = 1;
					col_motor = 8;
				} else {
					row_motor = pos_motor;
					col_motor = 0;
				}
			}
			break;
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_motor_thread */
/**
 * @brief  Function implementing the motor_task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_motor_thread */
void motor_thread(void const * argument)
{
  /* USER CODE BEGIN 5 */
	setup();
	init_motor();
	init_interupt();
	lcd = Lcd_create(port, pin, rs_GPIO_Port, rs_Pin, ena_GPIO_Port,
	ena_Pin, LCD_4_BIT_MODE);
	HAL_GPIO_WritePin(anoda_GPIO_Port, anoda_Pin, (int) !play);
	Lcd_cursor(&lcd, 0, 2);
	Lcd_string(&lcd, "SKRIPSI 2021");
	Lcd_cursor(&lcd, 1, 2);
	Lcd_string(&lcd, "IVAN FADHILA");
	HAL_Delay(2000);
	Lcd_clear(&lcd);
	/* Infinite loop */
	while (1) {
		if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0) {
			};
			if (play == false) {
				Lcd_clear(&lcd);
				if (++pos > 2) {
					pos = 0;
				}
				if (pos == 2) {
					row = 0;
					col = 9;
				} else {
					row = pos;
					col = 0;
				}
			}
		} else if (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 0) {
			while (HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == 0) {
			};
			Lcd_clear(&lcd);
			play = !play;
			if (pos == 0) {
				menu_motor();
			} else if (pos == 1) {
				menu_encoder();
			} else if (pos == 2) {
				if (play) {
					Lcd_cursor(&lcd, 0, 0);
					Lcd_string(&lcd, "Play Robot....");
				}
				HAL_GPIO_WritePin(anoda_GPIO_Port, anoda_Pin, (int) !play);
			}
		}

		switch (play) {
		case false:
			Lcd_cursor(&lcd, row, col);
			Lcd_string(&lcd, ">");
			Lcd_cursor(&lcd, 0, 1);
			Lcd_string(&lcd, "Motor");
			Lcd_cursor(&lcd, 1, 1);
			Lcd_string(&lcd, "RPM");
			Lcd_cursor(&lcd, 0, 10);
			Lcd_string(&lcd, "Play");
			break;
		}
		loop();
	}
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM14) {
		for (int i = 0; i < 4; i++) {
			if (last_setpoint[i] != setpoint[i]) {
				if (setpoint[i] < 0 && last_setpoint[i] > 0) {
					transmit_motor(0, 0, 0, 0);
					error_kalkulasi[i] = 0.0;
					delta_error[i] = 0.0;
					last_error[i] = 0.0;
					sigma_error[i] = 0.0;
					output[i] = 0.0;
//					last_kp[i] = pid[i][1];
//					pid[i][1] = 0.0;
//					count = 1;
				} else if (setpoint[i] > 0 && last_setpoint < 0) {
					transmit_motor(0, 0, 0, 0);
					error_kalkulasi[i] = 0.0;
					delta_error[i] = 0.0;
					last_error[i] = 0.0;
					sigma_error[i] = 0.0;
					output[i] = 0.0;
//					last_kp[i] = pid[i][1];
//					pid[i][1] = 0.0;
//					count = 1;
				}
			}
		}
//				int index = 3;
//		if (setpoint[index] > 0) {
//			if (change == 0) {
//				setpoint[index] = 0;
//				change = 1;
//				output[index] = 0;
//				delta_error[index] = 0;
//				error_kalkulasi[index] = 0;
//				sigma_error[index] = 0;
//				last_error[index] = 0;
//				inc += 1;
//			}
//		} else if (setpoint[index] < 0) {
//			if (change == 1) {
//				change = 0;
//				setpoint[index] = 0;
//				output[index] = 0;
//				delta_error[index] = 0;
//				error_kalkulasi[index] = 0;
//				sigma_error[index] = 0;
//				last_error[index] = 0;
//				inc += 1;
//			}
//		}

		for (int i = 0; i < 4; i++) {
			last_setpoint[i] = setpoint[i];
			output[i] = 0;
			error_kalkulasi[i] = setpoint[i] - rpm[i];
			delta_error[i] = last_error[i] - error_kalkulasi[i];
			sigma_error[i] = sigma_error[i] + error_kalkulasi[i];
			output[i] = (pid[i][0] * error_kalkulasi[i])
					+ (sigma_error[i] * pid[i][1] / ts)
					+ ((delta_error[i] * pid[i][2]) * ts);
			last_error[i] = error_kalkulasi[i];
			if (output[i] > max_output)
				output[i] = max_output;
			else if (output[i] < min_output)
				output[i] = min_output;
			if (setpoint[i] == 0) {
				output[i] = 0;
			} else {
				output[i] = (kp) + (ki * pid[i][1]);
			}
		}
		if (count == 1) {
			inc++;
			if (inc == 10) {
				inc = 0;
				count = 0;
				for (int i = 0; i < 4; i++) {
					pid[i][1] = last_kp[i];
				}
			}
		}
	}
	if (htim->Instance == TIM7) {
		int oddIndex = 1;
		int evenIndex = 0;
		for (int i = 0; i < 4; i++) {
			if (dir_enc[evenIndex] > dir_enc[oddIndex]) {
				rpm[i] = (((dir_enc[evenIndex] * 60) / ppr[i]) / 0.1);
			} else if (dir_enc[evenIndex] < dir_enc[oddIndex]) {
				rpm[i] = (((dir_enc[oddIndex] * 60) / ppr[i]) / 0.1) * -1;
			} else {
				rpm[i] = 0;
			}

			dir_enc[oddIndex] = 0;
			dir_enc[evenIndex] = 0;
			evenIndex += 2;
			oddIndex += 2;
		}
		rpm_publisher();
		oddIndex = 1;
		evenIndex = 0;
		// Gj	1:3	3:5 5:7	7: --9
		// Gn	0:2	2:4	4:6	6: --8
		// 		0 	1 	2 	3
	}
	linearMotor(setpoint);
	if (play) {
//		transmit_motor(setpoint[0], setpoint[1], setpoint[2], setpoint[3]);
		transmit_motor(direction[0], direction[1], direction[2], direction[3]);
	} else {
		transmit_motor(0, 0, 0, 0);
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
