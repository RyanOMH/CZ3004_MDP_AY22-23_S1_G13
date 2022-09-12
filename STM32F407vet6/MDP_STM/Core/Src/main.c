/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "PID.h"
#include "Q.h"
#include <stdio.h>
#include <stdlib.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ShowText */
osThreadId_t ShowTextHandle;
const osThreadAttr_t ShowText_attributes = {
  .name = "ShowText",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void Motor(void *argument);
void encoder_task(void *argument);
void gyro_task(void *argument);
void PID(int var, int read, int goal);

/* USER CODE BEGIN PFP */
void forward_motor_prep();
void backward_motor_prep();
void servomotor_center();
void servomotor_left();
void servomotor_right();
void move(float distance , int frontorback, int leftorright);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void send_UART(char*Tx_str);
void process_UART_Rx();
void gyroInit();
void writeByte(uint8_t addr,uint8_t data);
void readByte(uint8_t addr, uint8_t* data);
void state_controller(Queue *q);
void right_turn(int angle);
void left_turn(int angle);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const int BUFFER_SIZE = 20;
uint8_t aRxBuffer[20]; //Buffer of 20 bytes
uint8_t ICM_ADDR = 0x68;
uint8_t buff[20]; //Gyroscope buffer
double total_angle = 0;
double turning_angle = 0;
uint8_t OLED_Row_0[20],OLED_Row_1[20],OLED_Row_2[20],OLED_Row_3[20],OLED_Row_4[20],OLED_Row_5[20];
int pwm_L_f, pwm_L_b;
int pwm_R_f, pwm_R_b;
int servo_center, servo_left, servo_right;
int motor_dir; //Backward = -1, Stop = 0, Forward = 1
int servo_dir; //Left = -1, Center = 0, Right = 1
//int angle_dir;
int left_speed, right_speed;
float leftwheel_dist = 0;
float rightwheel_dist = 0;
Queue command;
int RX_FLAG;
char RX_MOTOR;
char RX_SERVO;
int RX_DIST;
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,3); //Receive 3 bytes
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ShowText */
  ShowTextHandle = osThreadNew(show, NULL, &ShowText_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(Motor, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of gyroTask */
  gyroTaskHandle = osThreadNew(gyro_task, NULL, &gyroTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED_3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void forward_motor_prep()
{
	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
}

void backward_motor_prep()
{
	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
}

void servomotor_center()
{
	// default: 150
	uint32_t value = 150;

	if (htim1.Instance->CCR4 == value){
		return;
	}
	htim1.Instance->CCR4 = value;
	osDelay(500);
}

void servomotor_left()
{
	// default: 110
	uint32_t value = 100;
	if (htim1.Instance->CCR4 == value){
			return;
	}
	htim1.Instance->CCR4 = value;
	osDelay(500);
}

void servomotor_right()
{
	// default: 210
	uint32_t value = 210;
	if (htim1.Instance->CCR4 == value){
			return;
	}
	htim1.Instance->CCR4 = value;
	osDelay(500);
}

void move(float distance, int frontorback , int leftorright)
{
	//default PWM values
	int pwmval_L = 1500;
	int pwmval_R = 1500;
	int flag = 1;

	//distance of the left & right wheel travelled
	float leftwheel_dist = 0;
	float rightwheel_dist = 0;
	float temp_leftwheel_dist = 0;
	float temp_rightwheel_dist = 0;

	//number of ticks for one full rotation of wheel
	float full_rotation_wheel = 1320;
	float circumference_wheel = 20.4f;

	//store the ticks from each encoder
	int left_encoder_prev, right_encoder_prev, left_encoder, right_encoder, diff_left, diff_right;

	//used to determine the rate at which we make adjustments
	uint32_t start_time , prev_time, curr_time;

	//Decide the correct motion based on the input parameters
	if (frontorback){//front
		forward_motor_prep();
	}
	else {
		backward_motor_prep();
	}

	if(leftorright == 1){//left
		servomotor_left();
	}
	else if(leftorright == 0){
		servomotor_right();
	}
	else {
		servomotor_center();
	}

	left_encoder_prev = __HAL_TIM_GET_COUNTER(&htim2);
	right_encoder_prev = __HAL_TIM_GET_COUNTER(&htim3);
	start_time = HAL_GetTick();
	prev_time = start_time;

	//Start the motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmval_L);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmval_R);

	uint8_t hello[20], hello1[20];
	//run until my duration set is up
	while (flag){
		if(HAL_GetTick() - start_time > 1000L){
			left_encoder = __HAL_TIM_GET_COUNTER(&htim2);
			right_encoder = __HAL_TIM_GET_COUNTER(&htim3);

			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
			{
				if(left_encoder < left_encoder_prev)
				{
					diff_left = left_encoder_prev - left_encoder;
				}
				else
				{
					diff_left = (65535 - left_encoder) + left_encoder_prev;
				}
			}
			else
			{
				if(left_encoder > left_encoder_prev)
				{
					diff_left = left_encoder - left_encoder_prev;
				}
				else
				{
					diff_left = (65535 - left_encoder_prev);
				}
			}
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			{
				if(right_encoder < right_encoder_prev)
				{
					diff_right = right_encoder_prev - right_encoder;
				}
				else
				{
					diff_right = (65535 - right_encoder) + right_encoder_prev;
				}
			}
			else
			{
				if(right_encoder > right_encoder_prev)
				{
					diff_right = right_encoder - right_encoder_prev;
				}
				else
				{
					diff_right = (65535 - right_encoder_prev);
				}
			}
//			sprintf(hello,"SpeedL:%5d\0",diff_left);
//			OLED_ShowString(10,20,hello);
//			sprintf(hello1,"SpeedR:%5d\0",diff_right);
//			OLED_ShowString(10,30,hello1);
			left_encoder_prev = __HAL_TIM_GET_COUNTER(&htim2);
			right_encoder_prev = __HAL_TIM_GET_COUNTER(&htim3);
			start_time = HAL_GetTick(); //tick value in milliseconds

			//Calculate the distance covered and compare with distance we input
			temp_leftwheel_dist = ((float)diff_left/full_rotation_wheel) * circumference_wheel;
			leftwheel_dist += temp_leftwheel_dist;

			temp_rightwheel_dist = ((float)diff_right/full_rotation_wheel) * circumference_wheel;
			rightwheel_dist += temp_rightwheel_dist;

			if (leftwheel_dist >= distance || rightwheel_dist >= distance){ //stop condition
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
				flag = 0;
			}
		}
	}
	HAL_Delay(100);
}

void right_turn(int angle)
{
	//Prep the servomotor to right
	servomotor_right();

	osDelay(250);

	turning_angle = 0;
	forward_motor_prep();
	//Start the motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);

	while (turning_angle > -1*angle){ //while it is still turning to the correct angle
		osDelay(10);
	}


	//Stop the Motor are completing the turn
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	servomotor_center();

}

void left_turn(int angle)
{
	//Prep the servomotor to left
	servomotor_left();

	osDelay(250);

	turning_angle = 0;
	forward_motor_prep();
	//Start the motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);

	while (turning_angle < angle){ //while it is still turning to the correct angle
		osDelay(10);
	}


	//Stop the Motor are completing the turn
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	servomotor_center();

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*Prevent unused argument(s) compilation warning*/
	UNUSED(huart);
	HAL_UART_Transmit(&huart3,(uint8_t *)aRxBuffer,3,0xFFFF);
	if(RX_FLAG == 0){
		RX_MOTOR = (char) aRxBuffer[0];
		RX_SERVO = (char) aRxBuffer[1];
		RX_DIST = (int) aRxBuffer[2];
		RX_FLAG = 1;
	}
	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,3);
}

void send_UART(char*Tx_str)
{
	for(int ch=0; ch<sizeof(Tx_str); ch++){
		char data = Tx_str[ch];
		HAL_UART_Transmit(&huart3,(uint8_t *)data,1,0xFFFF);
	}
}

void process_UART_Rx()
{
	int i;

	if (RX_FLAG == 1){
		//Control motor direction
		if (RX_MOTOR == 'F'){
			if (motor_dir != 1){
				forward_motor_prep();
				motor_dir = 1;
			}
		}
		else if (RX_MOTOR == 'B'){
			if (motor_dir != -1){
				backward_motor_prep();
				motor_dir = -1;
			}
		}
		else {
			if (motor_dir != 0){
				forward_motor_prep();
				motor_dir = 0;
			}
		}
		//Control servo direction
		if (RX_SERVO == 'C'){
			if (servo_dir != 0){
				servomotor_center();
				servo_dir = 0;
			}
		}
		else if (RX_SERVO == 'L'){
			if (servo_dir != -1){
				servomotor_left();
				servo_dir = -1;
			}
		}
		else if (RX_SERVO == 'R'){
			if (servo_dir != 1){
				servomotor_right();
				servo_dir = 1;
			}
		}
		else {
			if (servo_dir != 0){
				servomotor_center();
				servo_dir = 0;
			}
		}
//		for(i=0;i<BUFFER_SIZE;i++)
//		{
//			aRxBuffer[i] = '\0';
//		}
		RX_FLAG = 0;
	}

}

void state_controller (Queue *q) {
	Cmd cur_command;
	cur_command = q->ll.head->item;

	if (leftwheel_dist >= cur_command.dist || rightwheel_dist >= cur_command.dist){

	}
}

void gyroInit()
{
  writeByte(0x06, 0x00);
  osDelayUntil(10);
  writeByte(0x03, 0x80);
  osDelayUntil(10);
  writeByte(0x07, 0x3F);
  osDelayUntil(10);
  writeByte(0x06, 0x01);
  osDelayUntil(10);
  writeByte(0x7F, 0x20); // go to bank 2
  osDelayUntil(10);
  writeByte(0x01, 0x2F); // config gyro, enable gyro, dlpf, set gyro to +-2000dps; gyro lpf = 3'b101
  osDelayUntil(10);
  writeByte(0x00, 0x00); // set gyro sample rate divider = 1 + 0(GYRO_SMPLRT_DIV[7:0])
  osDelayUntil(10);
  writeByte(0x01, 0x2F); // config accel, enable gyro, dlpf, set gyro to +-2000dps; gyro lpf = 3'b101
  osDelayUntil(10);
  writeByte(0x00, 0x00); // set gyro sample rate divider = 1 + 0(GYRO_SMPLRT_DIV[7:0])
  osDelayUntil(10);
  writeByte(0x7F, 0x00); // return to bank 1
  osDelayUntil(10);
  writeByte(0x07, 0x00);
  osDelayUntil(10);
}

void writeByte(uint8_t addr, uint8_t data)
{
  buff[0] = addr;
  buff[1] = data;
  HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR<<1, buff, 2, 20);
}

void readByte(uint8_t addr, uint8_t *data)
{
  buff[0] = addr;
  // Tell we want to read from the register
  HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR<<1, buff, 1, 10);
  // Read 2 byte from z dir register
  HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR<<1, data, 2, 20);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOE, LED_3_Pin);
	//process_UART_Rx();
	osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the ShowText thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  /* Infinite loop */
	for (;;){
		OLED_ShowString(10,0,OLED_Row_0);
		OLED_ShowString(10,10,OLED_Row_1);
		OLED_ShowString(10,20,OLED_Row_2);
		OLED_ShowString(10,30,OLED_Row_3);
		OLED_ShowString(10,40,OLED_Row_4);
		OLED_ShowString(10,50,OLED_Row_5);
		OLED_Refresh_Gram();
	}
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_Motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor */
void Motor(void *argument)
{
  /* USER CODE BEGIN Motor */
	//start generating PWN signal for me
	HAL_TIM_PWM_Start(&htim8 , TIM_CHANNEL_1); // MotorA
	HAL_TIM_PWM_Start(&htim8 , TIM_CHANNEL_2); // MotorB
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Servo Motor
	motor_dir = 1; servo_dir = 0;
	forward_motor_prep();
	servomotor_center();

	pwm_L_f = 500;
	pwm_L_b = 500;
	pwm_R_f = 500;
	pwm_R_b = 500;

	left_turn(90);
	osDelay(500);
	//right_turn(130);


	struct PIDController motor_LF_PID, motor_RF_PID, motor_LB_PID, motor_RB_PID;

	MotorPIDController_Init(&motor_LF_PID);
	MotorPIDController_Init(&motor_RF_PID);
	MotorPIDController_Init(&motor_LB_PID);
	MotorPIDController_Init(&motor_RB_PID);
	//Make sure the Servo motor position is in middle position
	servomotor_center();
	/* Infinite loop */
	uint32_t tick = HAL_GetTick();
	for(;;)
	{	if (HAL_GetTick() - tick > 100L){
			if (motor_dir == 1){
				//Start the motor
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_L_f);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_R_f);
				sprintf(OLED_Row_0, "FRWD\0");
//				sprintf(OLED_Row_1, "PWML:%5d\0", pwm_L_f);
//				sprintf(OLED_Row_2, "PWMR:%5d\0", pwm_R_f);
				//ADD PID CONTROL
				pwm_L_f = MotorPIDController_Update(&motor_LF_PID, left_speed, 2000, pwm_L_f);
				pwm_R_f = MotorPIDController_Update(&motor_RF_PID, right_speed, 2000, pwm_R_f);
			}
			else if (motor_dir == -1){
				//Start the motor
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_L_b);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_R_b);
				sprintf(OLED_Row_0,"BKWD\0");
//				sprintf(OLED_Row_1, "PWML:%5d\0", pwm_L_b);
//				sprintf(OLED_Row_2, "PWMR:%5d\0", pwm_R_b);
				//ADD PID CONTROL
				pwm_L_b = MotorPIDController_Update(&motor_LB_PID, left_speed, 2000, pwm_L_b);
				pwm_R_b = MotorPIDController_Update(&motor_RB_PID, right_speed, 2000, pwm_R_b);
			}
			else {
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
				sprintf(OLED_Row_0,"STOP\0");
//				sprintf(OLED_Row_1, "PWML:%5d\0", 0);
//				sprintf(OLED_Row_2, "PWMR:%5d\0", 0);
			}
			tick = HAL_GetTick();
		}
		osDelay(10);
	}
  /* USER CODE END Motor */
}

/* USER CODE BEGIN Header_encoder_task */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_task */
void encoder_task(void *argument)
{
  /* USER CODE BEGIN encoder_task */
  /* Infinite loop */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL); //activate the encoder for Motor A
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); //activate the encoder for Motor B
  int left_prev, left_curr, left_diff, right_prev, right_curr, right_diff;
  int dir_L, dir_R;
  int distInt_L = 0, distInt_R = 0;

  float temp_leftwheel_dist = 0;
  float temp_rightwheel_dist = 0;

  uint32_t tick, cur_tick, T;
  //number of ticks for one full rotation of wheel
  float full_rotation_wheel = 1550;
  float circumference_wheel = 21.3f;

  left_prev = __HAL_TIM_GET_COUNTER(&htim2);
  right_prev = __HAL_TIM_GET_COUNTER(&htim3);
  tick = HAL_GetTick(); //tick value in milliseconds

  for(;;)
  {
	  cur_tick = HAL_GetTick();
	  if (cur_tick - tick > 100L){ //every 0.1 second
		  left_curr = __HAL_TIM_GET_COUNTER(&htim2);
		  right_curr = __HAL_TIM_GET_COUNTER(&htim3);
		  //Left encoder
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			  if(left_curr <= left_prev){
				  left_diff = left_prev - left_curr;
			  }
			  else {
				  left_diff = (65535 - left_curr) + left_prev; //handle overflow situation
			  }
			  dir_L = 1;
		  }
		  else {
			  if(left_curr >= left_prev){
				  left_diff = left_curr - left_prev;
			  }
			  else {
				  left_diff = (65535 - left_prev) + left_curr;
			  }
			  dir_L = -1;
		  }
		  //Right encoder
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			  if(right_curr <= right_prev){
				  right_diff = right_prev - right_curr;
			  }
			  else {
				  right_diff = (65535 - right_curr) + right_prev; //handle overflow situation
			  }
			  dir_R = -1;
		  }
		  else {
			  if(right_curr >= right_prev){
				  right_diff = right_curr - right_prev;
			  }
			  else {
				  right_diff = (65535 - right_prev) + right_curr;
			  }
			  dir_R = 1;
		  }
		  T = cur_tick - tick;
		  //Show speed in ticks/s
		  left_speed = left_diff * (1000/T);
		  right_speed = right_diff * (1000/T);
		  sprintf(OLED_Row_3,"SPD L: %6d\0",left_speed);
		  sprintf(OLED_Row_4,"SPD R: %6d\0",right_speed);

		  //Calculate distance traveled
//		  temp_leftwheel_dist = ((float)left_diff/full_rotation_wheel) * circumference_wheel * dir_L;
//		  leftwheel_dist += temp_leftwheel_dist;
//		  temp_rightwheel_dist = ((float)right_diff/full_rotation_wheel) * circumference_wheel * dir_R;
//		  rightwheel_dist += temp_rightwheel_dist;
//		  distInt_L = (int)leftwheel_dist;
//		  distInt_R = (int)rightwheel_dist;
		  distInt_L += left_diff * dir_L;
		  distInt_R += right_diff * dir_R;
		  sprintf(OLED_Row_1, "DST L: %6d\0", distInt_L);
		  sprintf(OLED_Row_2, "DST R: %6d\0", distInt_R);

		  //Reset counters
		  left_prev = __HAL_TIM_GET_COUNTER(&htim2);
		  right_prev = __HAL_TIM_GET_COUNTER(&htim3);
		  tick = HAL_GetTick(); //tick value in milliseconds
	  }
	  osDelay(1);
  }
  /* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_gyro_task */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyro_task */
void gyro_task(void *argument)
{
  /* USER CODE BEGIN gyro_task */
  /* Infinite loop */
  for(;;)
  {
	  uint8_t val[2] = {0, 0};
	  int16_t angular_speed = 0;
	  uint32_t T;

	  uint32_t tick = 0;
	  gyroInit();

	  tick = HAL_GetTick();
	  osDelayUntil(10);

	  for (;;)
	  {

	    osDelay(10);
	    if (HAL_GetTick() - tick >= 100)
	    {
	      readByte(0x37, val);
	      angular_speed = (val[0] << 8) | val[1];

	      T = HAL_GetTick() - tick;
	      total_angle += (double)(angular_speed + 7.5) * ((HAL_GetTick() - tick) / 16400.0);
	      turning_angle += (double)(angular_speed + 7.5) * ((HAL_GetTick() - tick) / 16400.0);


	      // prevSpeed = angular_speed;
	      if (total_angle >= 360)
	      {
	        total_angle = 0;
	      }
	      if (total_angle <= -360)
	      {
	        total_angle = 0;
	      }
	      sprintf(OLED_Row_5, "ANGLE: %6d\0", (int)(total_angle));

	      tick = HAL_GetTick();
	    }
	  }
    osDelay(1);
  }
  /* USER CODE END gyro_task */
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
