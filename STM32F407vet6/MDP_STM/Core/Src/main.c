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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "PID.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _command{
	char MOTOR_DIR;
	char SERVO_DIR;
	int MAGNITUDE;
} Cmd;
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
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
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
/* Definitions for UltraSoundTask */
osThreadId_t UltraSoundTaskHandle;
const osThreadAttr_t UltraSoundTask_attributes = {
  .name = "UltraSoundTask",
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
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void Motor(void *argument);
void encoder_task(void *argument);
void gyro_task(void *argument);
void ultrasound_task(void *argument);

/* USER CODE BEGIN PFP */
void forward_motor_prep();
void backward_motor_prep();
void servomotor_center();
void servomotor_left();
void servomotor_right();
void servomotor_set(int value);
void move(float distance , int frontorback, int leftorright);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void send_message(uint8_t * header, uint8_t * message);
void process_UART_Rx();
void gyroInit();
void writeByte(uint8_t addr,uint8_t data);
void readByte(uint8_t addr, uint8_t* data);
void state_controller(Cmd *command);
void pid_controller();
void prep_robot(char MDIR, char SDIR);
void right_turn(int angle);
void left_turn(int angle);
void reset_trackers();
int calc_progress(Cmd command);
void HCSR04_Read (void);
void delay_us(uint16_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//UART Global Variables
const int BUFFER_SIZE = 20;
uint8_t aRxBuffer[20]; //Buffer of 20 bytes
//Gyroscope Global Variables
uint8_t ICM_ADDR = 0x68;
uint8_t buff[20]; //Gyroscope buffer
double TOTAL_ANGLE = 0;
double TURNING_ANGLE = 0;
double TARGET_ANGLE = 0;
//OLED Global Variables
uint8_t OLED_Row_0[20],OLED_Row_1[20],OLED_Row_2[20],OLED_Row_3[20],OLED_Row_4[20],OLED_Row_5[20];
//Motor Global Variables
int PWML, PWMR;
int motor_dir; //Backward = -1, Stop = 0, Forward = 1
//Encoder Glober Variable
int left_speed, right_speed;
double LEFTWHEEL_DIST = 0;
double RIGHTWHEEL_DIST = 0;
//Servo Global Variables
int servo_dir; //Left = -1, Center = 0, Right = 1
int SERVO_CENTER = 150;
int SERVO_LEFTMAX = 100;
int SERVO_RIGHTMAX = 210;
int SERVO_CURRENT;
//State Control & PID Global Variables
int RX_FLAG = 0; //Signals when RX message received
int BUSY = 0; //Tracks when a command is being processed
int PID_DELAY = 0; //Delay PID for 500ms
int PID_ENABLE = 0; //PID on/off
double DEVIATION = 0; //Deviation angle
int ANGLE_OFFSET = 0; //Angle offset for straight movement
char RX_MOTOR;
char RX_SERVO;
int RX_MAG;
//UART Global Variables
uint8_t TX_STRING[9]; //9 bytes to transfer (Header + 5 Byte message)
uint8_t UART_RPI[4] = {'R','P','I','|'};
uint8_t UART_ALG[4] = {'A','L','G','|'};
uint8_t UART_AND[4] = {'A','N','D','|'};
//Ultrasound Global Variables
uint32_t Echo_Val1 = 0;
uint32_t Echo_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
double ultra_Distance  = 0; //Ultrasound distance
//Nmber of ticks for one full rotation of wheel
double full_rotation_wheel = 1600;
double circumference_wheel = 21.3;
Cmd command;
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
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_TIM_Base_Start(&htim7); //start timer to use delay_ms function for ultrasonic task
  HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer, 5); //Receive 3 bytes
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

  /* creation of UltraSoundTask */
  UltraSoundTaskHandle = osThreadNew(ultrasound_task, NULL, &UltraSoundTask_attributes);

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim7.Init.Prescaler = 16-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED_3_Pin|Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED_3_Pin Trig_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED_3_Pin|Trig_Pin;
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
	osDelay(250);
}

void servomotor_left()
{
	// default: 107
	uint32_t value = 107;
	if (htim1.Instance->CCR4 == value){
			return;
	}
	htim1.Instance->CCR4 = value;
	osDelay(400);
}

void servomotor_right()
{
	// default: 225
	uint32_t value = 225;
	if (htim1.Instance->CCR4 == value){
			return;
	}
	htim1.Instance->CCR4 = value;
	osDelay(400);
}

void servomotor_set(int value)
{
	if (value >= 200) value = 200;
	if (value <= 100) value = 100;
	if (htim1.Instance->CCR4 == value){
			return;
	}
	htim1.Instance->CCR4 = value;
	osDelay(400);
}

void move(float distance, int frontorback , int leftorright) //Keep for debugging purpose
{
	//default PWM values
	int pwmval_L = 1500;
	int pwmval_R = 1500;
	int flag = 1;

	//distance of the left & right wheel travelled
	float LEFTWHEEL_DIST = 0;
	float RIGHTWHEEL_DIST = 0;
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
			left_encoder_prev = __HAL_TIM_GET_COUNTER(&htim2);
			right_encoder_prev = __HAL_TIM_GET_COUNTER(&htim3);
			start_time = HAL_GetTick(); //tick value in milliseconds

			//Calculate the distance covered and compare with distance we input
			temp_leftwheel_dist = ((float)diff_left/full_rotation_wheel) * circumference_wheel;
			LEFTWHEEL_DIST += temp_leftwheel_dist;

			temp_rightwheel_dist = ((float)diff_right/full_rotation_wheel) * circumference_wheel;
			RIGHTWHEEL_DIST += temp_rightwheel_dist;

			if (LEFTWHEEL_DIST >= distance || RIGHTWHEEL_DIST >= distance){ //stop condition
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
				flag = 0;
			}
		}
	}
	HAL_Delay(100);
}

void right_turn(int angle) //Keep for debugging purpose
{
	//Prep the servomotor to right
	servomotor_right();

	osDelay(250);

	TURNING_ANGLE = 0;
	forward_motor_prep();
	//Start the motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);

	while (TURNING_ANGLE > -1*angle){ //while it is still turning to the correct angle
		osDelay(10);
	}


	//Stop the Motor are completing the turn
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	servomotor_center();

}

void left_turn(int angle) //Keep for debugging purpose
{
	//Prep the servomotor to left
	servomotor_left();

	osDelay(250);

	TURNING_ANGLE = 0;
	forward_motor_prep();
	//Start the motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);

	while (TURNING_ANGLE < angle){ //while it is still turning to the correct angle
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
	if(RX_FLAG == 0){
		RX_MOTOR = (char) aRxBuffer[0];
		RX_SERVO = (char) aRxBuffer[1];
		RX_MAG = ((int)(aRxBuffer[2] - '0') * 100) + ((int)(aRxBuffer[3] - '0') * 10) + ((int)(aRxBuffer[4] - '0'));
		RX_FLAG = 1;
	}
	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,5);
}

void send_message(uint8_t * header, uint8_t * message){
	uint8_t transmit[9];
	int pos;
	for (pos=0;pos<4;pos++) transmit[pos] = header[pos];
	for (pos=0;pos<5;pos++) transmit[pos+4] = message[pos];
	HAL_UART_Transmit_IT(&huart3,(uint8_t *)transmit,9);
}

void process_UART_Rx() //Keep for debugging purpose
{
	if (RX_FLAG == 1){
		prep_robot(RX_MOTOR, RX_SERVO);
		RX_FLAG = 0;
	}
}

void state_controller (Cmd *command) {
	uint8_t complete = 0;
	double angle;
	if (command->SERVO_DIR == 'C'){
		//Check if robot has reached magnitude based on distance
		if (((LEFTWHEEL_DIST+RIGHTWHEEL_DIST)/2 + 5) >= command->MAGNITUDE){
			complete = 1;
		}
	}
	else if (command->SERVO_DIR == 'L' || command->SERVO_DIR == 'R'){
		//Check if robot has reached magnitude based on angle moved
//		if (TURNING_ANGLE < 0) angle = -TURNING_ANGLE;
//		else angle = TURNING_ANGLE;
//		if (angle + 9 >= command->MAGNITUDE) complete = 1;
		//Control by target_angle
		angle = TARGET_ANGLE - TOTAL_ANGLE;
		if (angle < 0) angle = -angle;
		while (angle > 360) angle -= 360;
		if (angle - 7 <= 0) complete = 1; //previous offset -9
	}
	else {
		osDelay(0.500 * command->MAGNITUDE);
		complete = 1;
	}
	//Send complete
	if (complete == 1){
		command->MOTOR_DIR = 0; command->SERVO_DIR = 0; command->MAGNITUDE = 0;
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
		TX_STRING[0] = 'A'; TX_STRING[1] = 'L'; TX_STRING[2] = 'G'; TX_STRING[3] = '|';
		TX_STRING[4] = 'C'; TX_STRING[5] = 'M'; TX_STRING[6] = 'P'; TX_STRING[7] = 'L'; TX_STRING[8] = 'T';
		HAL_UART_Transmit_IT(&huart3,(uint8_t *)TX_STRING,9);
//		send_message(UART_ALG, TX_STRING);
		BUSY = 0;
	}
}

void prep_robot(char MDIR, char SDIR){
	int state_change = 0;
	//Control servo direction L or R
	if (SDIR == 'L'){
		if (servo_dir != -1){
			servomotor_left();
			servo_dir = -1;
			state_change = 1;
		}
	}
	else if (SDIR == 'R'){
		if (servo_dir != 1){
			servomotor_right();
			servo_dir = 1;
			state_change = 1;
		}
	}
	//Control motor direction
	if (MDIR == 'F'){
		if (motor_dir != 1){
			forward_motor_prep();
			motor_dir = 1;
			state_change = 1;
		}
	}
	else if (MDIR == 'B'){
		if (motor_dir != -1){
			backward_motor_prep();
			motor_dir = -1;
			state_change = 1;
		}
	}
	else {
		if (motor_dir != 0){
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
			forward_motor_prep();
			motor_dir = 0;
			state_change = 1;
			osDelay(500);
		}
	}
	//Control servo direction C & PID fine control
	if (PID_ENABLE == 0){
		if (SDIR == 'C'){
			if (servo_dir != 0){
				servomotor_center();
				servo_dir = 0;
				state_change = 1;
			}
		}
		else if (SDIR != 'L' && SDIR != 'R' && SDIR != 'C') {
			if (servo_dir != 0){
				servomotor_center();
				servo_dir = 0;
				state_change = 1;
			}
		}
	}
	//State changes
	if (state_change == 1) reset_trackers();
}

void reset_trackers(){
	PID_DELAY = 1;
	PID_ENABLE = 0;
	TURNING_ANGLE = 0;
	DEVIATION = 0;
	LEFTWHEEL_DIST = 0;
	RIGHTWHEEL_DIST = 0;
	while (TARGET_ANGLE > 180) { TARGET_ANGLE -= 360; TOTAL_ANGLE -= 360; }
	while (TARGET_ANGLE < -180) { TARGET_ANGLE +=360; TOTAL_ANGLE += 360; }
}

int calc_progress(Cmd command){
	double progress;
	double angle;
	if (command.SERVO_DIR == 'C') {
		progress = (double)(LEFTWHEEL_DIST + RIGHTWHEEL_DIST) / (2 * (double)command.MAGNITUDE);
	}
	else if (command.SERVO_DIR == 'L' || command.SERVO_DIR == 'R'){
		if (TURNING_ANGLE < 0) angle = -TURNING_ANGLE;
		else angle = TURNING_ANGLE;
		progress = angle / (double)command.MAGNITUDE;
	}
	else{
		progress = 0;
	}
	return (int)(progress*1000);
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

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim7,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim7) < us);  // wait for the counter to reach the us input in the parameter
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			Echo_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			Echo_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (Echo_Val2 > Echo_Val1)
			{
				Difference = Echo_Val2-Echo_Val1;
			}

			else if (Echo_Val1 > Echo_Val2)
			{
				//Difference = (0xffff - Echo_Val1) + Echo_Val2;
				Difference = 0;
			}

			ultra_Distance = (Difference * 0.0343)/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
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
	int progress = 0;
	command.MOTOR_DIR = 0;
	command.SERVO_DIR = 0;
	command.MAGNITUDE = 0;
	TURNING_ANGLE = 0;
	LEFTWHEEL_DIST = 0;
	RIGHTWHEEL_DIST = 0;
//	uint32_t tick;
//	tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	//Clear queue if XXX command received
	if (RX_FLAG == 1){
		if (RX_MOTOR == 'X' && RX_SERVO == 'X'){
			//Force stop
			progress = calc_progress(command); //return progress in XXX = XX.X%
			command.MOTOR_DIR = 0;
			command.SERVO_DIR = 0;
			command.MAGNITUDE = 0;
			reset_trackers();
			BUSY = 0;
		}
		else {
			if (BUSY == 0) {
				command.MOTOR_DIR = RX_MOTOR;
				command.SERVO_DIR = RX_SERVO;
				command.MAGNITUDE = RX_MAG;
				if (command.MOTOR_DIR == 'F' && command.SERVO_DIR != 'C'){
					if (command.SERVO_DIR == 'L') TARGET_ANGLE += command.MAGNITUDE;
					if (command.SERVO_DIR == 'R') TARGET_ANGLE -= command.MAGNITUDE;
				}
				else if (command.MOTOR_DIR == 'B' && command.SERVO_DIR != 'C'){
					if (command.SERVO_DIR == 'L') TARGET_ANGLE -= command.MAGNITUDE;
					if (command.SERVO_DIR == 'R') TARGET_ANGLE += command.MAGNITUDE;
				}
				if (TARGET_ANGLE >= 720) TARGET_ANGLE -= 720;
				if (TARGET_ANGLE <= -720) TARGET_ANGLE += 720;
				reset_trackers();
				BUSY = 1;
			}
			else {
				TX_STRING[0] = 'R'; TX_STRING[1] = 'P'; TX_STRING[2] = 'I'; TX_STRING[3] = '|';
				TX_STRING[4] = 'B'; TX_STRING[5] = 'U'; TX_STRING[6] = 'S'; TX_STRING[7] = 'Y'; TX_STRING[8] = '_';
				HAL_UART_Transmit_IT(&huart3,(uint8_t *)TX_STRING,9);
//				send_message(UART_RPI, TX_STRING);
			}
		}
		RX_FLAG = 0;
	}
  	HAL_GPIO_TogglePin(GPIOE, LED_3_Pin);
  	if (BUSY == 1) state_controller(&command);
  	prep_robot(command.MOTOR_DIR, command.SERVO_DIR);
  	if (progress != 0){
  		//Transmit progress
  		TX_STRING[0] = 'A'; TX_STRING[1] = 'L'; TX_STRING[2] = 'G'; TX_STRING[3] = '|';
		TX_STRING[4] = 'P'; TX_STRING[5] = '='; TX_STRING[6] = '0'+((progress/100)%10); TX_STRING[7] = '0'+((progress/10)%10); TX_STRING[8] = '0'+((progress)%10);
		HAL_UART_Transmit_IT(&huart3,(uint8_t *)TX_STRING,9);
		progress = 0;
  	}
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
		sprintf(OLED_Row_1,"PWM L: %6d\0",PWML);
		sprintf(OLED_Row_2,"PWM R: %6d\0",PWMR);
//		sprintf(OLED_Row_3,"SPD L: %6d\0",left_speed);
//		sprintf(OLED_Row_4,"SPD R: %6d\0",right_speed);
//		sprintf(OLED_Row_2,"OFFST: %6d\0",ANGLE_OFFSET);
//		sprintf(OLED_Row_3,"DEVIA: %6d\0",(int)DEVIATION);
		sprintf(OLED_Row_3,"T ANG: %6d\0",(int)TARGET_ANGLE);
		OLED_ShowString(10,0,OLED_Row_0); //Show motor state
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
	motor_dir = 0; servo_dir = 0;
	forward_motor_prep();
	servomotor_center();

	int pwm_L_f, pwm_L_b;
	int pwm_R_f, pwm_R_b;
	int pwm_L_div = 1, pwm_R_div = 1;
//	int offset_L, offset_R;
	pwm_L_f = 1950;
	pwm_L_b = 2050;
	pwm_R_f = 2050;
	pwm_R_b = 2100;
//	double deviation_angle, deviation_prev;
//	double turning_prev = 0;
//	double off_angle = 0;

	struct PIDController motor_LF_PID, motor_RF_PID, motor_LB_PID, motor_RB_PID;

	MotorPIDController_Init(&motor_LF_PID);
	MotorPIDController_Init(&motor_RF_PID);
	MotorPIDController_Init(&motor_LB_PID);
	MotorPIDController_Init(&motor_RB_PID);
	//Make sure the Servo motor position is in middle position
	servomotor_center();
	/* Infinite loop */
	uint32_t tick = HAL_GetTick();
	uint32_t pid_time_start = 0;
	for(;;)
	{	if (HAL_GetTick() - tick > 100L){
			//Calculate wheel speed change from turning
			if (servo_dir != 0){
				if (servo_dir == -1) {pwm_L_div = 2; pwm_R_div = 1;}
				if (servo_dir == 1) {pwm_L_div = 1; pwm_R_div = 2;}
			}
//			else if (motor_dir != 0) {
//				if (command.MAGNITUDE - ((LEFTWHEEL_DIST + RIGHTWHEEL_DIST)/2) < 30) {
//					pwm_L_div = 2; pwm_R_div = 2;
//				}
//			}
			else {pwm_L_div = 1; pwm_R_div = 1;}
			//Calculate deviation
			if (motor_dir != 0 && servo_dir == 0){
//				off_angle = TOTAL_ANGLE - TARGET_ANGLE; //if total-target > 0 too left, if total-target < 0 too right
//				deviation_angle = ((off_angle+turning_prev)/2) * ((2*3.14159)/360);
//				DEVIATION -= deviation_angle * (((double)left_speed * (circumference_wheel/full_rotation_wheel))*((double)(HAL_GetTick()-tick))/1000);
//				if (DEVIATION > 0.5){
//					if (off_angle <= 0) ANGLE_OFFSET = (int)(DEVIATION*5);
//					if (off_angle > 0) ANGLE_OFFSET = (int)(-3*off_angle);
//				}
//				else if (DEVIATION < 0.5){
//					if (TURNING_ANGLE >= 0) ANGLE_OFFSET = (int)(DEVIATION*5);
//					if (TURNING_ANGLE < 0) ANGLE_OFFSET = (int)(-3*off_angle);
//				}
//				else ANGLE_OFFSET = 0;
//				turning_prev = off_angle;
				ANGLE_OFFSET = 3*((int)(TARGET_ANGLE - TOTAL_ANGLE));
				if (ANGLE_OFFSET >= 10) ANGLE_OFFSET = 10;
				if (ANGLE_OFFSET <= -10) ANGLE_OFFSET = -10;
			}
			//Control PID enable
			if (PID_DELAY == 1){
				pid_time_start = HAL_GetTick();
				PID_DELAY = 0;
//				turning_prev = 0;
			}
			if (PID_ENABLE == 0 && (HAL_GetTick() - pid_time_start > 400L)){
				PID_ENABLE = 1;
			}
			//Control motor
			if (motor_dir == 1){
				//Start the motor
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_L_f/pwm_L_div);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_R_f/pwm_R_div);
				if (servo_dir == 0) servomotor_set(SERVO_CENTER-ANGLE_OFFSET);
				//ADD PID CONTROL
				if (PID_ENABLE == 1){
					pwm_L_f = pwm_L_div*PIDController_Update(&motor_LF_PID, left_speed, 2500/pwm_L_div, pwm_L_f/pwm_L_div);
					pwm_R_f = pwm_R_div*PIDController_Update(&motor_RF_PID, right_speed, 2500/pwm_R_div, pwm_R_f/pwm_R_div);
				}
				//display to OLED
				PWML = pwm_L_f/pwm_L_div;
				PWMR = pwm_R_f/pwm_R_div;
			}
			else if (motor_dir == -1){
				//Start the motor
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_L_b/pwm_L_div);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_R_b/pwm_R_div);
				if (servo_dir == 0) servomotor_set(SERVO_CENTER+ANGLE_OFFSET);
				//ADD PID CONTROL
				if (PID_ENABLE == 1){
					pwm_L_b = pwm_L_div*PIDController_Update(&motor_LB_PID, left_speed, 2500/pwm_L_div, pwm_L_b/pwm_L_div);
					pwm_R_b = pwm_R_div*PIDController_Update(&motor_RB_PID, right_speed, 2500/pwm_R_div, pwm_R_b/pwm_R_div);
				}
				//Display to OLED
				PWML = pwm_L_b/pwm_L_div;
				PWMR = pwm_R_b/pwm_R_div;
			}
			else {
				PWML = 0;
				PWMR = 0;
				PID_ENABLE = 0;
				ANGLE_OFFSET = 0;
//				turning_prev = TOTAL_ANGLE-TARGET_ANGLE;
//				deviation_prev = 0;
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

  uint32_t tick, cur_tick, T;

  left_prev = __HAL_TIM_GET_COUNTER(&htim2);
  right_prev = __HAL_TIM_GET_COUNTER(&htim3);
  tick = HAL_GetTick(); //tick value in milliseconds

  for(;;)
  {
	  cur_tick = HAL_GetTick();
	  if (cur_tick - tick > 50L){ //every 0.05 second
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

		  //Calculate distance traveled
//		  distInt_L += left_diff * dir_L;
//		  distInt_R += right_diff * dir_R;
//		  sprintf(OLED_Row_1, "DST L: %6d\0", distInt_L);
//		  sprintf(OLED_Row_2, "DST R: %6d\0", distInt_R);
		  LEFTWHEEL_DIST += (double)left_diff * (circumference_wheel/full_rotation_wheel);
		  RIGHTWHEEL_DIST += (double)right_diff * (circumference_wheel/full_rotation_wheel);

		  //Reset counters
		  left_prev = __HAL_TIM_GET_COUNTER(&htim2);
		  right_prev = __HAL_TIM_GET_COUNTER(&htim3);
		  tick = HAL_GetTick(); //tick value in milliseconds
	  }
	  osDelay(10);
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
	double offset = 7.848882995;//High power offset 7.85 //Low power offset 7.8475
	double angle;
  for(;;)
  {
	  uint8_t val[2] = {0, 0};
	  int16_t angular_speed = 0;

	  uint32_t tick = 0;
	  gyroInit();

	  tick = HAL_GetTick();
	  osDelayUntil(10);

	  for (;;)
	  {

	    osDelay(10);
	    if (HAL_GetTick() - tick >= 50L)
	    {
	      readByte(0x37, val);
	      angular_speed = (val[0] << 8) | val[1];

//	      if (motor_dir == 0){
//	    	  offset = (offset-(double)(angular_speed))/2;
//	      }

	      angle = ((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0);
//	      if (motor_dir == 0) angle = 0;
	      TOTAL_ANGLE += angle;
	      TURNING_ANGLE += angle;

	      // prevSpeed = angular_speed;
	      if (TOTAL_ANGLE >= 720)
	      {
	        TOTAL_ANGLE -= 720;
	      }
	      if (TOTAL_ANGLE <= -720)
	      {
	        TOTAL_ANGLE += 720;
	      }
//	      sprintf(OLED_Row_1, "OFSET: %6d\0", (int)((offset*10000)));
	      sprintf(OLED_Row_4, "ANGLE: %6d\0", (int)(TOTAL_ANGLE));
	      sprintf(OLED_Row_5, "A_100: %6d\0", (int)(100*TOTAL_ANGLE));

	      tick = HAL_GetTick();
	    }
	  }
    osDelay(1);
  }
  /* USER CODE END gyro_task */
}

/* USER CODE BEGIN Header_ultrasound_task */
/**
* @brief Function implementing the UltraSoundTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultrasound_task */
void ultrasound_task(void *argument)
{
  /* USER CODE BEGIN ultrasound_task */
  /* Infinite loop */
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	uint32_t tick;
	tick = HAL_GetTick();
	for(;;)
	{
	  if (HAL_GetTick() - tick > 200L) {
		  HCSR04_Read();
		  osDelay(10);
		  if (ultra_Distance > 40) ultra_Distance = -1;
		  sprintf(OLED_Row_0, "UDIST: %6d\0", (int)ultra_Distance);
	//	  if (motor_dir == 1 && ultra_Distance <= 15L && ultra_Distance > 2L){ //avoid collision with the obstacle
	//			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	//			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	//			PWML = 0;
	//			PWMR = 0;
	//			PID_ENABLE = 0;
	//			//Stop the movement of the robot and send the current progress
	//			//Pass the control over to StartDefaultTask()
	//			RX_FLAG = 1;
	//			RX_MOTOR = 'X';
	//			RX_SERVO = 'X';
	//	  }
		  tick = HAL_GetTick();
	  }
	  //HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
	}
  /* USER CODE END ultrasound_task */
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
