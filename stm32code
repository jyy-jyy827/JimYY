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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
int Start_Yaw,Bed1_Yaw,Bed3_Yaw;
int Pwm=500;/*max=1000*/
int MotoEncoderSpeed[3],MotoTargetSpeed[3];
int TargetAngle[3],RealAngle[3];
int Distance[3];
uint8_t data[50];
int SpeedError,SpeedError_Last=1,SpeedError_Prev=1; 
int AngleError,AngleError_Last=1,AngleError_Prev=1;
int YawError  ,YawError_Last=1  ,YawError_Prev=1;
float Speed_Pwm_add,Speed_Pwm;
float Angle_Pwm_add,Angle_Pwm[3];
float Speed_KP,Speed_KI,Speed_KD;
float Angle_KP,Angle_KI,Angle_KD;
float Yaw_KP  ,Yaw_KI  ,Yaw_KD  ;
int SampleT;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
int ReadEncoder(int);
float SpeedControl(int,int);
float AngleControl(int,int);
void MotoEnable(int,int);
void MotoSetPwm(int,float);
int GetYaw(void);
int Yaw_Start(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start (&htim4 ,TIM_CHANNEL_4 );
	HAL_TIM_PWM_Start (&htim2 ,TIM_CHANNEL_3 );
	HAL_TIM_PWM_Start (&htim2 ,TIM_CHANNEL_4 );
	HAL_TIM_Encoder_Start (&htim3 ,TIM_CHANNEL_1 );
	HAL_TIM_Encoder_Start (&htim3 ,TIM_CHANNEL_2 );
	HAL_TIM_Encoder_Start (&htim8 ,TIM_CHANNEL_1 );
	HAL_TIM_Encoder_Start (&htim8 ,TIM_CHANNEL_2 );
	HAL_TIM_Encoder_Start (&htim1 ,TIM_CHANNEL_1 );
	HAL_TIM_Encoder_Start (&htim1 ,TIM_CHANNEL_2 );
  HAL_TIM_Base_Start_IT (&htim2 );
  HAL_TIM_Base_Start_IT (&htim4 );
	Speed_Pwm =500;
	Speed_KP=5;Speed_KI=4;Speed_KD=2;
	Angle_KP=7;Angle_KI=20;Angle_KD=1;
	TargetAngle [0]=-15;
	TargetAngle [1]=-15;
	TargetAngle [2]=-15;
	HAL_GPIO_WritePin (GPIOF,GPIO_PIN_10,GPIO_PIN_RESET );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_TogglePin (GPIOF,GPIO_PIN_9);
	  HAL_Delay (1000);
		//HAL_Delay (500);HAL_Delay (500);HAL_Delay (500);HAL_Delay (500);HAL_Delay (500);
		//__HAL_TIM_SetCounter (&htim1 ,0);__HAL_TIM_SetCounter (&htim3 ,0);__HAL_TIM_SetCounter (&htim8 ,0);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 144;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		static unsigned int i = 0;
    /*if (htim == (&htim2))
    {
			 
				
        i++;
        if(i> 500)//Ã¿10msÖÐ¶ÏÒ»´Î£¬sampleTÎª²ÉÑùÖÜÆÚ
        {   
						Speed_Pwm = SpeedControl(MotoEncoderSpeed[0],MotoTargetSpeed[0]);
						MotoSetPwm (0,Speed_Pwm);					
						MotoEncoderSpeed[0] =ReadEncoder (0);	
						HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
						printf("%.2f,%.2f,% d,% d,% d,% d,% .2f\n",(float)MotoTargetSpeed[0]*80,(float)MotoEncoderSpeed[0]*80,(int)(Speed_KP*10),(int)(Speed_KI*10),(int)(Speed_KD*10),(int)SampleT,(float)Speed_Pwm);
						//10Îª×Ô¼ºÌí¼ÓµÄÄÚÈÝÖ»ÊÇÎªÁË·Å´ópidÐ§¹û
						//printf ("Error = %d,%d,%dpwmadd=%f\r\n",Error,Error_Last,Error_Prev,Pwm_add  );
					  i=0;
        }
			
    }*/
		if (htim == (&htim4))//angle_control_timer
		{
			  i++;
        if(i> 100){
						RealAngle[0]=__HAL_TIM_GetCounter(&htim1);
						if(RealAngle[0]>=5000){RealAngle[0]=RealAngle[0]-9999;}
						Angle_Pwm[0]=AngleControl(TargetAngle[0] ,RealAngle[0]);
						RealAngle[1]=__HAL_TIM_GetCounter(&htim3);
						if(RealAngle[1]>=5000){RealAngle[1]=RealAngle[1]-9999;}
						Angle_Pwm[1]=AngleControl(TargetAngle[1] ,RealAngle[1]);
						RealAngle[2]=__HAL_TIM_GetCounter(&htim8);
						if(RealAngle[2]>=5000){RealAngle[2]=RealAngle[2]-9999;}
						Angle_Pwm[2]=AngleControl(TargetAngle[2],RealAngle[2]);
						if(Angle_Pwm[0]!=0){MotoSetPwm(0,Angle_Pwm[0]);}
						if(Angle_Pwm[1]!=0){MotoSetPwm(1,Angle_Pwm[1]);}
						if(Angle_Pwm[2]!=0){MotoSetPwm(2,Angle_Pwm[2]);}
						//printf("%.2f,%.2f,% d,% d,% d,% d,% .2f\n",(float)TargetAngle[1],(float)RealAngle[1],(int)(Angle_KP*10),(int)(Angle_KI*10),(int)(Angle_KD*10),(int)SampleT,(float)Angle_Pwm[1]);
						i=0;
				}
		}
}
void MotoEnable(int moto,int dir)//MotoÎªµç»úÐòºÅ£¬dirÎª·½Ïò£¬0Í£1ÄæÊ±Õë2Ë³Ê±Õë
{
		switch(moto)
		{
			case 0:
					if(dir == 0){
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_0,GPIO_PIN_RESET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_1,GPIO_PIN_RESET );
						  break;}
					else if(dir == 1){
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_0,GPIO_PIN_RESET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_1,GPIO_PIN_SET);
						  break;}
					else {
					    HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_0,GPIO_PIN_SET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_1,GPIO_PIN_RESET);
						  break;}
			case 1:
					if(dir == 0){
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_2,GPIO_PIN_RESET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_3,GPIO_PIN_RESET);
						  break;}
					else if(dir == 1){
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_2,GPIO_PIN_RESET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_3,GPIO_PIN_SET);
						  break;}
					else{
					    HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_2,GPIO_PIN_SET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_3,GPIO_PIN_RESET);
						  break;}
			case 2:
					if(dir == 0){
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_4,GPIO_PIN_RESET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_5,GPIO_PIN_RESET);
						  break;}
					else if(dir == 1){
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_4,GPIO_PIN_RESET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_5,GPIO_PIN_SET);
						  break;}
					else {
					    HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_4,GPIO_PIN_SET);
							HAL_GPIO_WritePin (GPIOF ,GPIO_PIN_5,GPIO_PIN_RESET);
						  break;}				
		}
}
void MotoSetPwm(int moto,float pwm)
{
	  if(pwm < 0){
				MotoEnable (moto,2);pwm=-pwm;
		}
		else{
				MotoEnable (moto,1);
		}
		switch(moto){
			case 0:
					__HAL_TIM_SetCompare (&htim4,TIM_CHANNEL_4 ,(int)pwm );
					break;
      case 1:
					__HAL_TIM_SetCompare (&htim2,TIM_CHANNEL_3 ,(int)pwm );
					break;
			case 2:
					__HAL_TIM_SetCompare (&htim2,TIM_CHANNEL_4 ,(int)pwm );
					break;
		}
}
int ReadEncoder(int moto)//get encoder data
{		
	  int Speed;
		switch (moto)
		{
			case 0:Speed =__HAL_TIM_GetCounter (&htim1 );
						 __HAL_TIM_SetCounter (&htim1 ,0);
             if(Speed >=900){Speed = Speed - 9999;}
						 break;
			case 1:Speed =__HAL_TIM_GetCounter (&htim3 );
						 __HAL_TIM_SetCounter (&htim3 ,0);
             if(Speed >=900){Speed = Speed - 9999;}
  					 break;
			case 2:Speed =__HAL_TIM_GetCounter (&htim8 );
						 __HAL_TIM_SetCounter (&htim8 ,0);
             if(Speed >=900){Speed = Speed - 9999;}
  					 break;
		}
		return Speed ;
}
float SpeedControl(int motospeed,int targetspeed)//moto speed pid control
{
			  SpeedError = targetspeed  - motospeed ;
			  Speed_Pwm_add = Speed_KP *(SpeedError-SpeedError_Last)+
									      Speed_KI *SpeedError+
												Speed_KD *(SpeedError -2.0f*SpeedError_Last +SpeedError_Prev )+
												0.01;
			  Speed_Pwm += Speed_Pwm_add ;
			  SpeedError_Prev = SpeedError_Last;	  	  
			  SpeedError_Last = SpeedError;	            
			  if( Speed_Pwm > 900)  Speed_Pwm = 900;	   
		  	if( Speed_Pwm < 200)  Speed_Pwm = 200;
      return  Speed_Pwm;	             
}
float AngleControl(int TargetAngle,int RealAngle)//moto angle pid control
{
	  float pwm;
		if(__fabs (TargetAngle - RealAngle)<2 )
		{
		pwm=0;
		}
		AngleError =TargetAngle - RealAngle;
		pwm += Angle_KP*(AngleError-AngleError_Last)+
				   Angle_KI*AngleError+
					 Angle_KD*(AngleError -2.0f*AngleError_Last +AngleError_Prev )+
					 0.01;
		AngleError_Prev = AngleError_Last;	  	 
		AngleError_Last = AngleError;	
		if(pwm >  900) pwm =  900;
		if(pwm < -900) pwm = -900;
		return pwm;		
}
float YawControl(int TargetYaw,int RealYaw)//moto angle pid control
{
	  float Angle;
		YawError =TargetYaw - RealYaw;
		Angle += Yaw_KP*(YawError-YawError_Last)+
				   Yaw_KI*YawError+
					 Yaw_KD*(YawError -2.0f*YawError_Last +YawError_Prev )+
					 0.01;
		YawError_Prev = YawError_Last;	  	 
		YawError_Last = YawError;			
		return Angle;		
}
int fputc(int ch, FILE *f)//defination of "printf()"
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int GetYaw(void)//basic get yaw angle function
{
		int Yaw=0;
		loop:{
				for(int i=0;i<50;i++){
						HAL_UART_Receive (&huart5 ,&data[i],1,10);
						}
				for(int i=0;i<40;i++){
						if(data [i]==0x55 && data[i+1]==0x53){
						Yaw=(data[i+6]*256+data[i+7])*180/32768;
						}	  
				}
		}
		if(Yaw==0)goto loop;
		return Yaw;
}
int Yaw_Stable(void)//get stable starting yaw angle functiom
{
		int yaw[10],exam,i,j,final;
		bool flag=0;
		do{		  
				for(i=0;i<10;i++){
						yaw[i]=GetYaw();
				}
				for(i=0;i<10;i++){
					  exam=0;
						for(j=0;j<10;j++){
								if(yaw[i]==yaw[j]){exam++;}
								if(exam>8){flag=1;final=yaw[i];}						
						}
				}
		}while(!flag);
		return final;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
