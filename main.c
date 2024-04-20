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
#include "stdlib.h"
#include "string.h"
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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t rawValue;
float uVal;
float iVal;
float sbVal;
float phdVal;
float batVal;
float uMax = 7.00;
float uSet = 5.50; // на модуле заряда 5В недостаточно
float iSet = 0.25;
float iSetTmp = 0;
float k = 1;

uint32_t pwmT11CH1Counter = 0;            // длительность Ш�?М
uint32_t stepPWM = 10;

float analogMul = 1;                // растягивающий множитель

uint8_t i2cSenseOn  = 0;
uint8_t SBatSenseOn = 0;
uint8_t PDSenseOn   = 1;
uint8_t UARTSenseOn = 1;
uint8_t phdSenseOn = 1;

uint8_t flag = 1; // ждать прерываний только если flag == 0
volatile uint16_t timer = 0;

unsigned char USART_RXBuf[64];
unsigned char USART_TXBuf[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t ADC_Result(ADC_HandleTypeDef* hadc, uint32_t);
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc); // не работает с l151
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  //  pwmT11CH1Counter=300;
  pwmT11CH1Counter=1023;
  TIM11->CCR1=pwmT11CH1Counter;

  //  HAL_Delay(1000);

  //  uint8_t buff[16] = {0,};
  HAL_UART_Transmit(&huart1, (uint8_t*)"//////////////////////////////////// \n", 38, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"Solar battery imitator V1.0 \n", 29, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"uSet = 5.5 V, iSet = 0.25 A \n", 29, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"photodiode sensor is ON \n", 25, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"//////////////////////////////////// \n", 38, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to set voltage press U=x.xx \n", 29, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to set current press I=x.xx \n", 29, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"//////////////////////////////////// \n", 38, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to use photodiode press phd_on \n", 32, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to use I2C sensor press i2c_on \n", 32, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to use solar battery press sbs_on \n", 36, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to manage U&I with UART press UIuart \n", 38, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"to simulate solar orbit press sunorb \n", 38, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"//////////////////////////////////// \n", 38, 1000);
  HAL_UART_Receive_IT(&huart1, (uint8_t*)USART_RXBuf, 6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
///////////////////////////////////////////////////////////////////////////////////////////////////
	  	//read ADC data

	  	uVal = ADC_Result(&hadc, 0); //Vout
	  	uVal = (3.3/4096)*3.863*uVal;
	  	iVal = ADC_Result(&hadc, 1); //Iout
	  	iVal = (3.3/4096)*0.19*iVal;
	  	batVal = ADC_Result(&hadc, 2); //LiPo Bat
	  	phdVal = ADC_Result(&hadc, 3); //Photo Diode
	  	phdVal = (3.3/4096)*phdVal;
	  	sbVal = ADC_Result(&hadc, 4); //Solar Bat
	  	sbVal = (3.3/4096)*2*sbVal;

	  	if(SBatSenseOn == 1){
	  		k = sbVal/0.33;
	  	}

	  	if(phdSenseOn == 1){
	  		k = phdVal/3.3;
	  	}

	  	//read i2c optical sensor data

	  	if((i2cSenseOn == 1)){
//	  	if((i2cSenseOn == 1)&&(sysTimerCnt == 1000)){
//	  		sysTimerCnt = 0;
	  	    uint8_t ISMI_ADDR = 0x50;
			ISMI_ADDR = ISMI_ADDR << 1;

			uint8_t buf[32];

			  int16_t val;

		    // Tell ISMI-L that we want to read ADC registers
		  	buf[0]=0;
		  	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, ISMI_ADDR, buf, 1, HAL_MAX_DELAY); //1000
				if ( ret != HAL_OK )
				{
					;
					//	sprintf((char*)buf, "Error Tx I2C1\r\n");
					//	HAL_UART_Transmit(&huart2, buf, 32, 1000);
				}

				else { //HAL_I2C_Master_Transmit(&hi2c1, ISMI_ADDR, 0, 0, 100);
					ret = HAL_I2C_Master_Receive(&hi2c1, ISMI_ADDR, buf, 2, HAL_MAX_DELAY);
					if ( ret != HAL_OK )
					{
						k = 1;
					} else
					{
						val = (buf[1] << 8) + buf[0];
						//i2cK = 100/(float)val;
						k = 191/(float)val;
						//iSet = i2cK*iSet;

					}
				}
	  	}
///////////////////////////////////////////////////////////////////////////////////////////////
	  	//read UART data

	  	if( HAL_UART_Receive_IT (&huart1, USART_RXBuf, 6) != HAL_BUSY ) {
		  	if(strstr (USART_RXBuf, "U=")!= NULL){ //get U
		  		//uSet = strtod (USART_RXBuf, NULL);
	//	  		sprintf(USART_RXBuf, "5.5");

		  		char tmpBuf[64];
		 	  	for(uint8_t i=0; i<6; ++i)
		 	  		tmpBuf[i] = USART_RXBuf[i+2];

		  		uSet = (atof(tmpBuf));

		 		sprintf(USART_TXBuf, "Get U\n");
		 	  	HAL_UART_Transmit_IT(&huart1, USART_TXBuf, 6);

		 	  	for(uint8_t i=0; i<63; ++i)
		 	  		USART_RXBuf[i]=0;

//		 	  	HAL_UART_Receive_IT(&huart1, (uint8_t*)USART_RXBuf, 8);
		  	}

		  	if(strstr (USART_RXBuf, "I=")!= NULL){ //get I

		  		char tmpBuf[64];
		 	  	for(uint8_t i=0; i<6; ++i)
		 	  		tmpBuf[i] = USART_RXBuf[i+2];

		  		iSet = (atof(tmpBuf));

			 	sprintf(USART_TXBuf, "Get I\n");
			 	HAL_UART_Transmit_IT(&huart1, USART_TXBuf, 6);

		 	  	for(uint8_t i=0; i<63; ++i)
		 	  		USART_RXBuf[i]=0;

//		 	  	HAL_UART_Receive_IT(&huart1, (uint8_t*)USART_RXBuf, 8);
		  	}

		  	if (strcmp(USART_RXBuf, "sunorb") == 0) {
			  	HAL_UART_Transmit(&huart1, (uint8_t*)"sun cycle 15 minutes\n", 21, 100);
			  	i2cSenseOn  = 0;
			  	SBatSenseOn = 0;
			  	phdSenseOn = 0;
			  	k = 1;
			  	flag = 0;
			  	timer = 0;
		  	}

		  	if (strcmp(USART_RXBuf, "phd_on") == 0) {
		  	  i2cSenseOn  = 0;
		  	  SBatSenseOn = 0;
		  	  phdSenseOn = 1;
		  	  flag = 1;
		  	  HAL_UART_Transmit_IT(&huart1, "photodiode\n", 11);
		  	}

		  	if (strcmp(USART_RXBuf, "i2c_on") == 0) {
		  			  	  i2cSenseOn  = 1;
		  			  	  SBatSenseOn = 0;
		  			  	  phdSenseOn   = 0;
		  			  	  flag = 1;
		  			  	  HAL_UART_Transmit_IT(&huart1, "i2c sense\n", 10);
		  			  	}

		  	if (strcmp(USART_RXBuf, "sbs_on") == 0) {
		  			  	  i2cSenseOn  = 0;
		  			  	  SBatSenseOn = 1;
		  			  	  phdSenseOn  = 0;
		  			  	  flag = 1;
		  			  	  HAL_UART_Transmit_IT(&huart1, "solar battery\n", 14);
		  			  	}

		  	if (strcmp(USART_RXBuf, "UIuart") == 0) {
		  	  i2cSenseOn  = 0;
		  	  SBatSenseOn = 0;
		  	  phdSenseOn = 0;
		  	  flag = 1;
		  	  k = 1;
		  	  HAL_UART_Transmit(&huart1, (uint8_t*)"only UART manage U and I\n", 25, 100);
		  	  HAL_UART_Transmit(&huart1, (uint8_t*)"to set voltage press U=xx.x \n", 29, 100);
		  	  HAL_UART_Transmit(&huart1, (uint8_t*)"to set current press I=x.xx \n", 29, 100);
		  	}

	//	  	HAL_UART_Receive(&huart2, USART_RXBuf, 33, 100);
	  	}
///////////////////////////////////////////////////////////////////////////////////////////////////
	  	//interrupt on TIM2 few times

	  	if (flag == 0 && timer == 899) {
	  		timer = 0;
			HAL_UART_Transmit(&huart1, "toggle output!\n", 15, 100);
	  		k = 1 - k;
	  	}
///////////////////////////////////////////////////////////////////////////////////////////////////
	  	//set PWM

	  	iSetTmp = iSet;
	  	iSet = k*iSet;
	  	if (iSet > iSetTmp)
	  	{
	  		iSet = iSetTmp;
	  	}

//	  	if(SBatSenseOn == 1){
//	  		iSetTmp = iSet;
//	  		iSet = sbVal/22*14/9; // высчитывается ток и умножается на 1400/900 Вт/м2
//	  		if (iSet >= iSetTmp) {
//	  			iSet = iSetTmp;
//	  		}
//	  	}
//
//	  	else {
//	  		iSetTmp = iSet;
//	  		iSet = k*iSet;
//	  	}

//	  	if(1){

	  	if(iSet>=iVal){
			if(uVal < (uSet - 0.1))
				if(pwmT11CH1Counter > 0)
					pwmT11CH1Counter = pwmT11CH1Counter - stepPWM;
			if(uVal > (uSet + 0.1))
				if(pwmT11CH1Counter < 1023)
					pwmT11CH1Counter = pwmT11CH1Counter + stepPWM;
	  	}
	  	else if(iSet<iVal){
			if(iVal < (iSet - 0.01))
				if(pwmT11CH1Counter > 0)
					pwmT11CH1Counter = pwmT11CH1Counter - stepPWM;
			if(iVal > (iSet + 0.01))
				if(pwmT11CH1Counter < 1023)
					pwmT11CH1Counter = pwmT11CH1Counter + stepPWM;
	  	}

	  	iSet = iSetTmp;
	    TIM11->CCR1=pwmT11CH1Counter;                   // передает длительность Ш�?М таймеру, диапазон 0-1023, частота ~70кГц

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  htim2.Init.Prescaler = 31999;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1024;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

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
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint32_t ADC_Result(ADC_HandleTypeDef *hadc, uint32_t ch){
       ADC_ChannelConfTypeDef sConfig;
       uint32_t adcResult = 0;

       sConfig.Channel = ch;
       sConfig.Rank = ADC_REGULAR_RANK_1;
       sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
       HAL_ADC_ConfigChannel(hadc, &sConfig);
       HAL_ADC_Start(hadc);
       HAL_ADC_PollForConversion(hadc, 100);
       adcResult = HAL_ADC_GetValue(hadc);
       HAL_ADC_Stop(hadc);
       return adcResult;

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
