/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "retarget.h" //printf for Uart
#include "timer_pwm_lib.h"
#include "temp_lookup_table.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THERMISTOR_MODULE_NUMBER 			0
#define HIGHEST_ID 							5
#define LOWEST_ID 							0
#define NUM_THERMISTORS						6
#define NUM_CAN_MESSAGES 					4
#define CHECKSUM_DECODED_CONSTANT 			0x41

#define DUTY_CYCLE_START 					0
#define DUTY_CYCLE_STEP 					1
#define DUTY_CYCLE_IS_POSITIVE_LOGIC 		0
#define DUTY_CYCLE_MAX 						100
#define DUTY_CYCLE_MIN 						0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//CAN
CAN_TxHeaderTypeDef   	Tx_BMS_broadcast;
uint32_t              	TxMailbox;
uint8_t 				TxData[8] = {0};

//ADC
uint16_t volatile adc_vals[NUM_ADC_CHANNELS] = {};

//TIMER/PWM
uint8_t duty_cycle = 0;
uint8_t start_fan_pwm = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void CAN_Config(void);
int8_t getMin(int8_t* data, uint8_t n, int8_t *minID);
int8_t getMax(int8_t* data, uint8_t n, int8_t *maxID);
int8_t getAvg(int8_t* data, uint8_t n);

void Get_ADC_Scan(ADC_HandleTypeDef *hadc, uint8_t* channel_arr, uint8_t num_conversions, uint16_t* scan_output);
uint16_t Get_ADC_Value(ADC_HandleTypeDef *hadc, uint8_t channel);
void ADC_Channel_Config(ADC_HandleTypeDef *hadc, uint8_t channel);
void ADC_Channel_Select(uint8_t channel, ADC_ChannelConfTypeDef* ptr_sConfig);

uint8_t PWM_Sweep_Nonblocking(uint8_t timer, uint8_t channel, uint8_t duty_target, uint8_t duty_step_size, uint8_t is_positive_logic, uint8_t initialize, uint8_t duty_init);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) //ISR triggered by timer overflow
{
	static int8_t temp_vals[NUM_ADC_CHANNELS] = {};
	static uint16_t mean_adc_vals[NUM_ADC_CHANNELS] = {};
	static uint8_t channel_ranks[NUM_ADC_CHANNELS] = {10,11,12,13,14,15};
	static int8_t min_temp;
	static int8_t max_temp;
	static int8_t min_ID;
	static int8_t max_ID;
	static int8_t avg_temp;
	static uint8_t new_duty_cycle;

	if(htim == &htim4 && timer_4_repetition_counter == TIMER_4_PERIOD_MULTIPLIER-1)
    {
		//ISR for Timer 4
		timer_4_repetition_counter = 0;
        //User code here

		/*-------------------------GET ADC VALUES-------------------------*/
		Get_ADC_Scan(&hadc1, channel_ranks, NUM_ADC_CHANNELS, adc_vals);

		/*-------------------------DATA PROCESSING-------------------------*/
		getRollingAvg(adc_vals, mean_adc_vals);
		getTempVals(mean_adc_vals, temp_vals, NUM_ADC_CHANNELS);
		//getTempVals(adc_vals, temp_vals, NUM_ADC_CHANNELS);

		min_temp = getMin(temp_vals, NUM_ADC_CHANNELS, &min_ID);
		max_temp = getMax(temp_vals, NUM_ADC_CHANNELS, &max_ID);
		avg_temp = getAvg(temp_vals, NUM_ADC_CHANNELS);

		/*-------------------------CAN TRANSMISSION-------------------------*/
		//TMS->BMS Broadcast
		TxData[0] = THERMISTOR_MODULE_NUMBER;
		TxData[1] = min_temp;
		TxData[2] = max_temp;
		TxData[3] = avg_temp;
		TxData[4] = NUM_THERMISTORS;
		TxData[5] = max_ID;
		TxData[6] = min_ID;
		TxData[7] = CHECKSUM_DECODED_CONSTANT + (THERMISTOR_MODULE_NUMBER + (uint8_t)min_temp + (uint8_t)max_temp+ (uint8_t)avg_temp + NUM_THERMISTORS + max_ID + min_ID);

		HAL_CAN_AddTxMessage(&hcan1, &Tx_BMS_broadcast, TxData, &TxMailbox);

		/*-------------------------FAN PWM-------------------------*/
		if (start_fan_pwm)
		{
			new_duty_cycle = getDutyCycle(avg_temp);
			PWM_Sweep_Nonblocking(4, 1, new_duty_cycle, DUTY_CYCLE_STEP, DUTY_CYCLE_IS_POSITIVE_LOGIC, 0, DUTY_CYCLE_START);
		}

        //User code ends
    }
    else if(htim == &htim4)
    {
    	timer_4_repetition_counter++;
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
  MX_ADC1_Init();
  MX_CAN1_Init();
//  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //CAN Initialization
  CAN_Config();
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	/* Notification Error */
	Error_Handler();
  }

  //Themistor Module -> BMS Broadcast
  Tx_BMS_broadcast.StdId = 0x01;
  Tx_BMS_broadcast.ExtId = 0x1839F380;
  Tx_BMS_broadcast.RTR = CAN_RTR_DATA;
  Tx_BMS_broadcast.IDE = CAN_ID_EXT;
  Tx_BMS_broadcast.DLC = 8;
  Tx_BMS_broadcast.TransmitGlobalTime = DISABLE;

  Timer_Init_Base(4, 1, 1, 1, -1, -1, -1, 80);
  PWM_Sweep_Nonblocking(4, 1, DUTY_CYCLE_START, DUTY_CYCLE_STEP, DUTY_CYCLE_IS_POSITIVE_LOGIC, 1, DUTY_CYCLE_START);
  start_fan_pwm = 1;
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_10;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_11;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_12;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_13;
//  sConfig.Rank = ADC_REGULAR_RANK_4;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_14;
//  sConfig.Rank = ADC_REGULAR_RANK_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_15;
//  sConfig.Rank = ADC_REGULAR_RANK_6;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUG_LED_RED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_LED_GREEN_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void CAN_Config(void)
{
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
	  }

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	    /* Start Error */
	    Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
}

void Get_ADC_Scan(ADC_HandleTypeDef *hadc, uint8_t* channel_arr, uint8_t num_conversions, uint16_t* scan_output)
{
	for (int i = 0; i<num_conversions; i++)
	{
		scan_output[i] = Get_ADC_Value(hadc, channel_arr[i]);
	}
}

uint16_t Get_ADC_Value(ADC_HandleTypeDef *hadc, uint8_t channel)
{
	ADC_Channel_Config(hadc, channel);

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	uint16_t output_value = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	return output_value;
}

void ADC_Channel_Config(ADC_HandleTypeDef *hadc, uint8_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	ADC_Channel_Select(channel, &sConfig);
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;

	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
	  Error_Handler();
    }
}

void ADC_Channel_Select(uint8_t channel, ADC_ChannelConfTypeDef* ptr_sConfig)
{
	switch(channel)
	{
		case(0):
			ptr_sConfig->Channel = ADC_CHANNEL_0;
			return;
		case(1):
			ptr_sConfig->Channel = ADC_CHANNEL_1;
			return;
		case(2):
			ptr_sConfig->Channel = ADC_CHANNEL_2;
			return;
		case(3):
			ptr_sConfig->Channel = ADC_CHANNEL_3;
			return;
		case(4):
			ptr_sConfig->Channel = ADC_CHANNEL_4;
			return;
		case(5):
			ptr_sConfig->Channel = ADC_CHANNEL_5;
			return;
		case(6):
			ptr_sConfig->Channel = ADC_CHANNEL_6;
			return;
		case(7):
			ptr_sConfig->Channel = ADC_CHANNEL_7;
			return;
		case(8):
			ptr_sConfig->Channel = ADC_CHANNEL_8;
			return;
		case(9):
			ptr_sConfig->Channel = ADC_CHANNEL_9;
			return;
		case(10):
			ptr_sConfig->Channel = ADC_CHANNEL_10;
			return;
		case(11):
			ptr_sConfig->Channel = ADC_CHANNEL_11;
			return;
		case(12):
			ptr_sConfig->Channel = ADC_CHANNEL_12;
			return;
		case(13):
			ptr_sConfig->Channel = ADC_CHANNEL_13;
			return;
		case(14):
			ptr_sConfig->Channel = ADC_CHANNEL_14;
			return;
		case(15):
			ptr_sConfig->Channel = ADC_CHANNEL_15;
			return;
		case(16):
			ptr_sConfig->Channel = ADC_CHANNEL_16;
			return;
		case(17):
			ptr_sConfig->Channel = ADC_CHANNEL_17;
			return;
		case(18):
			ptr_sConfig->Channel = ADC_CHANNEL_18;
			return;
	}
}


uint8_t PWM_Sweep_Nonblocking(uint8_t timer, uint8_t channel, uint8_t duty_target, uint8_t duty_step_size, uint8_t is_positive_logic, uint8_t initialize, uint8_t duty_init)
{
	static current_duty = 0;

	if (initialize==1)
	{
		//check for valid arguments
		if(!(timer==1 || timer == 2 || timer == 4) ) 												{return 1;}
		else if( !( (channel>=1) && (channel<=4) ) ) 												{return 1;}
		else if( !( (duty_init >= DUTY_CYCLE_MIN) && (duty_init <= DUTY_CYCLE_MAX) ) ) 				{return 1;}
		else if( !( (duty_target >= DUTY_CYCLE_MIN) && (duty_target <= DUTY_CYCLE_MAX) ) ) 			{return 1;}
		else if( !( (duty_step_size >= DUTY_CYCLE_MIN) && ( duty_step_size <= DUTY_CYCLE_MAX ) ) ) 	{return 1;}
		else if( !( (duty_init >= DUTY_CYCLE_MIN) && ( duty_init <= DUTY_CYCLE_MAX ) ) ) 			{return 1;}
		else if( !( (is_positive_logic == 0) || (is_positive_logic == 1) ) )						{return 1;}

		current_duty = duty_init;
	}

	if( current_duty == duty_target )
	{
		return 0;
	}
	else if( current_duty > duty_target )
	{
		//make sure duty does not go below min
		current_duty = ( (current_duty - duty_step_size) >= DUTY_CYCLE_MIN ) ? (current_duty - duty_step_size) : DUTY_CYCLE_MIN;
	}
	else if( current_duty < duty_target )
	{
		//make sure duty does not go above max
		current_duty = ( (current_duty + duty_step_size) <= DUTY_CYCLE_MAX ) ? (current_duty + duty_step_size) : DUTY_CYCLE_MAX;
	}

	PWM_Stop(timer, channel);

	if (is_positive_logic)
	{
		PWM_Init(timer, channel, current_duty);
	}
	else
	{
		PWM_Init(timer, channel, DUTY_CYCLE_MAX - current_duty);
	}

	return 0;
}


int8_t getMin(int8_t* data, uint8_t n, int8_t *minID)
{
	int8_t min = data[0];
	*minID = 0;
	for (uint8_t i = 1; i<n; i++)
	{
		if (data[i]<min)
		{
			min = data[i];
			*minID = i;
		}
	}
	return min;
}

int8_t getMax(int8_t* data, uint8_t n, int8_t *maxID)
{
	int8_t max = data[0];
	*maxID = 0;

	for (uint8_t i = 1; i<n; i++)
	{
		if (data[i]>max)
		{
			max = data[i];
			*maxID = i;
		}
	}
	return max;
}

int8_t getAvg(int8_t* data, uint8_t n)
{
	int16_t sum = 0;

	for (uint8_t i = 0; i<n ; i++)
	{
		sum += data[i];
	}

	return (sum / n);
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
