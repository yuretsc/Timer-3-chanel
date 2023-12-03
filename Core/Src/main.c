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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
DMA_HandleTypeDef hdma_adc1;

/* Definitions for Polling_main */
osThreadId_t Polling_mainHandle;
const osThreadAttr_t Polling_main_attributes = {
  .name = "Polling_main",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Polling_DI */
osThreadId_t Polling_DIHandle;
const osThreadAttr_t Polling_DI_attributes = {
  .name = "Polling_DI",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myTimer_OFF */
osTimerId_t myTimer_OFFHandle;
const osTimerAttr_t myTimer_OFF_attributes = {
  .name = "myTimer_OFF"
};
/* Definitions for myTimer_ON */
osTimerId_t myTimer_ONHandle;
const osTimerAttr_t myTimer_ON_attributes = {
  .name = "myTimer_ON"
};
/* USER CODE BEGIN PV */
input_DataType DI_str; // структура для обработчика кнопок
uint16_t adc[40]; //массив для хранения опросов АЦП
uint8_t Time_on_relay; //время включения реле в сек
uint8_t Time_off_relay; //время включения реле в сек
uint16_t* VREFINT_CAL = (uint16_t*)(0x1FFF75AA);
uint8_t relays_sequence = 1; // очередь включения реле
uint8_t once = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StartTaskPolling_main(void *argument);
void StartTaskPolling_DI(void *argument);
void Callback01_OFF(void *argument);
void Callback_ON(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// медиана на 3 значения со своим буфером
uint32_t median(uint32_t newVal) {
  static int buf[3];
  static uint8_t count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}

int32_t filter(int32_t in, uint32_t coeff_A, uint32_t coeff_k ) {
	static uint32_t filt;

filt = (coeff_A * filt + in) >> coeff_k;
return filt;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer_OFF */
  myTimer_OFFHandle = osTimerNew(Callback01_OFF, osTimerOnce, NULL, &myTimer_OFF_attributes);

  /* creation of myTimer_ON */
  myTimer_ONHandle = osTimerNew(Callback_ON, osTimerOnce, NULL, &myTimer_ON_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Polling_main */
  Polling_mainHandle = osThreadNew(StartTaskPolling_main, NULL, &Polling_main_attributes);

  /* creation of Polling_DI */
  Polling_DIHandle = osThreadNew(StartTaskPolling_DI, NULL, &Polling_DI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 40);
	HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, 1);
	HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, 1);
	HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, 1);
	HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, 1);
	HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, 1);
	HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, 1);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay_1_Pin|Relay_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_1_Pin|Led_2_Pin|Led_3_Pin|Relay_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Relay_1_Pin Relay_2_Pin */
  GPIO_InitStruct.Pin = Relay_1_Pin|Relay_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Input_1_Pin Input_2_Pin Input_3_Pin */
  GPIO_InitStruct.Pin = Input_1_Pin|Input_2_Pin|Input_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_1_Pin Led_2_Pin Led_3_Pin Relay_3_Pin */
  GPIO_InitStruct.Pin = Led_1_Pin|Led_2_Pin|Led_3_Pin|Relay_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskPolling_main */
/**
  * @brief  Function implementing the Polling_main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskPolling_main */
void StartTaskPolling_main(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  	uint16_t Uvdda; //  измерение напряжения питания АЦП
		for (uint8_t i = 3; i<40; i+=4) // берем сумму каждых четных адресов
		  	{
				Uvdda = Uvdda + adc[i];
		  	}
		Uvdda = (uint16_t)(30000*(uint32_t)(*VREFINT_CAL)/(uint32_t)Uvdda); //напряжение в мВ на пине VDDA
		Uvdda = median(Uvdda);

		int32_t Input_AI1=0;
		for (uint8_t i = 0; i<40; i+=4) // берем сумму каждых первых адресов
			{
				Input_AI1 = Input_AI1 + adc[i];
			}

		int32_t Input_AI2=0;
		for (uint8_t i = 1; i<40; i+=4) // берем сумму каждых первых адресов
			{
				Input_AI2 = Input_AI2 + adc[i];
			}

		int32_t Input_AI3=0;
		for (uint8_t i = 2; i<40; i+=4) // берем сумму каждых первых адресов
			{
				Input_AI3 = Input_AI3 + adc[i];
			}

		Time_on_relay = (uint8_t)(Input_AI1*9/40950) + 1 ; // задание от 1 до 10 сек
		Time_off_relay = (uint8_t)(Input_AI2*99/40950) + 1; // задание от 1 до 100 сек


		if (once)
		{
			once=0;
			osTimerStart(myTimer_OFFHandle, (uint32_t)(Time_off_relay)*1000);
		}
		// выключение выходов, если нет сигнала
		if ((GPIO_PinState)!DI_str.input_1)
		{
			HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, 1);
			HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, 1);
		}

		if ((GPIO_PinState)!DI_str.input_2)
		{
			HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, 1);
			HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, 1);
		}

		if ((GPIO_PinState)!DI_str.input_3)
		{
			HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, 1);
			HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, 1);
		}

	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskPolling_DI */
/**
  * @brief  Function implementing the Polling_AI thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskPolling_DI */
void StartTaskPolling_DI(void *argument)
{
  /* USER CODE BEGIN StartTaskPolling_DI */
  /* Infinite loop */
  for(;;)
  {


	  	DI_str.input_1=(uint32_t)HAL_GPIO_ReadPin(Input_1_GPIO_Port, Input_1_Pin);
	    DI_str.input_2=(uint32_t)HAL_GPIO_ReadPin(Input_2_GPIO_Port, Input_2_Pin);
	    DI_str.input_3=(uint32_t)HAL_GPIO_ReadPin(Input_3_GPIO_Port, Input_3_Pin);

	  	          if (DI_str.input_2_rise == 1)
	  	              DI_str.input_2_rise = 0; // сброс статуса фронта нажатия
	  	          if (DI_str.input_2_fall == 1)
	  	              DI_str.input_2_fall = 0; // сброс статуса спада нажатия
	  	          // начало события, отсчет от первого фронта
	  	          if (DI_str.input_2 && DI_str.input_2_time == 0)
	  	          {
	  	              DI_str.input_2_time = xTaskGetTickCount();
	  	              //DI_str.input_2 = DI_str.input_2;
	  	          }
	  	          // проверка состояния контакта через время дребезга контактов 50мс
	  	          else if (DI_str.input_2 && xTaskGetTickCount() > DI_str.input_2_time + 50)
	  	          {
	  	              DI_str.input_2 = xTaskGetTickCount() - DI_str.input_2_time; // сохраняем время нажатой кнопки в мс*10
	  	              DI_str.input_2_rise = 1;                                             // формирование статуса фронт сигнала
	  	          }
	  	          else if (DI_str.input_2 == 0 && DI_str.input_2 != 0)
	  	          {
	  	              DI_str.input_2_time = 0;
	  	              DI_str.input_2 = 0;
	  	              DI_str.input_2_fall = 1;
	  	          }

	  	          if (DI_str.input_1_rise == 1)
	  	              DI_str.input_1_rise = 0; // сброс статуса фронта нажатия
	  	          if (DI_str.input_1_fall == 1)
	  	              DI_str.input_1_fall = 0; // сброс статуса спада нажатия
	  	          // начало события, отсчет от первого фронта
	  	          if (DI_str.input_1 && DI_str.input_1_time == 0)
	  	          {
	  	              DI_str.input_1_time = xTaskGetTickCount();
	  	             // DI_str.input_1 = DI_str.input_1;
	  	          }
	  	          // проверка состояния контакта через время дребезга контактов 50мс
	  	          else if (DI_str.input_1 && xTaskGetTickCount() > DI_str.input_1_time + 50)
	  	          {
	  	              DI_str.input_1 = xTaskGetTickCount() - DI_str.input_1_time; // сохраняем время нажатой кнопки в мс*10
	  	              DI_str.input_1_rise = 1;                                           // формирование статуса фронт сигнала
	  	          }
	  	          else if (DI_str.input_1 == 0 && DI_str.input_1 != 0)
	  	          {
	  	              DI_str.input_1_time = 0;
	  	              DI_str.input_1 = 0;
	  	              DI_str.input_1_fall = 1;
	  	          }

	  	          if (DI_str.input_3_rise == 1)
	  	              DI_str.input_3_rise = 0; // сброс статуса фронта нажатия
	  	          if (DI_str.input_3_fall == 1)
	  	              DI_str.input_3_fall = 0; // сброс статуса спада нажатия
	  	          // начало события, отсчет от первого фронта
	  	          if (DI_str.input_3 && DI_str.input_3_time == 0)
	  	          {
	  	              DI_str.input_3_time = xTaskGetTickCount();
	  	              //DI_str.input_3 = DI_str.input_3;
	  	          }
	  	          // проверка состояния контакта через время дребезга контактов 50мс
	  	          else if (DI_str.input_3 && xTaskGetTickCount() > DI_str.input_3_time + 50)
	  	          {
	  	              DI_str.input_3 = xTaskGetTickCount() - DI_str.input_3_time; // сохраняем время нажатой кнопки в мс*10
	  	              DI_str.input_3_rise = 1;                                             // формирование статуса фронт сигнала
	  	          }
	  	          else if (DI_str.input_3 == 0 && DI_str.input_3 != 0)
	  	          {
	  	              DI_str.input_3_time = 0;
	  	              DI_str.input_3 = 0;
	  	              DI_str.input_3_fall = 1;
	  	          }

	  	          DI_str.any_input = DI_str.input_1 | DI_str.input_2 | DI_str.input_3 ;


	      osDelay(100);

  }
  /* USER CODE END StartTaskPolling_DI */
}

/* Callback01_OFF function */
void Callback01_OFF(void *argument)
{
  /* USER CODE BEGIN Callback01_OFF */
	  if (relays_sequence==1 && (GPIO_PinState)DI_str.input_1)
	  {
		  HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, 0);
		  HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, 0);
	  }

	  else if (relays_sequence==2 && (GPIO_PinState)DI_str.input_2)
	  {
		  HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, 0);
		  HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, 0);
	  }

	  else if (relays_sequence==3 && (GPIO_PinState)DI_str.input_3)
	  {
		  HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, 0);
		  HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, 0);
	  }

	  osTimerStart(myTimer_ONHandle, (uint32_t)(Time_on_relay)*1000);
  /* USER CODE END Callback01_OFF */
}

/* Callback_ON function */
void Callback_ON(void *argument)
{
  /* USER CODE BEGIN Callback_ON */
	  if (relays_sequence==1 && (GPIO_PinState)DI_str.input_1)
	  {
		  HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, 1);
		  HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, 1);
	  }

	  else if (relays_sequence==2 && (GPIO_PinState)DI_str.input_2)
	  {
		  HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, 1);
		  HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, 1);
	  }

	  else if (relays_sequence==3 && (GPIO_PinState)DI_str.input_3)
	  {
		  HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, 1);
		  HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, 1);
	  }

	  uint8_t select=0;
	  select |= (DI_str.input_1 !=0)  << 0;
	  select |= (DI_str.input_2 !=0)  << 1;
	  select |= (DI_str.input_3 !=0)  << 2;

	  switch (select)
	  {
	  	  case 0:
	  		relays_sequence=1;
	  	  break;

	  	  case 1:
	  		relays_sequence=1;
	  		break;

	  	  case 2:
	  		relays_sequence=2;
	  		break;

	  	  case 3:
		  		if (relays_sequence==1) relays_sequence=2;
		  		else if (relays_sequence==2) relays_sequence=1;
	  		break;

	  	  case 4:
	  		relays_sequence=3;
	  		break;

	  	  case 5:
		  		if (relays_sequence==1) relays_sequence=3;
		  		else if (relays_sequence==3) relays_sequence=1;
	  		break;

	  	  case 6:
		  		if (relays_sequence==3) relays_sequence=2;
		  		else if (relays_sequence==2) relays_sequence=3;
	  		break;

	  	  case 7: //если все три реле разрешены

	  		if (relays_sequence==1) relays_sequence=2;
	  		else if (relays_sequence==2) relays_sequence=3;
	  		else if (relays_sequence==3) relays_sequence=1;
	  		break;

	  	  default:
	  		relays_sequence=1;
	  		break;
	  }




	  osTimerStart(myTimer_OFFHandle, (uint32_t)(Time_off_relay)*1000);
  /* USER CODE END Callback_ON */
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
