/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <ctype.h>

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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for SenzoriTask */
osThreadId_t SenzoriTaskHandle;
const osThreadAttr_t SenzoriTask_attributes = {
  .name = "SenzoriTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for ComTask */
osThreadId_t ComTaskHandle;
const osThreadAttr_t ComTask_attributes = {
  .name = "ComTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for MsgQueue */
osMessageQueueId_t MsgQueueHandle;
const osMessageQueueAttr_t MsgQueue_attributes = {
  .name = "MsgQueue"
};
/* Definitions for DataMutex */
osMutexId_t DataMutexHandle;
const osMutexAttr_t DataMutex_attributes = {
  .name = "DataMutex"
};
/* Definitions for UARTMutex */
osMutexId_t UARTMutexHandle;
const osMutexAttr_t UARTMutex_attributes = {
  .name = "UARTMutex"
};

osMutexId_t LogicMutexHandle;
const osMutexAttr_t LogicMutex_attributes = {
  .name = "LogicMutex"
};

/* Definitions for DataReadySem */
osSemaphoreId_t DataReadySemHandle;
const osSemaphoreAttr_t DataReadySem_attributes = {
  .name = "DataReadySem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartSenzoriTask(void *argument);
void StartComTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 0 */

#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_7

uint8_t RHI, RHD, TCI, TCD, SUM;   // 1 BYTE x 5 = 5 BYTES
uint32_t pMillis, cMillis;  // 4 BYTES x 2 = 8 BYTES
float tCelsius = 0;  //4B
float tFahrenheit = 0; //4B
float RH = 0; //4B

void microDelay(uint16_t delay) {
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}


uint8_t DHT11_Start(void) {
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
  microDelay(30);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

  microDelay(40);
  if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
    microDelay(80);
    if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
  }

  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read(void) {
  uint8_t a,b=0;
  for (a=0;a<8;a++) {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
      cMillis = HAL_GetTick();
    }
    microDelay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));

    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
      cMillis = HAL_GetTick();
    }
  }
  return b;
}


void UART_SendAll(char *msg) {
    osMutexAcquire(UARTMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osMutexRelease(UARTMutexHandle);
}

void UART_SendBT(const char *msg) {
    osMutexAcquire(UARTMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osMutexRelease(UARTMutexHandle);
}

void UART_SendUSB(const char *msg) {
  osMutexAcquire(UARTMutexHandle, osWaitForever);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  osMutexRelease(UARTMutexHandle);
}

uint8_t Read_WaterSensor(void)
{
    return HAL_GPIO_ReadPin(WATER_SENSOR_GPIO_Port, WATER_SENSOR_Pin);
    // 0 = apa, 1 = nu apa
}


// ---------------- PROFILING ----------------

// Start time in microseconds
static inline uint16_t prof_start(void) {
    return __HAL_TIM_GET_COUNTER(&htim1);
}

// End time in microseconds
static inline uint16_t prof_end(uint16_t t0) {
    uint16_t t1 = __HAL_TIM_GET_COUNTER(&htim1);
    return (uint16_t)(t1 - t0);
}

// Debug
void debug_us(const char *label, uint32_t v)
{
    osMutexAcquire(UARTMutexHandle, osWaitForever);

    HAL_UART_Transmit(&huart2, (uint8_t*)label, strlen(label), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)": ", 2, HAL_MAX_DELAY);

    char buf[12];
    int i = 0;

    if (v == 0) buf[i++] = '0';
    else {
        while (v > 0 && i < sizeof(buf)-1) {
            buf[i++] = '0' + (v % 10);
            v /= 10;
        }
        for(int j = 0; j < i/2; j++){
            char t = buf[j];
            buf[j] = buf[i-1-j];
            buf[i-1-j] = t;
        }
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, i, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)" us\r\n", 5, HAL_MAX_DELAY);

    osMutexRelease(UARTMutexHandle);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of DataMutex */
  DataMutexHandle = osMutexNew(&DataMutex_attributes);

  /* creation of UARTMutex */
  UARTMutexHandle = osMutexNew(&UARTMutex_attributes);

  LogicMutexHandle = osMutexNew(&LogicMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of DataReadySem */
  DataReadySemHandle = osSemaphoreNew(1, 1, &DataReadySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MsgQueue */
  MsgQueueHandle = osMessageQueueNew (5, 100, &MsgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SenzoriTask */
  SenzoriTaskHandle = osThreadNew(StartSenzoriTask, NULL, &SenzoriTask_attributes);

  /* creation of ComTask */
  ComTaskHandle = osThreadNew(StartComTask, NULL, &ComTask_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_19CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_19CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 63;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|DHT11_Pin_Pin|LEDGREEN_Pin|LEDYELLOW_Pin
                            |MOTOR_BI_Pin, GPIO_PIN_RESET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_FI_Pin|LDRED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_Pin_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pins : LEDGREEN_Pin LEDYELLOW_Pin */
  GPIO_InitStruct.Pin = LEDGREEN_Pin|LEDYELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SOIL_SENSOR_D0_Pin */
  GPIO_InitStruct.Pin = SOIL_SENSOR_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SOIL_SENSOR_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_FI_Pin */
  GPIO_InitStruct.Pin = MOTOR_FI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_FI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LDRED_Pin */
  GPIO_InitStruct.Pin = LDRED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LDRED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USART1 GPIO Configuration
     PC4 -> USART1_TX
     PC5 -> USART1_RX
  */

  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART1;   // AF1 = USART1 pe STM32G071
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


uint16_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return val;
}


typedef struct {
    float temp;
    float hum;
    uint16_t soil;
    uint16_t water;
    uint8_t water_ok;   // 0 = NU apa, 1 = DA apa
} SensorData_t;

SensorData_t data;

typedef struct {
    float last_temp;
    float last_hum;
    uint8_t pump_on;
    uint32_t pump_end_time;
    uint8_t led_green_state;
    uint8_t auto_mode;
    uint32_t last_water_time;
} SystemState_t;

SystemState_t sys = {0};

#define AUTO_WATER_INTERVAL  5000//10800000 // 3 ore in ms (3*60*60*1000)
#define SOIL_DRY_THRESHOLD   2400
#define SOIL_WET_THRESHOLD   1800
#define SOIL_NO_SENSOR       3800

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSenzoriTask */
/**
  * @brief  Function implementing the SenzoriTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSenzoriTask */
void StartSenzoriTask(void *argument)
{
    HAL_TIM_Base_Start(&htim1);

    for(;;)
    {
       //uint16_t T_task_start = prof_start();   // <-- profilare totala task

        SensorData_t local;

        /* ----------------- DHT11 ----------------- */
        //uint16_t T_dht_start = prof_start();

        if (DHT11_Start()) {
            uint8_t hi = DHT11_Read();
            uint8_t hd = DHT11_Read();
            uint8_t ti = DHT11_Read();
            uint8_t td = DHT11_Read();
            uint8_t cs = DHT11_Read();

            if((uint8_t)(hi + hd + ti + td) == cs)
            {
                osMutexAcquire(LogicMutexHandle, osWaitForever);
                sys.last_hum  = hi + hd/10.0f;
                sys.last_temp = ti + td/10.0f;
                osMutexRelease(LogicMutexHandle);
            }
        }

       //debug_us("DHT11 read", prof_end(T_dht_start));


        /* ----------------- copiez structura locala ----------------- */
        osMutexAcquire(LogicMutexHandle, osWaitForever);
        local.temp = sys.last_temp;
        local.hum  = sys.last_hum;
        osMutexRelease(LogicMutexHandle);


        /* ----------------- ADC ----------------- */
      // uint16_t T_adc_start = prof_start();

        local.soil  = Read_ADC_Channel(ADC_CHANNEL_0);
        local.water = Read_ADC_Channel(ADC_CHANNEL_1);
        local.water_ok = (local.water > 600);

       //debug_us("ADC read", prof_end(T_adc_start));


        /* ----------------- scriere globala ----------------- */
        osMutexAcquire(DataMutexHandle, osWaitForever);
        data = local;
        osMutexRelease(DataMutexHandle);

        osSemaphoreRelease(DataReadySemHandle);


        /* -----------------control pompa + AUTO ----------------- */
        osMutexAcquire(LogicMutexHandle, osWaitForever);

        // manual ON/OFF
        if(sys.pump_on)
        {
            HAL_GPIO_WritePin(MOTOR_FI_GPIO_Port, MOTOR_FI_Pin, GPIO_PIN_SET);
            if(HAL_GetTick() >= sys.pump_end_time)
            {
                sys.pump_on = 0;
                HAL_GPIO_WritePin(MOTOR_FI_GPIO_Port, MOTOR_FI_Pin, GPIO_PIN_RESET);
            }
        }
        else
        {
            HAL_GPIO_WritePin(MOTOR_FI_GPIO_Port, MOTOR_FI_Pin, GPIO_PIN_RESET);
        }

        /* ----------------- AUTO mode ----------------- */
        //uint16_t T_auto_start = prof_start();

        if(sys.auto_mode && local.water_ok &&
           local.soil > SOIL_DRY_THRESHOLD &&
           local.soil < SOIL_NO_SENSOR)
        {
            uint32_t now = HAL_GetTick();
            if(now - sys.last_water_time > AUTO_WATER_INTERVAL)
            {
                sys.pump_on = 1;
                sys.pump_end_time = HAL_GetTick() + 3000;
                sys.last_water_time = now;
            }
        }

        //debug_us("AUTO logic", prof_end(T_auto_start));

        osMutexRelease(LogicMutexHandle);


        /* ----------------- LED-uri ----------------- */
        if (!local.water_ok)
        {
            HAL_GPIO_WritePin(LDRED_GPIO_Port, LDRED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LEDYELLOW_GPIO_Port, LEDYELLOW_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(LDRED_GPIO_Port, LDRED_Pin, GPIO_PIN_RESET);

            if (local.soil > SOIL_DRY_THRESHOLD)
            {
                HAL_GPIO_WritePin(LEDYELLOW_GPIO_Port, LEDYELLOW_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(LEDYELLOW_GPIO_Port, LEDYELLOW_Pin, GPIO_PIN_RESET);

                osMutexAcquire(LogicMutexHandle, osWaitForever);
                sys.led_green_state ^= 1;
                osMutexRelease(LogicMutexHandle);

                HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin,
                                  sys.led_green_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
        }

        /* ----------------- timp total executie ----------------- */
        //debug_us("SenzoriTask total", prof_end(T_task_start));

        osDelay(2000);
    }
}






/* USER CODE BEGIN Header_StartComTask */
/**
* @brief Function implementing the ComTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComTask */
void StartComTask(void *argument)
{
    char c;
    char cmd[20];
    uint8_t idx = 0;
    char msg[64];

    UART_SendBT("\r\n--- Sistem Udare Inteligenta ---\r\n");
    UART_SendBT("Comenzi:\r\n");
    UART_SendBT("  ON        - Porneste pompa 3 sec (daca exista apa)\r\n");
    UART_SendBT("  OFF       - Opreste pompa imediat\r\n");
    UART_SendBT("  STATUS    - Afiseaza valorile senzorilor\r\n");
    UART_SendBT("  AUTO=ON   - Porneste udarea automata\r\n");
    UART_SendBT("  AUTO=OFF  - Opreste udarea automata\r\n");
    UART_SendBT("---------------------------------\r\n\r\n");

    for(;;)
    {
        if(HAL_UART_Receive(&huart1, (uint8_t*)&c, 1, 10) == HAL_OK)
        {
            if(c == '\r' || c == '\n')
            {
                cmd[idx] = 0;
                idx = 0;
                if(cmd[0] == 0) continue;

                for(char *p = cmd; *p; p++) *p = toupper(*p);

                /* ---------------- ON ---------------- */
                if(strcmp(cmd, "ON") == 0)
                {
                 //  uint16_t t0 = prof_start();

                    SensorData_t d;
                    osMutexAcquire(DataMutexHandle, osWaitForever);
                    d = data;
                    osMutexRelease(DataMutexHandle);

                    if(!d.water_ok)
                    {
                        UART_SendBT("NU EXISTA APA!\r\n");
                    }
                    else
                    {
                        osMutexAcquire(LogicMutexHandle, osWaitForever);
                        sys.pump_on = 1;
                        sys.pump_end_time = HAL_GetTick() + 3000;
                        osMutexRelease(LogicMutexHandle);

                        UART_SendBT("Pompa pornita.\r\n");
                    }

                 //  debug_us("ON latency", prof_end(t0));
                    continue;
                }

                /* ---------------- OFF ---------------- */
                if(strcmp(cmd, "OFF") == 0)
                {
                   // uint16_t t0 = prof_start();

                    osMutexAcquire(LogicMutexHandle, osWaitForever);
                    sys.pump_on = 0;
                    sys.last_water_time = HAL_GetTick();
                    osMutexRelease(LogicMutexHandle);

                    HAL_GPIO_WritePin(MOTOR_FI_GPIO_Port, MOTOR_FI_Pin, GPIO_PIN_RESET);

                    UART_SendBT("Pompa oprita.\r\n");

                  //  debug_us("OFF latency", prof_end(t0));
                    continue;
                }

                /* ---------------- STATUS ---------------- */
                if(strcmp(cmd, "STATUS") == 0)
                {
                    //uint16_t t0 = prof_start();

                    osSemaphoreAcquire(DataReadySemHandle, osWaitForever);

                    SensorData_t d;
                    osMutexAcquire(DataMutexHandle, osWaitForever);
                    d = data;
                    osMutexRelease(DataMutexHandle);

                    char *soil_state;
                    if (d.soil > SOIL_NO_SENSOR) soil_state = "SCOS / AER";
                    else if (d.soil > SOIL_DRY_THRESHOLD) soil_state = "USCAT";
                    else if (d.soil > SOIL_WET_THRESHOLD) soil_state = "UMED";
                    else soil_state = "FOARTE UMED";

                    // mesaj mic, local transmis o singura data
                    sprintf(msg,
                        "Temp=%.1fC  Hum=%.1f%%\r\nSol=%u (%s)\r\nApa=%s\r\nAUTO=%s\r\n\r\n",
                        d.temp, d.hum, d.soil, soil_state,
                        d.water_ok ? "DA" : "NU",
                        sys.auto_mode ? "ON" : "OFF"
                    );

                    UART_SendBT(msg);

                   // debug_us("STATUS latency", prof_end(t0));
                    continue;
                }

                /* ---------------- AUTO ON ---------------- */
                if(strcmp(cmd, "AUTO=ON") == 0)
                {
                   // uint16_t t0 = prof_start();

                    osMutexAcquire(LogicMutexHandle, osWaitForever);
                    sys.auto_mode = 1;
                    osMutexRelease(LogicMutexHandle);

                    UART_SendBT("Mod automat ACTIV.\r\n");

                   // debug_us("AUTO=ON latency", prof_end(t0));
                    continue;
                }

                /* ---------------- AUTO OFF ---------------- */
                if(strcmp(cmd, "AUTO=OFF") == 0)
                {
                   // uint16_t t0 = prof_start();

                    osMutexAcquire(LogicMutexHandle, osWaitForever);
                    sys.auto_mode = 0;
                    osMutexRelease(LogicMutexHandle);

                    UART_SendBT("Mod automat OPRIT.\r\n");

                   // debug_us("AUTO=OFF latency", prof_end(t0));
                    continue;
                }

                /* ---------------- NECUNOSCUT ---------------- */
                UART_SendBT("Comanda necunoscuta!\r\n");
            }
            else if(idx < sizeof(cmd)-1)
                cmd[idx++] = c;
        }
    }
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
#ifdef USE_FULL_ASSERT
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
