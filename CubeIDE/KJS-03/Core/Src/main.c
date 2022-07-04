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
#include "ics.h"
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

osThreadId ICSTaskHandle;
osThreadId LEDTaskHandle;
osThreadId IMUTaskHandle;
osThreadId PSTaskHandle;
osThreadId ADCTaskHandle;
osThreadId TXBUFFTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
void StartICSTask(void const * argument);
void StartLEDTask(void const * argument);
void StartIMUTask(void const * argument);
void StartPSTask(void const * argument);
void StartADCTask(void const * argument);
void StartTXBUFFTask(void const * argument);

/* USER CODE BEGIN PFP */
void mpuWrite(uint8_t, uint8_t);
void ps_select_channel();
void ps_init();
void imu_init();
void ps_update();
void ps_update_single();
void imu_update();
void adc_update();
void sw_update();
void initialise_monitor_handles(void);
uint32_t UART_IsEnabledIT_RX();
uint8_t getlen();
void ics_init();
void ics_init_DMA();
void txbuff_update();

extern uint32_t current_time;
extern uint32_t receive_time1;
extern uint32_t receive_time2;
extern uint32_t receive_time3;
extern uint32_t getUs(void);
extern uint8_t flagRcved;

// dma buffer
static UART_HandleTypeDef *huart_cobs;
#define RX_DMA_WRITE_IDX ( (RX_CIRC_BUF_SZ - ((DMA_Stream_TypeDef *)huart_cobs->hdmarx->Instance)->NDTR) & (RX_CIRC_BUF_SZ - 1) )
//#define RX_DMA_WRITE_IDX ( (RX_CIRC_BUF_SZ - ((DMA_Stream_TypeDef *)huart6.hdmarx->Instance)->NDTR) & (RX_CIRC_BUF_SZ - 1) )

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//static uint16_t ADC_Data1[ADC_CHANNEL_NUM];
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  sp.id = 19;

  sp.mode = SC_ALL;//initial value, update all sensor

  sp.board_select = SELECT_revA;
  sp.imu_select = SELECT_ICM_20600;

  sp.it_count = 0;
  sp.uart_error_count = 0;
  sp.check2com_delay = 0;
  sp.count_frame = 0;
  sp.prev_frame = getUs();
  sp.com_en = 0;
  sp.ics_timeout_count = 0;

#if USE_SEMIHOSTING
  initialise_monitor_handles();
#endif

	ics_init_DMA(&huart6);

   if (HAL_ADC_Start(&hadc1) !=  HAL_OK)
   {
  	  Error_Handler();
    }
  	//ics_init(&huart6);

    ps_init(&hi2c2);

    imu_init(&hspi1);

    //receive_time1 = receive_time2 = receive_time3 = getUs();

    //receive_time1: past toggle time
    //receive_time2: past uart_rx interrupt time
    //receive_time3: past receive ics command time

    HAL_HalfDuplex_EnableReceiver(&huart6);
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
  /* definition and creation of ICSTask */
  osThreadDef(ICSTask, StartICSTask, osPriorityHigh, 0, 1000);
  ICSTaskHandle = osThreadCreate(osThread(ICSTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityIdle, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, StartIMUTask, osPriorityIdle, 0, 128);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of PSTask */
  osThreadDef(PSTask, StartPSTask, osPriorityIdle, 0, 128);
  PSTaskHandle = osThreadCreate(osThread(PSTask), NULL);

  /* definition and creation of ADCTask */
  osThreadDef(ADCTask, StartADCTask, osPriorityIdle, 0, 128);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

  /* definition and creation of TXBUFFTask */
  osThreadDef(TXBUFFTask, StartTXBUFFTask, osPriorityIdle, 0, 128);
  TXBUFFTaskHandle = osThreadCreate(osThread(TXBUFFTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart6.Init.BaudRate = 1250000;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback_old(UART_HandleTypeDef *huart){
	sp.uart_error[0] = huart6.Lock;
	sp.uart_error[1] = huart6.gState;
	sp.uart_error[2] = huart6.RxState;
	sp.uart_error[3] = huart6.ErrorCode & 0x000000ff;
  //MX_GPIO_Init();
  HAL_UART_Abort(&huart6);
	//start HAL_UART_Abort
	//CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_TCIE));
	//CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
	//end HAL_UART_Abort
  //MX_USART6_UART_Init();
  //CLEAR_BIT(huart->Instance->SR, (USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  //CLEAR_BIT(huart->Instance->SR, (USART_SR_ORE));

  //CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));//minimum reset + RX_It reset
  //CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));//minimum reset
  //huart6.RxState=HAL_UART_STATE_READY;
  //huart6.ErrorCode=HAL_UART_ERROR_NONE;
  //__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  //  MX_I2C2_Init();
  //  ps_init(&hi2c1);
  HAL_HalfDuplex_EnableReceiver(&huart6);
  flagRcved = IT_FLAG_FALSE;

  if(sp.uart_error_count < 4096){
	  sp.uart_error_count += 1;
  }
  //sp.mode = SC_ALL; //initial value, update all sensor
  //NVIC_SystemReset();
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  MX_GPIO_Init();
  MX_I2C2_Init();
  ps_init(&hi2c2);
  //sp.mode = SC_ALL; //initial value, update all sensor
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
  MX_GPIO_Init();
  MX_ADC1_Init();
  if (HAL_ADC_Start(&hadc1) !=  HAL_OK)
    {
      Error_Handler();
    }
  //sp.mode = SC_ALL; //initial value, update all sensor
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
  MX_GPIO_Init();
  MX_SPI1_Init();
  imu_init(&hspi1);
  //sp.mode = SC_ALL; //initial value, update all sensor
}

void ics_init_DMA(UART_HandleTypeDef *huart)
{
    huart_cobs = huart;
    memset(sp.rx_dma_circ_buf, 0, sizeof(sp.rx_dma_circ_buf));

    __HAL_UART_DISABLE_IT(huart_cobs, UART_IT_PE);//parity error interrupt
    __HAL_UART_DISABLE_IT(huart_cobs, UART_IT_ERR);//other error interrupt

    __HAL_UART_DISABLE_IT(huart_cobs, UART_IT_TXE); //txe interrupt
    __HAL_UART_DISABLE_IT(huart_cobs, UART_IT_TC); //transmission complete interrupt

    HAL_HalfDuplex_EnableTransmitter(huart_cobs);
    HAL_UART_Receive_DMA(huart_cobs, sp.rx_dma_circ_buf, RX_CIRC_BUF_SZ);
    memset(sp.txbuff, 0, sizeof(sp.txbuff));
    sp.txbuff[0] = 0x20 | sp.id;
    sp.rd_idx = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	sp.uart_error[0] = huart6.Lock;
	sp.uart_error[1] = huart6.gState;
	sp.uart_error[2] = huart6.RxState;
	sp.uart_error[3] = huart6.ErrorCode & 0x000000ff;
	memset(sp.rx_dma_circ_buf, 0, sizeof(sp.rx_dma_circ_buf));

	HAL_UART_Abort(huart);

	if(sp.uart_error_count < 4096){
		sp.uart_error_count += 1;
	}else{
		sp.uart_error_count = 0;
	}
	//HAL_HalfDuplex_EnableReceiver(&huart6);
	HAL_HalfDuplex_EnableReceiver(huart_cobs);
	HAL_UART_Receive_DMA(huart_cobs, sp.rx_dma_circ_buf, RX_CIRC_BUF_SZ);
	sp.rd_idx = 0;
	//	_Error_Handler("isrSensors.c, uart error",53);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartICSTask */
/**
  * @brief  Function implementing the ICSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartICSTask */
void StartICSTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t sensor_flag;
	uint8_t ret;
	uint8_t tx_length;
  /* Infinite loop */
  for(;;)
  {
	  sp.count_frame = getUs() - sp.prev_frame;
	  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE) ||
			  __HAL_UART_GET_FLAG(&huart6, UART_FLAG_NE) ||
			  __HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE) ||
			  __HAL_UART_GET_FLAG(&huart6, UART_FLAG_PE) ){
		  HAL_UART_Abort(&huart6);
	  }
	  if((sp.count_frame > UART_TIMEOUT_TIME) & (sp.com_en == 0x01)){
		  //NVIC_SystemReset();
		  sp.com_en = 0x00;
		  sp.it_count = 0;
		  sp.ics_timeout_count += 1;
		  HAL_UART_ErrorCallback(&huart6);
	  }
	  if (sp.rd_idx == RX_DMA_WRITE_IDX){
		  osDelay(1);
	  }else{
	  	  sp.rxbuff[0] = sp.rx_dma_circ_buf[sp.rd_idx++];//
	  	  sp.rd_idx &= (RX_CIRC_BUF_SZ - 1);//128 = 0
	  	  if(((sp.rxbuff[0] & 0xE0) == 0xA0) && (sp.rxbuff[0] & 0x1f) == sp.id){
	  		  sp.prev_frame = getUs();
		  	  sp.rxbuff[1] = sp.rx_dma_circ_buf[sp.rd_idx++];//
		  	  sp.rd_idx &= (RX_CIRC_BUF_SZ - 1);//128 = 0
	  		  sensor_flag = sp.rxbuff[1];
	  		  if ((sensor_flag & SC_NONE) == SC_NONE){
	  			  sp.mode = sensor_flag;
	  			  if(sp.it_count < 4096){
	  				  sp.it_count += 1;
				  }else{
					  sp.it_count = 0;
				  }
			  }
	  		  if((sp.it_count == 1) & (sp.check2com_delay == 0)){
	  			  sp.check2com_delay = sp.prev_frame;
	  		  }
	  		  if((sp.it_count == COM_EN_IT_NUM) & (sp.com_en == 0x00)){
	  			  sp.check2com_delay = getUs() - sp.check2com_delay;
  				  sp.com_en |= 0x01;
	  		  }
	  		  sp.txbuff[1] = sensor_flag;
	  		  tx_length = getlen(sp.mode);
	  		  //taskENTER_CRITICAL();
	  		  for(int i = 0; i < MAX_TXBUFF_LEN; i++) {
	  			  sp.txbuff[i] &= 0x7f;
	  		  }
	  		  HAL_HalfDuplex_EnableTransmitter(&huart6);
	  		  ret = HAL_UART_Transmit(&huart6, sp.txbuff, tx_length, HAL_MAX_DELAY);
	  		  //HAL_UART_Transmit_IT(&huart6, sp.txbuff, tx_length);
	  		  HAL_HalfDuplex_EnableReceiver(&huart6);
	  		  //taskEXIT_CRITICAL();
			  }
	  	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
	  	if(sp.com_en == 0x01){
	  		HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
	  	}
	    osDelay(100);
	  //if (current_time - receive_time1 > LED_TOGGLE_TIME) {
	  //receive_time1 = current_time;
	  //}
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  for(;;)
  {
	  if(((sp.mode & SC_GYRO) == SC_GYRO) || ((sp.mode & SC_ACC) == SC_ACC)){
		  imu_update(&hspi1);
	  }
    osDelay(1);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartPSTask */
/**
* @brief Function implementing the PSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPSTask */
void StartPSTask(void const * argument)
{
  /* USER CODE BEGIN StartPSTask */
  /* Infinite loop */
  for(;;)
  {
	  if((sp.mode & SC_PS) == SC_PS){
		  ps_update(&hi2c2);
	  }

    osDelay(1);
  }
  /* USER CODE END StartPSTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
  /* Infinite loop */
  for(;;)
  {
	 if((sp.mode & SC_SW_ADC) == SC_SW_ADC){
		 adc_update(&hadc1);
	 }
	 osDelay(1);
  }
  /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Header_StartTXBUFFTask */
/**
* @brief Function implementing the TXBUFFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTXBUFFTask */
void StartTXBUFFTask(void const * argument)
{
  /* USER CODE BEGIN StartTXBUFFTask */
  /* Infinite loop */
  for(;;)
  {
	txbuff_update();
    osDelay(1);
  }
  /* USER CODE END StartTXBUFFTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
