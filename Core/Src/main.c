/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "app_types.h"
#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for UARTTXTask */
osThreadId_t UARTTXTaskHandle;
const osThreadAttr_t UARTTXTask_attributes = {
  .name = "UARTTXTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPIAcquisitionT */
osThreadId_t SPIAcquisitionTHandle;
const osThreadAttr_t SPIAcquisitionT_attributes = {
  .name = "SPIAcquisitionT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AccelQueue */
osMessageQueueId_t AccelQueueHandle;
uint8_t AccelQueueBuffer[ 100 * 12 ];
osStaticMessageQDef_t AccelQueueControlBlock;
const osMessageQueueAttr_t AccelQueue_attributes = {
  .name = "AccelQueue",
  .cb_mem = &AccelQueueControlBlock,
  .cb_size = sizeof(AccelQueueControlBlock),
  .mq_mem = &AccelQueueBuffer,
  .mq_size = sizeof(AccelQueueBuffer)
};
/* Definitions for SPIBuffMutex */
osMutexId_t SPIBuffMutexHandle;
const osMutexAttr_t SPIBuffMutex_attributes = {
  .name = "SPIBuffMutex"
};
/* Definitions for SPIBinarySem */
osSemaphoreId_t SPIBinarySemHandle;
const osSemaphoreAttr_t SPIBinarySem_attributes = {
  .name = "SPIBinarySem"
};
/* USER CODE BEGIN PV */

// tx and rx buffers for SPI
// their size is 7 since we have a initial command byte
// and then 6 bytes of dummy data so the clock drives and
// shifts out the 6 register bytes
static uint8_t spi_tx_buffer[SPI_BUFF_SIZE];
static uint8_t spi_rx_buffer[SPI_BUFF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartUARTTXTask(void *argument);
void StartSPIAcquisitionTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t reverse6(uint8_t v)
{
    v &= 0x7F;                // keep only 6 bits

    uint8_t r = 0;
    for (int i = 0; i < 6; i++)
    {
        r <<= 1;
        r |= (v & 1);
        v >>= 1;
    }
    return r;
}

static void generate_spi_cmd(uint8_t *cmd_buffer, uint8_t reg_address, bool auto_increment, bool read)
{
	uint8_t msb_reg_addr = reverse6(reg_address);

	cmd_buffer[0] = (msb_reg_addr << 2) | (auto_increment << 1) | read;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of SPIBuffMutex */
  SPIBuffMutexHandle = osMutexNew(&SPIBuffMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SPIBinarySem */
  SPIBinarySemHandle = osSemaphoreNew(1, 1, &SPIBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of AccelQueue */
  AccelQueueHandle = osMessageQueueNew (100, 12, &AccelQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UARTTXTask */
  UARTTXTaskHandle = osThreadNew(StartUARTTXTask, NULL, &UARTTXTask_attributes);

  /* creation of SPIAcquisitionT */
  SPIAcquisitionTHandle = osThreadNew(StartSPIAcquisitionTask, NULL, &SPIAcquisitionT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1)
    {
    	Accel_Sample accel_data = {
    			.ax = (int16_t)(spi_rx_buffer[2] << 8 | spi_rx_buffer[1]),
				.ay = (int16_t)(spi_rx_buffer[4] << 8 | spi_rx_buffer[3]),
				.az = (int16_t)(spi_rx_buffer[6] << 8 | spi_rx_buffer[5])
    	};
        // SPI transfer done, received bytes are now in your rx buffer, push to queue
    	osMessageQueuePut(AccelQueueHandle, &accel_data, 0, 0);

    	// increment count, wake sleeping task in semaphore queue
    	osSemaphoreRelease(SPIBinarySemHandle);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUARTTXTask */
/**
  * @brief  Function implementing the UARTTXTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUARTTXTask */
void StartUARTTXTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	Accel_Sample accel_data;
	for(;;)
	{
		// read/pop data from rx queue, osWaitForever
		if(osMessageQueueGet(AccelQueueHandle, &accel_data, 0, osWaitForever) == osOK)
		{
			uint8_t buf[2];

			// x-axis
			buf[0] = accel_data.ax & 0xFF;      // LSB
			buf[1] = (accel_data.ax >> 8) & 0xFF; // MSB
			HAL_UART_Transmit_IT(&huart2, buf, 2);

			// y-axis
			buf[0] = accel_data.ay & 0xFF;      // LSB
			buf[1] = (accel_data.ay >> 8) & 0xFF; // MSB
			HAL_UART_Transmit_IT(&huart2, buf, 2);

			// z-axis
			buf[0] = accel_data.az & 0xFF;      // LSB
			buf[1] = (accel_data.az >> 8) & 0xFF; // MSB
			HAL_UART_Transmit_IT(&huart2, buf, 2);
		}

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSPIAcquisitionTask */
/**
* @brief Function implementing the SPIAcquisitionT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPIAcquisitionTask */
void StartSPIAcquisitionTask(void *argument)
{
  /* USER CODE BEGIN StartSPIAcquisitionTask */
  /* Infinite loop */
  for(;;)
  {
	  memset(spi_tx_buffer, 0x00, SPI_BUFF_SIZE);

	  generate_spi_cmd(spi_tx_buffer, OUT_X_L_ADDR, true, true);

	  HAL_SPI_TransmitReceive_IT(&hspi1, spi_tx_buffer, spi_rx_buffer, 7);

	  // put our current task to sleep into semaphore queue, and decrement count
	  osSemaphoreAcquire(SPIBinarySemHandle, osWaitForever);
  }
  /* USER CODE END StartSPIAcquisitionTask */
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
  if (htim->Instance == TIM6)
  {
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
