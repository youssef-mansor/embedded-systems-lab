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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// I2C address definitions
#define RTC_ADDR_WRITE 0xD0
#define RTC_ADDR_READ  0xD1

// RTC Register addresses
#define RTC_SECONDS_REG    0x00
#define RTC_MINUTES_REG    0x01
#define RTC_HOURS_REG      0x02
#define RTC_TEMP_MSB_REG   0x11
#define RTC_TEMP_LSB_REG   0x12

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId TimeDisplayTaskHandle;
osThreadId TempDisplayTaskHandle;
/* USER CODE BEGIN PV */
osMutexId uart_mutexHandle;  // Mutex for UART protection

// RTC time buffers
uint8_t secbuffer[2], minbuffer[2], hourbuffer[2];

// Temperature buffers
uint8_t temp_msb_buffer[2];
uint8_t temp_lsb_buffer[2];

// UART formatting buffers
char time_uart_buf[50];
char temp_uart_buf[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void start_TimeDisplayTask(void const * argument);
void start_TempDisplayTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Check if RTC is ready
void RTC_Init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, RTC_ADDR_WRITE, 10, HAL_MAX_DELAY) == HAL_OK)
    {
        // RTC detected - blink LED to indicate
        for (int i = 0; i < 6; i++)
        {
            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            HAL_Delay(200);
        }
    }
}

// Set RTC time (24h format, BCD)
void RTC_SetTime(uint8_t hour, uint8_t minute, uint8_t second)
{
    uint8_t txBuf[2];
    
    // Set seconds
    txBuf[0] = RTC_SECONDS_REG;
    txBuf[1] = second;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, 10);

    // Set minutes
    txBuf[0] = RTC_MINUTES_REG;
    txBuf[1] = minute;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, 10);

    // Set hours (24h format)
    txBuf[0] = RTC_HOURS_REG;
    txBuf[1] = hour;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, 10);
}

// Read RTC time
void RTC_ReadTime(void)
{
    // Read seconds
    secbuffer[0] = RTC_SECONDS_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, secbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, secbuffer + 1, 1, 10);

    // Read minutes
    minbuffer[0] = RTC_MINUTES_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, minbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, minbuffer + 1, 1, 10);

    // Read hours
    hourbuffer[0] = RTC_HOURS_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, hourbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, hourbuffer + 1, 1, 10);
    
    // Mask to 24h format
    hourbuffer[1] &= 0x3F;
}

// Read DS3231 temperature (10-bit resolution)
float RTC_ReadTemperature(void)
{
    int8_t temp_msb;
    uint8_t temp_lsb;
    float temperature;
    
    // Read MSB (register 0x11) - integer part, 2's complement
    temp_msb_buffer[0] = RTC_TEMP_MSB_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, temp_msb_buffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, temp_msb_buffer + 1, 1, 10);
    temp_msb = (int8_t)temp_msb_buffer[1];
    
    // Read LSB (register 0x12) - fractional part (upper 2 bits)
    temp_lsb_buffer[0] = RTC_TEMP_LSB_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, temp_lsb_buffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, temp_lsb_buffer + 1, 1, 10);
    temp_lsb = temp_lsb_buffer[1];
    
    // Combine: integer part + (fractional >> 6) * 0.25
    // Upper 2 bits of LSB represent 0.25°C increments
    temperature = (float)temp_msb + ((temp_lsb >> 6) * 0.25f);
    
    return temperature;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize RTC and set initial time
  RTC_Init();
  
  // Set time to 14:30:00 (2:30 PM in 24h format)
  // 0x14 = 14 hours, 0x30 = 30 minutes, 0x00 = 0 seconds (BCD format)
  RTC_SetTime(0x14, 0x30, 0x00);
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  // Create mutex for UART protection
  osMutexDef(uart_mutex);
  uart_mutexHandle = osMutexCreate(osMutex(uart_mutex));
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
  /* definition and creation of TimeDisplayTask */
  osThreadDef(TimeDisplayTask, start_TimeDisplayTask, osPriorityNormal, 0, 256);
  TimeDisplayTaskHandle = osThreadCreate(osThread(TimeDisplayTask), NULL);

  /* definition and creation of TempDisplayTask */
  osThreadDef(TempDisplayTask, start_TempDisplayTask, osPriorityNormal, 0, 256);
  TempDisplayTaskHandle = osThreadCreate(osThread(TempDisplayTask), NULL);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_TimeDisplayTask */
/**
  * @brief  Function implementing the TimeDisplayTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_TimeDisplayTask */
void start_TimeDisplayTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    for (;;)
    {
        // Read current time from RTC
        RTC_ReadTime();
        
        // Format time string (BCD to decimal conversion for display)
        sprintf(time_uart_buf, "Time: %02x:%02x:%02x\r\n", 
                hourbuffer[1], minbuffer[1], secbuffer[1]);
        
        // Take mutex to protect UART
        osMutexWait(uart_mutexHandle, osWaitForever);
        
        // Transmit time string
        HAL_UART_Transmit(&huart2, (uint8_t *)time_uart_buf, 
                          strlen(time_uart_buf), HAL_MAX_DELAY);
        
        // Release mutex
        osMutexRelease(uart_mutexHandle);
        
        // Delay 1 second before next update
        osDelay(1000);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_TempDisplayTask */
/**
* @brief Function implementing the TempDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_TempDisplayTask */
void start_TempDisplayTask(void const * argument)
{
  /* USER CODE BEGIN start_TempDisplayTask */
    float temperature;
    
    for (;;)
    {
        // Read temperature from DS3231
        temperature = RTC_ReadTemperature();
        
        // Format temperature string
        sprintf(temp_uart_buf, "Temp: %.2f°C\r\n", temperature);
        
        // Take mutex to protect UART
        osMutexWait(uart_mutexHandle, osWaitForever);
        
        // Transmit temperature string
        HAL_UART_Transmit(&huart2, (uint8_t *)temp_uart_buf, 
                          strlen(temp_uart_buf), HAL_MAX_DELAY);
        
        // Release mutex
        osMutexRelease(uart_mutexHandle);
        
        // Delay 2 seconds before next update (different from time task)
        osDelay(2000);
    }
  /* USER CODE END start_TempDisplayTask */
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
