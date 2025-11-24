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
#include <string.h>

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
#define STACK_BYTES(x) ((x) / sizeof(StackType_t))
#define INPUT_BUFFER_SIZE 8


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId calculation_tasHandle;
osThreadId print_taskHandle;
osThreadId Task3_UARTReceiHandle;
osMessageQId ISR_to_calculationHandle;
osMessageQId calculation_to_printHandle;
/* USER CODE BEGIN PV */
osSemaphoreId uart_semaphoreHandle;  // Binary semaphore handle
uint8_t rx_data;                     // For interrupt-driven reception
char input_buffer[16];               // Buffer for incoming expression
uint8_t buffer_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void start_calculation_task(void const * argument);
void start_print_task(void const * argument);
void start_Task3_UARTReceive(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart2_send_string(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Create binary semaphore */
  osSemaphoreDef(uart_semaphore);
  uart_semaphoreHandle = osSemaphoreCreate(osSemaphore(uart_semaphore), 1);
  
  /* Take the semaphore initially so task waits until ISR gives it */
  osSemaphoreWait(uart_semaphoreHandle, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ISR_to_calculation */
  osMessageQDef(ISR_to_calculation, 16, uint8_t);
  ISR_to_calculationHandle = osMessageCreate(osMessageQ(ISR_to_calculation), NULL);

  /* definition and creation of calculation_to_print */
  osMessageQDef(calculation_to_print, 16, uint8_t);
  calculation_to_printHandle = osMessageCreate(osMessageQ(calculation_to_print), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of calculation_tas */
  osThreadDef(calculation_tas, start_calculation_task, osPriorityNormal, 0, 128);
  calculation_tasHandle = osThreadCreate(osThread(calculation_tas), NULL);

  /* definition and creation of print_task */
  osThreadDef(print_task, start_print_task, osPriorityLow, 0, 128);
  print_taskHandle = osThreadCreate(osThread(print_task), NULL);

  /* definition and creation of Task3_UARTRecei */
  osThreadDef(Task3_UARTRecei, start_Task3_UARTReceive, osPriorityAboveNormal, 0, 128);
  Task3_UARTReceiHandle = osThreadCreate(osThread(Task3_UARTRecei), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start UART interrupt reception AFTER semaphore and tasks are created */
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  
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

/* USER CODE BEGIN Header_start_calculation_task */
/**
  * @brief  Function implementing the calculation_tas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_calculation_task */
void start_calculation_task(void const * argument)
{
  /* USER CODE BEGIN 5 */
    uint8_t received_byte;
    char expr[INPUT_BUFFER_SIZE];
    uint8_t idx = 0;

    for (;;)
    {
        if (xQueueReceive(ISR_to_calculationHandle, &received_byte, portMAX_DELAY) == pdTRUE)
        {
            if (received_byte != '\n') // Not end of expression
            {
                if (idx < INPUT_BUFFER_SIZE - 1)
                    expr[idx++] = received_byte;
            }
            else
            {
                expr[idx] = '\0'; // null-terminate

                // Evaluate simple digit(+or-)digit expression
                if (idx == 3 && (expr[1] == '+' || expr[1] == '-'))
                {
                    int a = expr[0] - '0';
                    char op = expr[1];
                    int b = expr[2] - '0';
                    int result = (op == '+') ? (a + b) : (a - b);
                    
                    // Echo expression to print queue
                    for (uint8_t i = 0; i < idx; i++)
                        xQueueSend(calculation_to_printHandle, &expr[i], portMAX_DELAY);
                    
                    // Send newline to print queue
                    uint8_t nl = '\n';
                    uint8_t cr = '\r';
                    xQueueSend(calculation_to_printHandle, &nl, portMAX_DELAY);
                    xQueueSend(calculation_to_printHandle, &cr, portMAX_DELAY);

                    // Send each digit of result to print queue
                    if (result < 0)
                    {
                        // Handle negative results
                        uint8_t minus = '-';
                        xQueueSend(calculation_to_printHandle, &minus, portMAX_DELAY);
                        result = -result;
                    }
                    
                    if (result < 10)
                    {
                        uint8_t c = result + '0';
                        xQueueSend(calculation_to_printHandle, &c, portMAX_DELAY);
                    }
                    else
                    {
                        uint8_t tens = (result / 10) + '0';
                        uint8_t ones = (result % 10) + '0';
                        xQueueSend(calculation_to_printHandle, &tens, portMAX_DELAY);
                        xQueueSend(calculation_to_printHandle, &ones, portMAX_DELAY);
                    }

                    // Send newline to print queue
                    xQueueSend(calculation_to_printHandle, &nl, portMAX_DELAY);
                    xQueueSend(calculation_to_printHandle, &cr, portMAX_DELAY);
                }

                idx = 0; // reset buffer
            }
        }
        vTaskDelay(1);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_print_task */
/* UART Rx Complete Callback --------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        // Disable UART interrupt
        HAL_NVIC_DisableIRQ(USART2_IRQn);
        
        // Give semaphore to unblock Task3_UARTReceive
        xSemaphoreGiveFromISR(uart_semaphoreHandle, &xHigherPriorityTaskWoken);
        
        // Yield to higher priority task if needed
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
* @brief Function implementing the print_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_print_task */
void start_print_task(void const * argument)
{
  /* USER CODE BEGIN start_print_task */
    uint8_t c;
    
    for (;;)
    {
        if (xQueueReceive(calculation_to_printHandle, &c, portMAX_DELAY) == pdTRUE)
        {
            taskENTER_CRITICAL();
            HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
            taskEXIT_CRITICAL();
        }
    }
  /* USER CODE END start_print_task */
}

/* USER CODE BEGIN Header_start_Task3_UARTReceive */
/**
* @brief Function implementing the Task3_UARTRecei thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_Task3_UARTReceive */
void start_Task3_UARTReceive(void const * argument)
{
  /* USER CODE BEGIN start_Task3_UARTReceive */
    for (;;)
    {
        // Wait for semaphore from ISR (blocks until data arrives)
        if (osSemaphoreWait(uart_semaphoreHandle, osWaitForever) == osOK)
        {
            // Process the received character
            // Only accept digits or '+'/'-' or '\r'
            if (((rx_data >= '0' && rx_data <= '9') || rx_data == '+' || rx_data == '-' || rx_data == '\r')
                 && buffer_index < INPUT_BUFFER_SIZE - 1)
            {
                if (rx_data != '\r')
                {
                    // Store character in buffer
                    input_buffer[buffer_index++] = rx_data;
                }
                else
                {
                    // Send all buffered characters to queue
                    for (uint8_t i = 0; i < buffer_index; i++)
                    {
                        xQueueSend(ISR_to_calculationHandle, &input_buffer[i], portMAX_DELAY);
                    }

                    // Send delimiter to mark end of expression
                    char delimiter = '\n';
                    xQueueSend(ISR_to_calculationHandle, &delimiter, portMAX_DELAY);
                    
                    // Reset buffer for next input
                    buffer_index = 0;
                }
            }
            
            // Re-arm UART reception
            HAL_UART_Receive_IT(&huart2, &rx_data, 1);
            
            // Re-enable UART interrupt
            HAL_NVIC_EnableIRQ(USART2_IRQn);
        }
    }
  /* USER CODE END start_Task3_UARTReceive */
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
