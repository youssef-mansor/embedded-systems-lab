/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with RTC Alarm 1 and UART input
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
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
#define RTC_CONTROL_REG    0x0E
#define RTC_STATUS_REG     0x0F
#define RTC_ALARM1_SEC     0x07
#define RTC_ALARM1_MIN     0x08
#define RTC_ALARM1_HOUR    0x09
#define RTC_ALARM1_DATE    0x0A

// LED pin
#define READY_LED_GPIO GPIOB
#define READY_LED_PIN  GPIO_PIN_3

// Buzzer pin
#define BUZZER_GPIO    GPIOB
#define BUZZER_PIN     GPIO_PIN_4

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Buffers for RTC registers
uint8_t secbuffer[2] = {0x00, 0x00};
uint8_t minbuffer[2] = {0x01, 0x00};
uint8_t hourbuffer[2] = {0x02, 0x00};
uint8_t txBuf[2];
char uartBuf[100];

// UART input variables
uint8_t rx_char;
char input_buffer[7] = {0};  // Buffer for 6 digits + null terminator
volatile uint8_t input_index = 0;
volatile uint8_t new_alarm_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
void UART_PrintMessage(const char* msg);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ----- Utility Functions ----- */

// Convert decimal to BCD
uint8_t decToBcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

// Convert BCD to decimal
uint8_t bcdToDec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

/* ----- RTC Functions ----- */

// Blink LED to indicate RTC ready
void RTC_ReadyIndicator(void)
{
    for (int i = 0; i < 10; i++)
    {
        HAL_GPIO_TogglePin(READY_LED_GPIO, READY_LED_PIN);
        HAL_Delay(250);
    }
}

// Initialize RTC: check device readiness
void RTC_Init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, RTC_ADDR_WRITE, 10, HAL_MAX_DELAY) == HAL_OK)
    {
        RTC_ReadyIndicator();
    }
    else
    {
        UART_PrintMessage("ERROR: RTC not detected!\r\n");
    }
}

// Set RTC time (24h format, BCD)
void RTC_SetTime(uint8_t hour, uint8_t minute, uint8_t second)
{
    // seconds
    secbuffer[0] = 0x00;
    secbuffer[1] = second;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, secbuffer, 2, 10);

    // minutes
    minbuffer[0] = 0x01;
    minbuffer[1] = minute;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, minbuffer, 2, 10);

    // hours (24-hour format: bit 6 = 0)
    hourbuffer[0] = 0x02;
    hourbuffer[1] = hour & 0x3F;  // Ensure 24-hour format
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, hourbuffer, 2, 10);
}

// Read RTC time into buffers
void RTC_ReadTime(void)
{
    // seconds
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, secbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, secbuffer + 1, 1, 10);

    // minutes
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, minbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, minbuffer + 1, 1, 10);

    // hours
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, hourbuffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, hourbuffer + 1, 1, 10);

    // mask upper hour bits
    hourbuffer[1] &= 0x3F;  // 24-hour format
}

// Print time via UART
void UART_PrintTime(void)
{
    sprintf(uartBuf, "Time: %02x:%02x:%02x\r\n", hourbuffer[1], minbuffer[1], secbuffer[1]);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 10);
}

// Set Alarm 1 on DS3231 (24-hour format)
void RTC_SetAlarm1(uint8_t hour, uint8_t minute, uint8_t second)
{
    // A1 Seconds (0x07)
    txBuf[0] = RTC_ALARM1_SEC;
    txBuf[1] = second;  // Match seconds
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // A1 Minutes (0x08)
    txBuf[0] = RTC_ALARM1_MIN;
    txBuf[1] = minute;  // Match minutes
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // A1 Hours (0x09) - 24-hour format
    txBuf[0] = RTC_ALARM1_HOUR;
    txBuf[1] = hour & 0x3F;  // Ensure 24-hour format (bit 6 = 0)
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // A1 Day/Date (0x0A)
    txBuf[0] = RTC_ALARM1_DATE;
    txBuf[1] = 0x80;  // A1M4=1, ignore day/date match
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);
}

// Enable Alarm 1 interrupt
void RTC_EnableAlarm1(void)
{
    // Control register (0x0E)
    txBuf[0] = RTC_CONTROL_REG;
    txBuf[1] = 0x1D;  // INTCN=1 (bit2), A1IE=1 (bit0)
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // Clear status register (0x0F)
    txBuf[0] = RTC_STATUS_REG;
    txBuf[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);
}

// Check if Alarm 1 has triggered
uint8_t RTC_CheckAlarm1(void)
{
    uint8_t statusReg;
    
    txBuf[0] = RTC_STATUS_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, &statusReg, 1, 10);
    
    return (statusReg & 0x01);
}

// Clear Alarm 1 flag
void RTC_ClearAlarm1Flag(void)
{
    uint8_t statusReg;
    
    txBuf[0] = RTC_STATUS_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, &statusReg, 1, 10);
    
    statusReg &= ~0x01;
    
    txBuf[0] = RTC_STATUS_REG;
    txBuf[1] = statusReg;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, 10);
}

// Print message via UART
void UART_PrintMessage(const char* msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

/* ----- UART Input Processing ----- */

// Process the 6-digit input and set alarm
void ProcessAlarmInput(void)
{
    if (input_index == 6)
    {
        // Extract hours, minutes, seconds from input_buffer
        char hh[3] = {input_buffer[0], input_buffer[1], '\0'};
        char mm[3] = {input_buffer[2], input_buffer[3], '\0'};
        char ss[3] = {input_buffer[4], input_buffer[5], '\0'};
        
        int hours = atoi(hh);
        int minutes = atoi(mm);
        int seconds = atoi(ss);
        
        // Validate input
        if (hours >= 0 && hours < 24 && minutes >= 0 && minutes < 60 && seconds >= 0 && seconds < 60)
        {
            // Convert to BCD
            uint8_t hours_bcd = decToBcd((uint8_t)hours);
            uint8_t minutes_bcd = decToBcd((uint8_t)minutes);
            uint8_t seconds_bcd = decToBcd((uint8_t)seconds);
            
            // Set the alarm
            RTC_SetAlarm1(hours_bcd, minutes_bcd, seconds_bcd);
            RTC_ClearAlarm1Flag();
            
            sprintf(uartBuf, "Alarm set to %02d:%02d:%02d (24-hour format)\r\n", hours, minutes, seconds);
            UART_PrintMessage(uartBuf);
            UART_PrintMessage("Enter new alarm time (hhmmss): ");
        }
        else
        {
            UART_PrintMessage("Invalid time! Hours: 00-23, Minutes/Seconds: 00-59\r\n");
            UART_PrintMessage("Enter alarm time (hhmmss): ");
        }
        
        // Reset input buffer
        input_index = 0;
        memset(input_buffer, 0, sizeof(input_buffer));
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize RTC and check connection
  RTC_Init();
  
  // Set initial time to 14:59:55 (24-hour format)
  RTC_SetTime(decToBcd(14), decToBcd(59), decToBcd(55));
  
  // Set initial Alarm 1 to trigger at 15:00:00
  RTC_SetAlarm1(decToBcd(15), decToBcd(0), decToBcd(0));
  
  // Enable Alarm 1 interrupt
  RTC_EnableAlarm1();
  
  // Print startup messages
  UART_PrintMessage("\r\n=== DS3231 RTC with UART Alarm Input ===\r\n");
  UART_PrintMessage("RTC initialized: 14:59:55\r\n");
  UART_PrintMessage("Default alarm: 15:00:00\r\n");
  UART_PrintMessage("Enter alarm time (hhmmss, 24-hour format): ");
  
  // Enable UART interrupt for receiving
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Read and display current time
    RTC_ReadTime();
    UART_PrintTime();
    
    // Check if alarm has triggered
    if (RTC_CheckAlarm1())
    {
        UART_PrintMessage("\r\n*** ALARM TRIGGERED! ***\r\n");
        
        // Turn on buzzer
        HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, GPIO_PIN_SET);
        
        // Blink LED during alarm
        for (int i = 0; i < 10; i++)
        {
            HAL_GPIO_TogglePin(READY_LED_GPIO, READY_LED_PIN);
            HAL_Delay(500);
        }
        
        // Turn off buzzer and LED
        HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(READY_LED_GPIO, READY_LED_PIN, GPIO_PIN_RESET);
        
        // Clear the alarm flag
        RTC_ClearAlarm1Flag();
        
        UART_PrintMessage("Alarm cleared.\r\n");
        UART_PrintMessage("Enter new alarm time (hhmmss): ");
    }
    
    // Process new alarm input if ready
    if (new_alarm_ready)
    {
        ProcessAlarmInput();
        new_alarm_ready = 0;
    }
    
    HAL_Delay(1000);  // 1-second delay
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000C12;
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
  hi2c3.Init.Timing = 0x00000C12;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Check if character is a digit
        if (rx_char >= '0' && rx_char <= '9')
        {
            if (input_index < 6)
            {
                input_buffer[input_index++] = rx_char;
                
                // Echo the character back
                HAL_UART_Transmit(&huart2, &rx_char, 1, 10);
                
                // If we have 6 digits, set flag to process
                if (input_index == 6)
                {
                    UART_PrintMessage("\r\n");
                    new_alarm_ready = 1;
                }
            }
        }
        else if (rx_char == '\r' || rx_char == '\n')
        {
            // Enter pressed - check if we have incomplete data
            if (input_index > 0 && input_index < 6)
            {
                UART_PrintMessage("\r\nError: Enter exactly 6 digits (hhmmss)!\r\n");
                UART_PrintMessage("Enter alarm time (hhmmss): ");
                input_index = 0;
                memset(input_buffer, 0, sizeof(input_buffer));
            }
        }
        else if (rx_char == 0x08 || rx_char == 0x7F)  // Backspace or DEL
        {
            if (input_index > 0)
            {
                input_index--;
                input_buffer[input_index] = '\0';
                // Echo backspace sequence
                UART_PrintMessage("\b \b");
            }
        }
        else if (rx_char == 0x1B)  // ESC key - clear buffer
        {
            if (input_index > 0)
            {
                // Clear the line
                for (int i = 0; i < input_index; i++)
                {
                    UART_PrintMessage("\b \b");
                }
                input_index = 0;
                memset(input_buffer, 0, sizeof(input_buffer));
            }
        }
        
        // Re-enable receive interrupt for next character
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    }
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
