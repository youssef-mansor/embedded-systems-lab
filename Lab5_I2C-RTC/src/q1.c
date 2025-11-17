/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : RTC Full Date-Time Display & Alarm 2 (Date/Hour/Minute Match)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
#define RTC_ADDR_WRITE 0xD0
#define RTC_ADDR_READ  0xD1
#define RTC_CONTROL_REG    0x0E
#define RTC_STATUS_REG     0x0F
#define RTC_ALARM2_MIN     0x0B
#define RTC_ALARM2_HOUR    0x0C
#define RTC_ALARM2_DATE    0x0D
#define READY_LED_GPIO GPIOB
#define READY_LED_PIN  GPIO_PIN_3
#define BUZZER_GPIO    GPIOB
#define BUZZER_PIN     GPIO_PIN_4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char uartBuf[100];
uint8_t txBuf[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void RTC_ReadyIndicator(void);
void RTC_Init(void);
void RTC_SetDateTime(uint8_t hour, uint8_t minute, uint8_t second,
                     uint8_t day_of_week, uint8_t date, uint8_t month, uint8_t year);
void RTC_ReadDateTime(uint8_t *hour, uint8_t *minute, uint8_t *second,
                      uint8_t *day_of_week, uint8_t *date, uint8_t *month, uint8_t *year);
void UART_PrintDateTime(uint8_t hour, uint8_t minute, uint8_t second,
                        uint8_t date, uint8_t month, uint8_t year, uint8_t day_of_week);
void UART_PrintMessage(const char *msg);
void RTC_SetAlarm2(uint8_t date, uint8_t hour, uint8_t minute);
void RTC_EnableAlarm2(void);
uint8_t RTC_CheckAlarm2(void);
void RTC_ClearAlarm2Flag(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
}

// Set full date & time
void RTC_SetDateTime(uint8_t hour, uint8_t minute, uint8_t second,
                     uint8_t day_of_week, uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t buffer[2];

    buffer[0] = 0x00; buffer[1] = second;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
    buffer[0] = 0x01; buffer[1] = minute;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
    buffer[0] = 0x02; buffer[1] = hour;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
    buffer[0] = 0x03; buffer[1] = day_of_week;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
    buffer[0] = 0x04; buffer[1] = date;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
    buffer[0] = 0x05; buffer[1] = month;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
    buffer[0] = 0x06; buffer[1] = year;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 2, 10);
}

// Read full date & time
void RTC_ReadDateTime(uint8_t *hour, uint8_t *minute, uint8_t *second,
                      uint8_t *day_of_week, uint8_t *date, uint8_t *month, uint8_t *year)
{
    uint8_t buffer[2];
    buffer[0] = 0x00; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *second = buffer[1];
    buffer[0] = 0x01; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *minute = buffer[1];
    buffer[0] = 0x02; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *hour = buffer[1] & 0x3F;
    buffer[0] = 0x03; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *day_of_week = buffer[1];
    buffer[0] = 0x04; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *date = buffer[1];
    buffer[0] = 0x05; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *month = buffer[1];
    buffer[0] = 0x06; HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, buffer, 1, 10); HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, buffer+1, 1, 10); *year = buffer[1];
}

// Print full date & time via UART
void UART_PrintDateTime(uint8_t hour, uint8_t minute, uint8_t second,
                        uint8_t date, uint8_t month, uint8_t year, uint8_t day_of_week)
{
    sprintf(uartBuf, "20%02x-%02x-%02x %02x:%02x:%02x (Day %d)\r\n", year, month, date, hour, minute, second, day_of_week);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, sizeof(uartBuf), 10);
}

// Print message via UART (no strlen)
void UART_PrintMessage(const char *msg)
{
    int len = 0;
    while (msg[len] != '\0') {
        len++;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 10);
}

// Set Alarm2: match date, hour, minute
void RTC_SetAlarm2(uint8_t date, uint8_t hour, uint8_t minute)
{
    // Match minute (A2M2=0, MSB cleared)
    txBuf[0] = RTC_ALARM2_MIN;
    txBuf[1] = minute & 0x7F;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // Match hour (A2M3=0, MSB cleared)
    txBuf[0] = RTC_ALARM2_HOUR;
    txBuf[1] = hour & 0x3F;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // Match date (A2M4=0, MSB cleared)
    txBuf[0] = RTC_ALARM2_DATE;
    txBuf[1] = date & 0x3F;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);
}

// Enable Alarm 2 interrupt
void RTC_EnableAlarm2(void)
{
    txBuf[0] = RTC_CONTROL_REG;
    txBuf[1] = 0x1E; // INTCN=1 (bit2), A2IE=1 (bit1)
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);

    // Clear status register (0x0F)
    txBuf[0] = RTC_STATUS_REG;
    txBuf[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, HAL_MAX_DELAY);
}

// Check if Alarm 2 has triggered
uint8_t RTC_CheckAlarm2(void)
{
    uint8_t statusReg;
    txBuf[0] = RTC_STATUS_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, &statusReg, 1, 10);
    return (statusReg & 0x02); // bit 1 = A2F
}

// Clear Alarm 2 flag
void RTC_ClearAlarm2Flag(void)
{
    uint8_t statusReg;
    txBuf[0] = RTC_STATUS_REG;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, RTC_ADDR_READ, &statusReg, 1, 10);
    statusReg &= ~0x02;
    txBuf[0] = RTC_STATUS_REG;
    txBuf[1] = statusReg;
    HAL_I2C_Master_Transmit(&hi2c1, RTC_ADDR_WRITE, txBuf, 2, 10);
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
  MX_SPI3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  RTC_Init();
  UART_PrintMessage("RTC Demo: Full Date/Time & Alarm 2.\r\n");

  // Set time: 18:56:00, day-of-week 2, date 17, month 11, year 25 (BCD)
  RTC_SetDateTime(0x18, 0x56, 0x00, 0x02, 0x17, 0x11, 0x25);

  // Set Alarm 2: trigger at date 17, hour 18, minute 57
  RTC_SetAlarm2(0x17, 0x18, 0x57);
  RTC_EnableAlarm2();

  uint8_t hour, minute, second, day_of_week, date, month, year;

  while (1)
  {
    RTC_ReadDateTime(&hour, &minute, &second, &day_of_week, &date, &month, &year);
    UART_PrintDateTime(hour, minute, second, date, month, year, day_of_week);

    if (RTC_CheckAlarm2())
    {
        UART_PrintMessage("ALARM 2 MATCH! Buzzing.\r\n");
        HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, GPIO_PIN_SET);
        HAL_Delay(5000);
        HAL_GPIO_WritePin(BUZZER_GPIO, BUZZER_PIN, GPIO_PIN_RESET);
        RTC_ClearAlarm2Flag();
    }

    HAL_Delay(1000);
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
