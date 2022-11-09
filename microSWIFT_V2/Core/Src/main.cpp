/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPSWaves_Task */
osThreadId_t GPSWaves_TaskHandle;
const osThreadAttr_t GPSWaves_Task_attributes = {
  .name = "GPSWaves_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Update_Sys_Cloc */
osThreadId_t Update_Sys_ClocHandle;
const osThreadAttr_t Update_Sys_Cloc_attributes = {
  .name = "Update_Sys_Cloc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CT_Task */
osThreadId_t CT_TaskHandle;
const osThreadAttr_t CT_Task_attributes = {
  .name = "CT_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Write_Logs_Task */
osThreadId_t Write_Logs_TaskHandle;
const osThreadAttr_t Write_Logs_Task_attributes = {
  .name = "Write_Logs_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Check_Batt_Task */
osThreadId_t Check_Batt_TaskHandle;
const osThreadAttr_t Check_Batt_Task_attributes = {
  .name = "Check_Batt_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Send_Msg_Task */
osThreadId_t Send_Msg_TaskHandle;
const osThreadAttr_t Send_Msg_Task_attributes = {
  .name = "Send_Msg_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART6_UART_Init(void);
void GPS_Start(void *argument);
void IMU_Start(void *argument);
void GPSWaves_Start(void *argument);
void Update_Sys_Clock_Start(void *argument);
void CT_Start(void *argument);
void Write_Logs_Start(void *argument);
void Check_Battery_Start(void *argument);
void Send_Msg_Start(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(GPS_Start, NULL, &GPS_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(IMU_Start, NULL, &IMU_Task_attributes);

  /* creation of GPSWaves_Task */
  GPSWaves_TaskHandle = osThreadNew(GPSWaves_Start, NULL, &GPSWaves_Task_attributes);

  /* creation of Update_Sys_Cloc */
  Update_Sys_ClocHandle = osThreadNew(Update_Sys_Clock_Start, NULL, &Update_Sys_Cloc_attributes);

  /* creation of CT_Task */
  CT_TaskHandle = osThreadNew(CT_Start, NULL, &CT_Task_attributes);

  /* creation of Write_Logs_Task */
  Write_Logs_TaskHandle = osThreadNew(Write_Logs_Start, NULL, &Write_Logs_Task_attributes);

  /* creation of Check_Batt_Task */
  Check_Batt_TaskHandle = osThreadNew(Check_Battery_Start, NULL, &Check_Batt_Task_attributes);

  /* creation of Send_Msg_Task */
  Send_Msg_TaskHandle = osThreadNew(Send_Msg_Start, NULL, &Send_Msg_Task_attributes);

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
//	    uint8_t Test[] = "Hello World !!!\r\n"; //Data to send
//	    HAL_UART_Transmit(&huart2,Test,sizeof(Test),10);// Sending in normal mode
//	    HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_GPS_Start */
/**
  * @brief  Function implementing the GPS_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_GPS_Start */
void GPS_Start(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    uint8_t messagebuf[320] = {0};
    char ubx_nav_pvt_message[92] = {0};
    int32_t messageClass = 0;
    int32_t messageId = 0;
    uint16_t numbytesreceived = 0;
//    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle(&huart6, &messagebuf[0], 300, &numbytesreceived, 200);
    HAL_UART_Receive(&huart6, &messagebuf[0], 160, 200);

    int32_t retval = uUbxProtocolDecode((char*)&messagebuf[0], 192, &messageClass, &messageId, &ubx_nav_pvt_message[0], 92, NULL);

    if (retval == 92) {
	    int32_t lon = ubx_nav_pvt_message[24] + (ubx_nav_pvt_message[25]<<8) + (ubx_nav_pvt_message[26]<<16) + (ubx_nav_pvt_message[27]<<24);
	    int32_t lat = ubx_nav_pvt_message[28] + (ubx_nav_pvt_message[29]<<8) + (ubx_nav_pvt_message[30]<<16) + (ubx_nav_pvt_message[31]<<24);
	    int32_t vnorth = ubx_nav_pvt_message[48] + (ubx_nav_pvt_message[49] << 8) + (ubx_nav_pvt_message[50] << 16) + (ubx_nav_pvt_message[51] << 24);
	    int32_t veast = ubx_nav_pvt_message[52] + (ubx_nav_pvt_message[53] << 8) + (ubx_nav_pvt_message[54] << 16) + (ubx_nav_pvt_message[55] << 24);
	    int32_t vdown = ubx_nav_pvt_message[56] + (ubx_nav_pvt_message[57] << 8) + (ubx_nav_pvt_message[58] << 16) + (ubx_nav_pvt_message[59] << 24);

	    char txstr[140] = {0};
	    snprintf(txstr, 140, "\r\nLatitude: %ld\r\n Longitude: %ld\r\n Velocity North: %ld\r\n Velocity East: %ld\r\n Velocity Down: %ld\r\n", lon, lat, vnorth, veast, vdown);
	    HAL_UART_Transmit(&huart2,(uint8_t*) txstr, sizeof(txstr), 10);
    } else {
    	uint8_t errormsg[] = "didn't grab message\r\n";
    	HAL_UART_Transmit(&huart2, errormsg, sizeof(errormsg), 10);
    }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_IMU_Start */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Start */
void IMU_Start(void *argument)
{
  /* USER CODE BEGIN IMU_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Start */
}

/* USER CODE BEGIN Header_GPSWaves_Start */
/**
* @brief Function implementing the GPSWaves_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPSWaves_Start */
void GPSWaves_Start(void *argument)
{
  /* USER CODE BEGIN GPSWaves_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GPSWaves_Start */
}

/* USER CODE BEGIN Header_Update_Sys_Clock_Start */
/**
* @brief Function implementing the Update_Sys_Cloc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Update_Sys_Clock_Start */
void Update_Sys_Clock_Start(void *argument)
{
  /* USER CODE BEGIN Update_Sys_Clock_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Update_Sys_Clock_Start */
}

/* USER CODE BEGIN Header_CT_Start */
/**
* @brief Function implementing the CT_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CT_Start */
void CT_Start(void *argument)
{
  /* USER CODE BEGIN CT_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CT_Start */
}

/* USER CODE BEGIN Header_Write_Logs_Start */
/**
* @brief Function implementing the Write_Logs_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Write_Logs_Start */
void Write_Logs_Start(void *argument)
{
  /* USER CODE BEGIN Write_Logs_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Write_Logs_Start */
}

/* USER CODE BEGIN Header_Check_Battery_Start */
/**
* @brief Function implementing the Check_Batt_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Check_Battery_Start */
void Check_Battery_Start(void *argument)
{
  /* USER CODE BEGIN Check_Battery_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Check_Battery_Start */
}

/* USER CODE BEGIN Header_Send_Msg_Start */
/**
* @brief Function implementing the Send_Msg_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_Msg_Start */
void Send_Msg_Start(void *argument)
{
  /* USER CODE BEGIN Send_Msg_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Send_Msg_Start */
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
