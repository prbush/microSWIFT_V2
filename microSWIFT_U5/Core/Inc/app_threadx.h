/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.h
  * @author  MCD Application Team
  * @brief   ThreadX applicative header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_THREADX_H
#define __APP_THREADX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_dma.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "gnss.h"
#include "battery.h"
#include "ct_sensor.h"
#include "imu.h"
#include "iridium.h"
#include "log.h"
#include "math.h"
#include "mem_replacements.h"
#include "configuration.h"
#include "linked_list.h"

// Waves files
#include "NEDwaves_memlight.h"
#include "NEDwaves_memlight_emxAPI.h"
#include "NEDwaves_memlight_terminate.h"
#include "NEDwaves_memlight_types.h"
#include "rt_nonfinite.h"
#include "rtwhalf.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
 typedef enum status_flags{
 	// Ready states
 	GNSS_READY = 1 << 0,
 	IMU_READY = 1 << 1,
 	CT_READY = 1 << 2,
 	IRIDIUM_READY = 1 << 3,
 	WAVES_READY = 1 << 4,
 	// Done states
 	GNSS_DONE = 1 << 5,
 	IMU_DONE = 1 << 6,
 	CT_DONE = 1 << 7,
 	IRIDIUM_DONE = 1 << 8,
	WAVES_DONE = 1 << 9,
 	// Sleep states (for future use)
 	SET_STOP_2 = 1 << 10,
 	RESET_STOP_2 = 1 << 11,
 	SET_SHUTDOWN = 1 << 12,
 	RESET_SHUTDOWN = 1 << 13,
 	// Error states
 	GNSS_ERROR = 1 << 14,
 	IMU_ERROR = 1 << 15,
 	CT_ERROR = 1 << 16,
 	MODEM_ERROR = 1 << 17,
 	MEMORY_ALLOC_ERROR = 1 << 18,
 	DMA_ERROR = 1 << 19,
 	UART_ERROR = 1 << 20,
	RTC_ERROR = 1 << 21,
	// Misc
	GNSS_CONFIG_RECVD = 1 << 22,
	CT_MSG_RECVD = 1 << 23,
	IRIDIUM_MSG_RECVD = 1 << 24,
	UBX_QUEUE_FULL = 1 << 25
 }status_flags_t;

typedef enum led_sequence_t{
	INITIAL_LED_SEQUENCE = 1,
	TEST_PASSED_LED_SEQUENCE = 2,
	TEST_NON_CRITICAL_FAULT_LED_SEQUENCE = 3,
	TEST_CRITICAL_FAULT_LED_SEQUENCE = 4
}led_sequence_t;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// Total number of samples in a sampling window
// Sensor data arrays -> 2bytes * 8192 samples = 16384 bytes, which is 32 byte aligned.
#define SENSOR_DATA_ARRAY_SIZE (TOTAL_SAMPLES_PER_WINDOW * sizeof(int16_t))
// Waves arrays -> 4 bytes * 8192 samples = 32786 bytes, which is 32 byte aligned.
#define WAVES_ARRAY_SIZE (TOTAL_SAMPLES_PER_WINDOW * sizeof(float))
// The max times we'll try to get a single peripheral up before sending reset vector
#define MAX_SELF_TEST_RETRIES 3
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */
void MX_ThreadX_Init(device_handles_t *handles);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */
