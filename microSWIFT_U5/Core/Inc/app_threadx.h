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
#include <math.h>
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_dma.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "Peripherals/gnss.h"
#include "Peripherals/battery.h"
#include "Peripherals/ct_sensor.h"
#include "Peripherals/rf_switch.h"
#include "Peripherals/imu.h"
#include "Peripherals/iridium.h"
#include "NEDWaves/mem_replacements.h"
#include "configuration.h"
#include "linked_list.h"

// Waves files
#include "NEDWaves/NEDwaves_memlight.h"
#include "NEDWaves/NEDwaves_memlight_emxAPI.h"
#include "NEDWaves/NEDwaves_memlight_terminate.h"
#include "NEDWaves/NEDwaves_memlight_types.h"
#include "NEDWaves/rt_nonfinite.h"
#include "NEDWaves/rtwhalf.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum control_flags{
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
	FULL_CYCLE_COMPLETE = 1 << 10,
	// DMA reception flags
	GNSS_CONFIG_RECVD = 1 << 11,
	CT_MSG_RECVD = 1 << 12,
	IRIDIUM_MSG_RECVD = 1 << 13
}control_flags_t;

typedef enum error_flags{
 	GNSS_ERROR = 1 << 1,
 	IMU_ERROR = 1 << 2,
 	CT_ERROR = 1 << 3,
 	MODEM_ERROR = 1 << 4,
 	MEMORY_ALLOC_ERROR = 1 << 5,
 	DMA_ERROR = 1 << 6,
 	UART_ERROR = 1 << 7,
	RTC_ERROR = 1 << 8
}error_flags_t;

typedef enum led_sequence{
	INITIAL_LED_SEQUENCE = 1,
	TEST_PASSED_LED_SEQUENCE = 2,
	TEST_NON_CRITICAL_FAULT_LED_SEQUENCE = 3,
	TEST_CRITICAL_FAULT_LED_SEQUENCE = 4
}led_sequence_t;

typedef enum self_test_status{
	SELF_TEST_PASSED = 0,
	SELF_TEST_NON_CRITICAL_FAULT = 1,
	SELF_TEST_CRITICAL_FAULT = 2
}self_test_status_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define THREAD_EXTRA_LARGE_STACK_SIZE 4096
#define THREAD_LARGE_STACK_SIZE 2048
#define THREAD_MEDIUM_STACK_SIZE 1024
#define THREAD_SMALL_STACK_SIZE 512
#define THREAD_EXTRA_SMALL_STACK_SIZE 256

// The max times we'll try to get a single peripheral up before sending reset vector
#define MAX_SELF_TEST_RETRIES 3
// The maximum amount of time (in milliseconds) a sample window could take
#define MAX_ALLOWABLE_WINDOW_TIME_IN_MINUTES 60
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */
void MX_ThreadX_Init(device_handles_t *handles);
void watchdog_thread_entry(ULONG thread_input);
void startup_thread_entry(ULONG thread_input);
void gnss_thread_entry(ULONG thread_input);
void waves_thread_entry(ULONG thread_input);
void iridium_thread_entry(ULONG thread_input);
void end_of_cycle_thread_entry(ULONG thread_input);
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
