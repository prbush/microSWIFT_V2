/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DBUG
#undef DBUG
// For testing purposed, use the Nucleo onboard LEDs
#define NUCLEO_LIGHT_SHOW
// #undef NUCLEO_LIGHT_SHOW TODO: uncomment this
#include "configuration.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	RTC_HandleTypeDef* hrtc;
	UART_HandleTypeDef* CT_uart;
	UART_HandleTypeDef* Iridium_uart;
	UART_HandleTypeDef* GNSS_uart;
	DMA_HandleTypeDef* CT_dma_handle;
	DMA_HandleTypeDef* GNSS_dma_handle;
	DMA_HandleTypeDef* Iridium_tx_dma_handle;
	DMA_HandleTypeDef* Iridium_rx_dma_handle;
	TIM_HandleTypeDef* iridium_timer;
	IWDG_HandleTypeDef* watchdog_handle;
	uint32_t reset_reason;
} device_handles_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define EXT_LED_RED_Pin GPIO_PIN_0
#define EXT_LED_RED_GPIO_Port GPIOF
#define EXT_LED_GREEN_Pin GPIO_PIN_1
#define EXT_LED_GREEN_GPIO_Port GPIOF
#define BUS_5V_FET_Pin GPIO_PIN_2
#define BUS_5V_FET_GPIO_Port GPIOF
#define GNSS_UART_TX_Pin GPIO_PIN_2
#define GNSS_UART_TX_GPIO_Port GPIOA
#define GNSS_UART_RX_Pin GPIO_PIN_3
#define GNSS_UART_RX_GPIO_Port GPIOA
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define UCPD1_CC2_Pin GPIO_PIN_15
#define UCPD1_CC2_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOG
#define CT_FET_Pin GPIO_PIN_3
#define CT_FET_GPIO_Port GPIOG
#define GNSS_FET_Pin GPIO_PIN_4
#define GNSS_FET_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define USB_OTG_FS_DM_Pin GPIO_PIN_11
#define USB_OTG_FS_DM_GPIO_Port GPIOA
#define USB_OTG_FS_DP_Pin GPIO_PIN_12
#define USB_OTG_FS_DP_GPIO_Port GPIOA
#define UCPD1_CC1_Pin GPIO_PIN_15
#define UCPD1_CC1_GPIO_Port GPIOA
#define CT_UART_TX_Pin GPIO_PIN_10
#define CT_UART_TX_GPIO_Port GPIOC
#define CT_UART_RX_Pin GPIO_PIN_11
#define CT_UART_RX_GPIO_Port GPIOC
#define IRIDIUM_UART_TX_Pin GPIO_PIN_12
#define IRIDIUM_UART_TX_GPIO_Port GPIOC
#define IMU_FET_Pin GPIO_PIN_0
#define IMU_FET_GPIO_Port GPIOD
#define IRIDIUM_FET_Pin GPIO_PIN_1
#define IRIDIUM_FET_GPIO_Port GPIOD
#define IRIDIUM_UART_RX_Pin GPIO_PIN_2
#define IRIDIUM_UART_RX_GPIO_Port GPIOD
#define RF_SWITCH_VCTL_Pin GPIO_PIN_3
#define RF_SWITCH_VCTL_GPIO_Port GPIOD
#define RF_SWITCH_EN_Pin GPIO_PIN_4
#define RF_SWITCH_EN_GPIO_Port GPIOD
#define IRIDIUM_NetAv_Pin GPIO_PIN_5
#define IRIDIUM_NetAv_GPIO_Port GPIOD
#define IRIDIUM_RI_Pin GPIO_PIN_6
#define IRIDIUM_RI_GPIO_Port GPIOD
#define IRIDIUM_OnOff_Pin GPIO_PIN_7
#define IRIDIUM_OnOff_GPIO_Port GPIOD
#define GNSS_EXTINT_Pin GPIO_PIN_9
#define GNSS_EXTINT_GPIO_Port GPIOG
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOG
#define IMU_nRESET_Pin GPIO_PIN_12
#define IMU_nRESET_GPIO_Port GPIOG
#define UCPD_DBn_Pin GPIO_PIN_5
#define UCPD_DBn_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* Size of Reception buffer */
#define RX_BUFFER_SIZE   512
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
