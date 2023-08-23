/*
 * Iridium.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_IRIDIUM_H_
#define SRC_IRIDIUM_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stdint.h"
#include "NEDWaves/rtwhalf.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_tim.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"

// Return codes
typedef enum iridium_error_code{
	IRIDIUM_SUCCESS = 0,
	IRIDIUM_UNKNOWN_ERROR = -1,
	IRIDIUM_UART_ERROR = -2,
	IRIDIUM_TRANSMIT_ERROR = -3,
	IRIDIUM_COMMAND_RESPONSE_ERROR = -4,
	IRIDIUM_SELF_TEST_FAILED = -5,
	IRIDIUM_RECEIVE_ERROR = -6,
	IRIDIUM_FLASH_STORAGE_ERROR = -7,
	IRIDIUM_STORAGE_QUEUE_FULL = -8,
	IRIDIUM_STORAGE_QUEUE_EMPTY = -9,
	IRIDIUM_TIMER_ERROR = -10,
	IRIDIUM_TRANSMIT_TIMEOUT = -11,
	IRIDIUM_TRANSMIT_UNSUCCESSFUL = -12
} iridium_error_code_t;

// Macros
#define IRIDIUM_INITIAL_CAP_CHARGE_TIME TX_TIMER_TICKS_PER_SECOND * 30
#define IRIDIUM_TOP_UP_CAP_CHARGE_TIME TX_TIMER_TICKS_PER_SECOND * 10
#define MAX_RETRIES 10
#define ACK_MESSAGE_SIZE 9
#define DISABLE_FLOW_CTRL_SIZE 12
#define ENABLE_RI_SIZE 18
#define STORE_CONFIG_SIZE 12
#define SELECT_PWR_UP_SIZE 12
#define SBDWB_READY_RESPONSE_SIZE 22
#define SBDWB_LOAD_RESPONSE_SIZE 11
#define SBDWB_RESPONSE_CODE_INDEX 2
#define SBDI_RESPONSE_CODE_INDEX 16
#define SBDI_RESPONSE_SIZE 19
#define SBDD_RESPONSE_SIZE 20
#define SBDIX_WAIT_TIME ONE_SECOND * 13
#define IRIDIUM_MESSAGE_PAYLOAD_SIZE 327
#define IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE 333
#define ERROR_MESSAGE_TYPE 99
#define IRIDIUM_CHECKSUM_LENGTH 2
#define IRIDIUM_MAX_RESPONSE_SIZE 64
#define IRIDIUM_FET_PIN 0
#define IRIDIUM_NETAV_PIN 0
#define IRIDIUM_RI_PIN 0
#define IRIDIUM_ONOFF_PIN 0
#define IRIDIUM_DEFAULT_BAUD_RATE 19200
#define CHECKSUM_FIRST_BYTE_INDEX 327
#define CHECKSUM_SECOND_BYTE_INDEX 328
#define ONE_SECOND 1000
#define ASCII_ZERO 48
#define ASCII_FIVE 53
#define IRIDIUM_MAX_TRANSMIT_PERIOD 6 - 1
#define MAX_NUM_MSGS_STORED 24 * 7
#define ERROR_MESSAGE_MAX_LENGTH 320
#define IRIDIUM_TIMER_INSTANCE TIM17
#define IRIDIUM_UART_INSTANCE UART5
#define IRIDIUM_LL_TX_DMA_HANDLE LL_DMA_CHANNEL_2
#define IRIDIUM_LL_RX_DMA_HANDLE LL_DMA_CHANNEL_3
#define SECONDS_IN_MIN 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000 // 365 day year, not accounting for 1/4 leap days
#define SECONDS_1970_TO_2000 946684800
#define EPOCH_YEAR 1970
#define CURRENT_CENTURY 2000
// Magic number used to indicate the SBD message queue has been initialized
#define SBD_QUEUE_MAGIC_NUMBER 0x101340

typedef struct sbd_message_type_52 {
		 char			legacy_number_7;
		 uint8_t 		type;
		 uint8_t 		port;
__packed uint16_t 		size;
__packed real16_T 		Hs;
__packed real16_T		Tp;
__packed real16_T 		Dp;
__packed real16_T	 	E_array[42];
__packed real16_T		f_min;
__packed real16_T		f_max;
		 signed char	a1_array[42];
		 signed char	b1_array[42];
		 signed char	a2_array[42];
		 signed char	b2_array[42];
		 unsigned char	cf_array[42];
__packed float			Lat;
__packed float			Lon;
__packed real16_T		mean_temp;
__packed real16_T		mean_salinity;
__packed real16_T		mean_voltage;
__packed float			timestamp;
		 uint8_t		checksum_a;
		 uint8_t		checksum_b;
} sbd_message_type_52;

typedef struct Iridium {
	// Our global configuration struct
	microSWIFT_configuration* global_config;
	// The UART and DMA handle for the Iridium interface
	UART_HandleTypeDef* iridium_uart_handle;
	DMA_HandleTypeDef* iridium_rx_dma_handle;
	DMA_HandleTypeDef* iridium_tx_dma_handle;
	// Pointer to hardware timer handle
	TIM_HandleTypeDef* timer;
	// Event flags
	TX_EVENT_FLAGS_GROUP* control_flags;
	TX_EVENT_FLAGS_GROUP* error_flags;
	// Handle to the RTC
	RTC_HandleTypeDef* rtc_handle;
	// pointer to the message array
	sbd_message_type_52* current_message;
	// Pointer to the error message payload buffer
	uint8_t* error_message_buffer;
	// pointer to the response array
	uint8_t* response_buffer;
	// Unsent message storage queue
	struct Iridium_message_storage* storage_queue;
	// current lat/long (for future use)
	float current_lat;
	float current_lon;

	iridium_error_code_t (*config)(void);
	void				 (*charge_caps)(uint32_t caps_charge_time_ticks);
	iridium_error_code_t (*self_test)(void);
	iridium_error_code_t (*transmit_message)(void);
	iridium_error_code_t (*transmit_error_message)(char* error_message);
	float				 (*get_timestamp)(void);
	void 				 (*sleep)(GPIO_PinState pin_state);
	void				 (*on_off)(GPIO_PinState pin_state);
	void				 (*cycle_power)(void);
	iridium_error_code_t (*store_in_flash)(void);
	iridium_error_code_t (*reset_uart)(uint16_t baud_rate);
	iridium_error_code_t (*reset_timer)(uint8_t timeout_in_minutes);
	iridium_error_code_t (*queue_add)(sbd_message_type_52* payload);
	iridium_error_code_t (*queue_get)(uint8_t* msg_index);
	void                 (*queue_flush)(void);

	bool timer_timeout;
	bool skip_current_message;
} Iridium;

typedef struct Iridium_message {
	sbd_message_type_52 payload;
	bool valid;
}Iridium_message;

typedef struct Iridium_message_storage {
	// Store 7 days worth of messages
	Iridium_message msg_queue [MAX_NUM_MSGS_STORED];
	uint8_t num_msgs_enqueued;
	uint64_t magic_number;
}Iridium_message_storage;



/* Function declarations */
void iridium_init(Iridium* struct_ptr, microSWIFT_configuration* global_config,
		UART_HandleTypeDef* iridium_uart_handle, DMA_HandleTypeDef* iridium_rx_dma_handle,
		TIM_HandleTypeDef* timer, DMA_HandleTypeDef* iridium_tx_dma_handle,
		TX_EVENT_FLAGS_GROUP* control_flags, TX_EVENT_FLAGS_GROUP* error_flags,
		RTC_HandleTypeDef* rtc_handle, sbd_message_type_52* current_message,
		uint8_t* error_message_buffer, uint8_t* response_buffer,
		Iridium_message_storage* storage_queue);
// watchdog refresh function
extern void 		 register_watchdog_refresh();

#endif /* SRC_IRIDIUM_H_ */
