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
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_tim.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"

// Return codes
typedef enum {
	// Error/ success codes
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
	IRIDIUM_TIMER_ERROR = -10
} iridium_error_code_t;

// Macros
// TODO: define GPIO pins
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
#define IRIDIUM_CHECKSUM_LENGTH 2
#define IRIDIUM_MAX_RESPONSE_SIZE 128
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
// TODO: figure out a good value for this
#define IRIDIUM_CAP_CHARGE_TIME 25000
// number of mins to try to get a message off, starts at 0, so to get 10, you actually use 9
#define IRIDIUM_MAX_TRANSMIT_PERIOD 6 - 1
#define MAX_SRAM4_MESSAGES 16384 / (IRIDIUM_MESSAGE_PAYLOAD_SIZE + 1)
#define STORAGE_QUEUE_SIZE MAX_SRAM4_MESSAGES * (IRIDIUM_MESSAGE_PAYLOAD_SIZE  + 1)
#define SRAM4_START_ADDR 0x28000000
#define ERROR_MESSAGE_MAX_LENGTH 320
#define IRIDIUM_TIMER_INSTANCE TIM17
#define IRIDIUM_UART_INSTANCE UART5
#define IRIDIUM_LL_TX_DMA_HANDLE LL_DMA_CHANNEL_2
#define IRIDIUM_LL_RX_DMA_HANDLE LL_DMA_CHANNEL_3

typedef struct Iridium {
	// The UART and DMA handle for the Iridium interface
	UART_HandleTypeDef* iridium_uart_handle;
	DMA_HandleTypeDef* iridium_rx_dma_handle;
	DMA_HandleTypeDef* iridium_tx_dma_handle;
	// ThreadX timer (tick-based granularity is fine for this)
	TIM_HandleTypeDef* timer;
	// Event flags
	TX_EVENT_FLAGS_GROUP* event_flags;
	// pointer to the message array
	uint8_t* message_buffer;
	// pointer to the response array
	uint8_t* response_buffer;

	// Unsent message storage queue
	struct Iridium_message_queue* storage_queue;
	// current lat/long
	int32_t current_lat;
	int32_t current_long;

	iridium_error_code_t (*config)(struct Iridium* self);
	iridium_error_code_t (*self_test)(struct Iridium* self);
	iridium_error_code_t (*transmit_message)(struct Iridium* self);
	iridium_error_code_t (*transmit_error_message)(struct Iridium* self, char* error_message);
	iridium_error_code_t (*get_location)(struct Iridium* self);
	void 				 (*on_off)(struct Iridium* self, bool on);
	iridium_error_code_t (*store_in_flash)(struct Iridium* self);
	iridium_error_code_t (*reset_uart)(struct Iridium* self, uint16_t baud_rate);
	iridium_error_code_t (*reset_timer)(struct Iridium* self, uint8_t timeout_in_minutes);
	void				 (*queue_create)(struct Iridium* self);
	iridium_error_code_t (*queue_add)(struct Iridium* self, uint8_t* payload);
	iridium_error_code_t (*queue_get)(struct Iridium* self, uint8_t* msg_index);
	void                 (*queue_flush)(struct Iridium* self);

	bool timer_timeout;
} Iridium;

typedef struct Iridium_message {
	uint8_t payload[IRIDIUM_MESSAGE_PAYLOAD_SIZE];
	bool valid;
}Iridium_message;

typedef struct Iridium_message_queue {
	Iridium_message msg_queue [MAX_SRAM4_MESSAGES];
//	uint8_t msg_queue [MAX_SRAM4_MESSAGES][IRIDIUM_MESSAGE_PAYLOAD_SIZE];
	uint8_t num_msgs_enqueued;
}Iridium_message_queue;



/* Function declarations */
void iridium_init(Iridium* self, UART_HandleTypeDef* iridium_uart_handle,
		DMA_HandleTypeDef* iridium_rx_dma_handle, TIM_HandleTypeDef* timer,
		DMA_HandleTypeDef* iridium_tx_dma_handle,TX_EVENT_FLAGS_GROUP* event_flags,
		uint8_t* message_buffer, uint8_t* response_buffer);
iridium_error_code_t iridium_config(Iridium* self);
iridium_error_code_t iridium_self_test(Iridium* self);
iridium_error_code_t iridium_transmit_message(Iridium* self);
iridium_error_code_t iridium_transmit_error_message(Iridium* self, char* error_message);
iridium_error_code_t iridium_get_location(Iridium* self);
void				 iridium_on_off(Iridium* self, bool on);
iridium_error_code_t iridium_store_in_flash(Iridium* self);
iridium_error_code_t iridium_reset_iridium_uart(Iridium* self, uint16_t baud_rate);
iridium_error_code_t iridium_reset_timer(Iridium* self, uint8_t timeout_in_minutes);
void				 iridium_storage_queue_create(Iridium* self);
iridium_error_code_t iridium_storage_queue_add(Iridium* self,uint8_t* payload);
iridium_error_code_t iridium_storage_queue_get(Iridium* self,uint8_t* msg_index);
void                 iridium_storage_queue_flush(Iridium* self);


#endif /* SRC_IRIDIUM_H_ */
