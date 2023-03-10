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
	IRIDIUM_SELF_TEST_FAILED = -4,
	IRIDIUM_RECEIVE_ERROR = -5,
	IRIDIUM_FLASH_STORAGE_ERROR = -6
} iridium_error_code_t;

// Macros
// TODO: define GPIO pins
#define MAX_TRANSMIT_TIME 600
#define MAX_RETRIES 10
#define ACK_MESSAGE_SIZE 9
#define ACK_MAX_WAIT_TIME 100
#define IRIDIUM_MESSAGE_PAYLOAD_SIZE 340
#define IRIDIUM_FET_PIN 0
#define IRIDIUM_NETAV_PIN 0
#define IRIDIUM_RI_PIN 0
#define IRIDIUM_ONOFF_PIN 0
#define IRIDIUM_DEFAULT_BAUD_RATE 19200
// TODO: figure out a good value for this
#define IRIDIUM_CAP_CHARGE_TIME 30000

typedef struct Iridium {
	// The UART and DMA handle for the Iridium interface
	UART_HandleTypeDef* iridium_uart_handle;
	DMA_HandleTypeDef* iridium_rx_dma_handle;
	DMA_HandleTypeDef* iridium_tx_dma_handle;
	// ThreadX timer (tick-based granularity is fine for this)
	TIM_HandleTypeDef* ten_min_timer;
	// Event flags
	TX_EVENT_FLAGS_GROUP* event_flags;
	// pointer to the message array
	uint8_t* message_buffer;
	// pointer to the response array
	uint8_t* response_buffer;
	// current lat/long
	int32_t current_lat;
	int32_t current_long;
	// Next available flash page
	uint32_t current_flash_page;
	// How many times we've tried to transmit the current message
	uint32_t current_message_transmit_attempts;

	iridium_error_code_t (*config)(struct Iridium* self);
	iridium_error_code_t (*self_test)(struct Iridium* self);
	iridium_error_code_t (*transmit_message)(struct Iridium* self);
	iridium_error_code_t (*get_location)(struct Iridium* self);
	iridium_error_code_t (*sleep)(struct Iridium* self);
	iridium_error_code_t (*store_in_flash)(struct Iridium* self);
	iridium_error_code_t (*reset_uart)(struct Iridium* self, uint16_t baud_rate);
} Iridium;

//TODO: fill out this struct for making a bastardized priority queue in flash
typedef struct Iridium_Flash_Message {
	uint8_t* message_payload;
	uint32_t payload_size;
	// TODO: maybe allow for negative numbers to indicate bas messages?
	int32_t message_priority;
	struct Iridium_Flash_Message* next_message;
	uint32_t next_message_page;
}Iridium_Flash_Message;

/* Function declarations */
void iridium_init(Iridium* self, UART_HandleTypeDef* iridium_uart_handle,
		DMA_HandleTypeDef* iridium_rx_dma_handle, TIM_HandleTypeDef* ten_min_timer,
		DMA_HandleTypeDef* iridium_tx_dma_handle,TX_EVENT_FLAGS_GROUP* event_flags,
		uint8_t* message_buffer, uint8_t* response_buffer);
iridium_error_code_t iridium_config(Iridium* self);
iridium_error_code_t iridium_self_test(Iridium* self);
iridium_error_code_t iridium_transmit_message(Iridium* self);
iridium_error_code_t iridium_get_location(Iridium* self);
iridium_error_code_t iridium_sleep(Iridium* self);
iridium_error_code_t iridium_store_in_flash(Iridium* self);
iridium_error_code_t iridium_reset_iridium_uart(Iridium* self, uint16_t baud_rate);

#endif /* SRC_IRIDIUM_H_ */
