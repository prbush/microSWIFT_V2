/*
 * gnss.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 *
 *  TODO:Define error code macros
 *       Define macros
 *       Finish setting RTC time in processMessage
 *       Finish setting Lat/Long in processMessage
 *       Figure out power pins -- this will be tied to board design
 *       TIMER!! need a dedicated timer
 *       Remove testFunct in production
 */

#ifndef SRC_GPS_H_
#define SRC_GPS_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"
#include "u_ubx_protocol.h"
#include "u_error_common.h"

// Return codes
typedef enum {
	// Error/ success codes
	GNSS_SUCCESS = 0,
	GNSS_UNKNOWN_ERROR = -1,
	GNSS_LOCATION_INVALID = -2,
	GNSS_VELOCITY_INVALID = -3,
	GNSS_NO_SAMPLES_ERROR = -4,
	GNSS_TIMEOUT_ERROR = -5,
	GNSS_BUSY_ERROR = -6,
	GNSS_NO_MESSAGE_RECEIVED = -7,
	GNSS_UART_ERROR = -8,
	GNSS_CONFIG_ERROR = -9,
	GNSS_SELF_TEST_FAILED = -10,
	GNSS_MESSAGE_PROCESS_ERROR = -11
} gnss_error_code_t;

// Macros
// TODO: define GPIO pins
#define MAX_POSSIBLE_VELOCITY 10000	// 10 m/s
#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define UBX_NAV_PVT_MESSAGE_LENGTH 100
#define UBX_MESSAGE_SIZE (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)
#define UBX_BUFFER_SIZE (10 * UBX_MESSAGE_SIZE)
#define SELF_TEST_BUFFER_SIZE 500
#define UBX_NAV_PVT_PAYLOAD_LENGTH 92
#define UBX_ACK_MESSAGE_LENGTH 10
#define MAX_ACCEPTABLE_TACC 50 // TODO: figure out a good value for this
#define MAX_ACCEPTABLE_SACC 100 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 600 // scale is 0.01, max acceptable is 6.0
#define MAX_EMPTY_QUEUE_WAIT 50 // wait for max 50ms
#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins

typedef struct GNSS {
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* gnss_uart_handle;
	DMA_HandleTypeDef* gnss_dma_handle;
	// Event flags
	TX_EVENT_FLAGS_GROUP* event_flags;
	// UBX message queue filled from DMA ISR
	TX_QUEUE* message_queue;
	// Velocity sample array pointers
	int16_t* GNSS_N_Array;
	int16_t* GNSS_E_Array;
	int16_t* GNSS_D_Array;
	// Number of messages processed in a given buffer
	int16_t messages_processed;
	// Keep a running track of sum -- to be used in getRunningAverage
	int64_t v_north_sum;
	int64_t v_east_sum;
	int64_t v_down_sum;
	// Hold the current lat/long for whatever we might need it for (modem)
	int32_t current_latitude;
	int32_t current_longitude;
	// Increment with each sample or running average
	uint16_t total_samples;
	// We'll keep track of how many times we had to sub in a running average
	uint16_t total_samples_averaged; // Just do a %10 in the end
	// How many times we've had to skip a sample - gets reset with valid data
	uint16_t number_cycles_without_data;
	// Flags
	bool is_configured;
	bool is_location_valid;
	bool is_velocity_valid;
	bool is_clock_set;
	// Function pointers
	gnss_error_code_t (*config)(struct GNSS* self);
	gnss_error_code_t (*self_test)(struct GNSS* self);
	gnss_error_code_t (*get_location)(struct GNSS* self, int32_t* latitude,
			int32_t* longitude);
	gnss_error_code_t (*get_running_average_velocities)(struct GNSS* self);
	gnss_error_code_t (*gnss_process_message)(struct GNSS* self);
	gnss_error_code_t (*sleep)(struct GNSS* self, bool put_to_sleep);
	void			  (*on_off)(struct GNSS* self, bool on);
	gnss_error_code_t (*reset_gnss_uart)(struct GNSS* self, uint16_t baud_rate);
} GNSS;

/* Function declarations */
void gnss_init(GNSS* self, UART_HandleTypeDef* gnss_uart_handle,
		DMA_HandleTypeDef* gnss_dma_handle, TX_EVENT_FLAGS_GROUP* event_flags,
		TX_QUEUE* message_queue, int16_t* GNSS_N_Array, int16_t* GNSS_E_Array,
		int16_t* GNSS_D_Array);
gnss_error_code_t gnss_config(GNSS* self);
gnss_error_code_t gnss_self_test(GNSS* self);
gnss_error_code_t gnss_get_location(GNSS* self, int32_t* latitude, int32_t* longitude);
gnss_error_code_t gnss_get_running_average_velocities(GNSS* self);
gnss_error_code_t gnss_process_message(GNSS* self);
gnss_error_code_t gnss_sleep(GNSS* self, bool put_to_sleep);
void			  gnss_on_off(GNSS* self, bool on);
gnss_error_code_t reset_gnss_uart(GNSS* self, uint16_t baud_rate);



#endif /* SRC_GPS_H_ */
