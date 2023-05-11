/*
 * gnss.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 *
 */

#ifndef SRC_GPS_H_
#define SRC_GPS_H_

#include "byte_array.h"
#include "app_threadx.h"
#include "tx_api.h"
#include "tx_user.h"
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
typedef enum gnss_error_code{
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
	GNSS_MESSAGE_PROCESS_ERROR = -11,
	GNSS_RTC_ERROR = -12,
	GNSS_TIMER_ERROR = -13,
	GNSS_TIME_RESOLUTION_ERROR = -14,
	GNSS_FIRST_SAMPLE_RESOLUTION_ERROR = -15
} gnss_error_code_t;

// Macros
#define CONFIG_BUFFER_SIZE 600
#define MAX_POSSIBLE_VELOCITY 10000	// 10000 mm/s = 10 m/s
#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define UBX_NAV_PVT_MESSAGE_LENGTH 100
#define UBX_MESSAGE_SIZE (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)
#define UBX_BUFFER_SIZE (10 * UBX_MESSAGE_SIZE)
#define INITIAL_STAGES_BUFFER_SIZE 500
#define UBX_NAV_PVT_PAYLOAD_LENGTH 92
#define UBX_ACK_MESSAGE_LENGTH 10
#define MAX_ACCEPTABLE_SACC 250 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 1000 // (units = 0.01) greater than 10 means fair fix accuracy
#define MAX_EMPTY_QUEUE_WAIT 50 // wait for max 50ms
#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins
#define GNSS_DEFAULT_BAUD_RATE 9600
#define MAX_THREADX_WAIT_TICKS_FOR_CONFIG 7
#define ONE_SECOND 1000
#define MILLISECONDS_PER_MINUTE 60000
#define MM_PER_METER 1000.0
#define MIN_SATELLITES_TO_PASS_TEST 4
#define LOWER_4_BITS_MASK 0xF
#define LAT_LON_CONVERSION_FACTOR 10000000 // format as 1E-7
// UBX message definitions
#define FULLY_RESOLVED_AND_VALID_TIME 0x7
#define UBX_NAV_PVT_YEAR_INDEX 4
#define UBX_NAV_PVT_MONTH_INDEX 6
#define UBX_NAV_PVT_DAY_INDEX 7
#define UBX_NAV_PVT_HOUR_INDEX 8
#define UBX_NAV_PVT_MINUTE_INDEX 9
#define UBX_NAV_PVT_SECONDS_INDEX 10
#define UBX_NAV_PVT_VALID_FLAGS_INDEX 11
#define UBX_NAV_PVT_TACC_INDEX 12
#define UBX_NAV_PVT_VALID_FLAGS2_INDEX 22
#define UBX_NAV_PVT_NUMSV_INDEX 23
#define UBX_NAV_PVT_LON_INDEX 24
#define UBX_NAV_PVT_LAT_INDEX 28
#define UBX_NAV_PVT_HACC_INDEX 40
#define UBX_NAV_PVT_VACC_INDEX 44
#define UBX_NAV_PVT_V_NORTH_INDEX 48
#define UBX_NAV_PVT_V_EAST_INDEX 52
#define UBX_NAV_PVT_V_DOWN_INDEX 56
#define UBX_NAV_PVT_SACC_INDEX 68
#define UBX_NAV_PVT_PDOP_INDEX 76

// GNSS struct definition -- packed for good organization, not memory efficiency
typedef struct GNSS {
	// Our global configuration struct
	microSWIFT_configuration* global_config;
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* gnss_uart_handle;
	DMA_HandleTypeDef* gnss_dma_handle;
	// Handle to the RTC
	RTC_HandleTypeDef* rtc_handle;
	// Event flags
	TX_EVENT_FLAGS_GROUP* control_flags;
	TX_EVENT_FLAGS_GROUP* error_flags;
	// UBX message process buffer filled from DMA ISR
	uint8_t* ubx_process_buf;
	// Velocity sample array pointers
	float* GNSS_N_Array;
	float* GNSS_E_Array;
	float* GNSS_D_Array;
	// Number of messages processed in a given buffer
	int16_t messages_processed;
	// Keep a running track of sum -- to be used in getRunningAverage
	int32_t v_north_sum;
	int32_t v_east_sum;
	int32_t v_down_sum;
	// Hold the current lat/long for whatever we might need it for (modem)
	int32_t current_latitude;
	int32_t current_longitude;
	// The start time for the sampling window
	uint32_t sample_window_start_time;
	// The start time for the sampling window
	uint32_t sample_window_stop_time;
	// The true calculated sample window frequency
	double sample_window_freq;
	// Increment with each sample or running average
	uint16_t total_samples;
	// We'll keep track of how many times we had to sub in a running average
	uint16_t total_samples_averaged;
	// How many times we've had to skip a sample - gets reset with valid data
	uint16_t number_cycles_without_data;
	// Flags
	bool current_fix_is_good;
	bool all_resolution_stages_complete;
	bool is_configured;
	bool is_clock_set;
	bool rtc_error;
	bool all_samples_processed;
	// Function pointers
	gnss_error_code_t (*config)(struct GNSS* self);
	gnss_error_code_t (*sync_and_start_reception)(struct GNSS* self,
				gnss_error_code_t (*start_dma)(struct GNSS*, uint8_t*, size_t),
				uint8_t* buffer, size_t msg_size);
	gnss_error_code_t (*get_location)(struct GNSS* self, float* latitude,
			float* longitude);
	gnss_error_code_t (*get_running_average_velocities)(struct GNSS* self);
	void		 	  (*process_message)(struct GNSS* self);
	gnss_error_code_t (*sleep)(struct GNSS* self, bool put_to_sleep);
	void			  (*on_off)(struct GNSS* self, GPIO_PinState pin_state);
	void			  (*cycle_power)(struct GNSS* self);
	gnss_error_code_t (*set_rtc)(struct GNSS* self, uint8_t* msg_payload);
	gnss_error_code_t (*reset_uart)(struct GNSS* self, uint16_t baud_rate);
} GNSS;

/* Function declarations */
void gnss_init(GNSS* self, microSWIFT_configuration* global_config,
		UART_HandleTypeDef* gnss_uart_handle, DMA_HandleTypeDef* gnss_dma_handle,
		TX_EVENT_FLAGS_GROUP* control_flags, TX_EVENT_FLAGS_GROUP* error_flags,
		uint8_t* ubx_process_buf, RTC_HandleTypeDef* rtc_handle,
		float* GNSS_N_Array, float* GNSS_E_Array, float* GNSS_D_Array);
gnss_error_code_t gnss_config(GNSS* self);
gnss_error_code_t gnss_sync_and_start_reception(GNSS* self, gnss_error_code_t (*start_dma)(GNSS*, uint8_t*, size_t),
		uint8_t* buffer, size_t msg_size);
gnss_error_code_t gnss_get_location(GNSS* self, float* latitude, float* longitude);
gnss_error_code_t gnss_get_running_average_velocities(GNSS* self);
void 			  gnss_process_message(GNSS* self);
gnss_error_code_t gnss_sleep(GNSS* self, bool put_to_sleep);
void			  gnss_on_off(GNSS* self, GPIO_PinState pin_state);
void			  gnss_cycle_power(GNSS* self);
gnss_error_code_t gnss_set_rtc(GNSS* self, uint8_t* msg_payload);
gnss_error_code_t reset_gnss_uart(GNSS* self, uint16_t baud_rate);



#endif /* SRC_GPS_H_ */
