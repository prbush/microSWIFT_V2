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
	GNSS_FIRST_SAMPLE_RESOLUTION_ERROR = -15,
	GNSS_NAK_MESSAGE_RECEIVED = -16,
	GNSS_HIGH_PERFORMANCE_ENABLE_ERROR = -17,
	GNSS_DONE_SAMPLING = -18
} gnss_error_code_t;

// Macros
#define CONFIG_BUFFER_SIZE 600
#define CONFIGURATION_ARRAY_SIZE 164
#define MAX_POSSIBLE_VELOCITY 10000	// 10000 mm/s = 10 m/s
#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define UBX_NAV_PVT_MESSAGE_LENGTH 100
#define UBX_CFG_VALSET_CLASS 0x06
#define UBX_CFG_VALSET_ID 0x8A
#define UBX_MESSAGE_SIZE (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)
#define UBX_BUFFER_SIZE (10 * UBX_MESSAGE_SIZE)
#define INITIAL_STAGES_BUFFER_SIZE 500
#define FRAME_SYNC_RX_SIZE 200
#define UBX_NAV_PVT_PAYLOAD_LENGTH 92
#define UBX_ACK_MESSAGE_LENGTH 10
#define MAX_ACCEPTABLE_SACC 250 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 1000 // (units = 0.01) greater than 10 means fair fix accuracy
#define MAX_EMPTY_QUEUE_WAIT 50 // wait for max 50ms
#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins
#define MAX_FRAME_SYNC_ATTEMPTS 3
#define MAX_CONFIG_STEP_ATTEMPTS 3
#define GNSS_DEFAULT_BAUD_RATE 9600
#define MAX_THREADX_WAIT_TICKS_FOR_CONFIG (TX_TIMER_TICKS_PER_SECOND + (TX_TIMER_TICKS_PER_SECOND / 4))
#define ONE_SECOND 1000
#define MILLISECONDS_PER_MINUTE 60000
#define MM_PER_METER 1000.0
#define MIN_SATELLITES_TO_PASS_TEST 4
#define LOWER_4_BITS_MASK 0xF
#define LAT_LON_CONVERSION_FACTOR 10000000 // format as 1E-7
#define GNSS_TIMER_INSTANCE TIM16
// UBX message definitions
#define RESOLVED_TIME_BITS ((1 << 0) | (1 << 2)) // Only resolve to the time of day
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
#define UBX_ACK_ACK_CLSID_INDEX 0
#define UBX_ACK_ACK_MSGID_INDEX 1
#define HIGH_PERFORMANCE_QUERY_SIZE 28
#define HIGH_PERFORMANCE_RESPONSE_SIZE 36
#define ENABLE_HIGH_PERFORMANCE_SIZE 60

// GNSS struct definition -- packed for good organization, not memory efficiency
typedef struct GNSS {
	// Our global configuration struct
	microSWIFT_configuration* global_config;
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* gnss_uart_handle;
	DMA_HandleTypeDef* gnss_rx_dma_handle;
	DMA_HandleTypeDef* gnss_tx_dma_handle;
	// Handle to the RTC
	RTC_HandleTypeDef* rtc_handle;
	// Event flags
	TX_EVENT_FLAGS_GROUP* control_flags;
	TX_EVENT_FLAGS_GROUP* error_flags;
	// Pointer to hardware timer handle
	TIM_HandleTypeDef* minutes_timer;
	// UBX message process buffer filled from DMA ISR
	uint8_t* ubx_process_buf;
	// Configuration response buffer
	uint8_t* config_response_buf;
	// Velocity sample array pointers
	float* GNSS_N_Array;
	float* GNSS_E_Array;
	float* GNSS_D_Array;
	// Number of messages processed in a given buffer
	uint32_t messages_processed;
	// Keep a running track of sum -- to be used in getRunningAverage
	// NOTE: These values are in units of mm/s and must be converted
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
	bool timer_timeout;
	// Function pointers
	gnss_error_code_t (*config)(void);
	gnss_error_code_t (*sync_and_start_reception)(
				gnss_error_code_t (*start_dma)(struct GNSS*, uint8_t*, size_t),
				uint8_t* buffer, size_t msg_size);
	gnss_error_code_t (*get_location)(float* latitude, float* longitude);
	gnss_error_code_t (*get_running_average_velocities)(void);
	void		 	  (*process_message)(void);
	gnss_error_code_t (*sleep)(bool put_to_sleep);
	void			  (*on_off)(GPIO_PinState pin_state);
	void			  (*cycle_power)(void);
	gnss_error_code_t (*set_rtc)(uint8_t* msg_payload);
	gnss_error_code_t (*reset_uart)(uint16_t baud_rate);
	gnss_error_code_t (*reset_timer)(uint16_t timeout_in_minutes);
} GNSS;

/* Function declarations */
void gnss_init(GNSS* struct_ptr, microSWIFT_configuration* global_config,
		UART_HandleTypeDef* gnss_uart_handle, DMA_HandleTypeDef* gnss_rx_dma_handle,
		DMA_HandleTypeDef* gnss_tx_dma_handle, TX_EVENT_FLAGS_GROUP* control_flags,
		TX_EVENT_FLAGS_GROUP* error_flags, TIM_HandleTypeDef* timer, uint8_t* ubx_process_buf,
		uint8_t* config_response_buffer, RTC_HandleTypeDef* rtc_handle, float* GNSS_N_Array,
		float* GNSS_E_Array, float* GNSS_D_Array);
// watchdog refresh function
extern void 	  register_watchdog_refresh();



#endif /* SRC_GPS_H_ */
