/*
 * GPS.h
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
#include "gnss_error_codes.h"
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

// Macros
#define MAX_POSSIBLE_VELOCITY 10000	// 10 m/s

#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define UBX_NAV_PVT_MESSAGE_LENGTH 100
#define UBX_NAV_PVT_PAYLOAD_LENGTH 92
#define UBX_ACK_MESSAGE_LENGTH 10
#define MAX_ACCEPTABLE_TACC 50 // TODO: figure out a good value for this
#define MAX_ACCEPTABLE_SACC 100 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 600 // scale is 0.01, max acceptable is 6.0

#define MAX_EMPTY_QUEUE_WAIT 50 // wait for max 50ms

#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins

#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins TODO: verify this

typedef struct GNSS {
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* gnss_uart_handle;
	DMA_HandleTypeDef* gnss_dma_handle;
	// Pointers to the arrays
	int16_t* GNSS_N_Array;
	int16_t* GNSS_E_Array;
	int16_t* GNSS_D_Array;

	// Keep a running track of sum -- to be used in getRunningAverage
	int64_t vNorthSum;
	int64_t vEastSum;
	int64_t vDownSum;

	// Hold the current lat/long for whatever we might need it for (modem)
	int32_t currentLatitude;
	int32_t currentLongitude;

	// Increment with each sample or running average
	uint16_t totalSamples;
	// We'll keep track of how many times we had to sub in a running average
	uint16_t totalSamplesAveraged; // Just do a %10 in the end
	// How many times we've had to skip a sample - gets reset with valid data
	uint16_t numberCyclesWithoutData;

	// Flags
	bool latLongIsValid;
	bool velocityIsValid;
	bool clockHasBeenSet;
	bool validMessageProcessed;

	// Event flags
	TX_EVENT_FLAGS_GROUP* event_flags;

	// Function pointers
	gnss_error_code_t (*config)(struct GNSS* self);
	gnss_error_code_t (*get_location)(struct GNSS* self, int32_t* latitude,
			int32_t* longitude);
	gnss_error_code_t (*get_running_average_velocities)(struct GNSS* self,
			int16_t* returnNorth, int16_t* returnEast, int16_t* returnDown);
	gnss_error_code_t (*gnss_process_message)(struct GNSS* self, char* process_buf);
	gnss_error_code_t (*sleep)(struct GNSS* self);
} GNSS;

/* Function declarations */
void gnss_init(GNSS* self, UART_HandleTypeDef* gnss_uart_handle,
		DMA_HandleTypeDef* gnss_dma_handle, TX_EVENT_FLAGS_GROUP* event_flags,
		int16_t* GNSS_N_Array, int16_t* GNSS_E_Array, int16_t* GNSS_D_Array);

gnss_error_code_t gnss_config(GNSS* self);

gnss_error_code_t gnss_get_location(GNSS* self, int32_t* latitude, int32_t* longitude);

gnss_error_code_t gnss_get_running_average_velocities(GNSS* self, int16_t* returnNorth,
		int16_t* returnEast, int16_t* returnDown);

gnss_error_code_t gnss_process_message(GNSS* self, char* process_buf);

gnss_error_code_t gnss_sleep(GNSS* self);



#endif /* SRC_GPS_H_ */
