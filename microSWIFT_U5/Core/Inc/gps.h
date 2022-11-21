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
#include <gnss_error_codes.h>
#include <stdint.h>
#include <string.h>
#include "stm32u5xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "u_ubx_protocol.h"
#include "u_error_common.h"

// Macros
#define MAX_POSSIBLE_VELOCITY 10000	// 10 m/s

#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define UBX_NAV_PVT_MESSAGE_LENGTH 92
#define MAX_ACCEPTABLE_TACC 50 // TODO: figure out a good value for this
#define MAX_ACCEPTABLE_SACC 100 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 600 // scale is 0.01, max acceptable is 6.0

#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins

#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins TODO: verify this

typedef struct GNSS {
	// The UART handle for the GNSS interface
	UART_HandleTypeDef* gnss_uart_handle;

	// Pointers to the arrays
	int16_t* uGNSSArray;
	int16_t* vGNSSArray;
	int16_t* zGNSSArray;

	// Keep a running track of sum -- to be used in getRunningAverage
	float vNorthSum;
	float vEastSum;
	float vDownSum;

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

	// Function pointers
	void (*init)(GNSS* self, UART_HandleTypeDef* gnss_uart_handle,
			int16_t* uGNSSArray, int16_t* vGNSSArray, int16_t* zGNSSArray);
	gnss_error_code_t (*get_location)(GNSS* gnss_struct, int32_t* latitude,
			int32_t* longitude);
	gnss_error_code_t (*get_running_average_velocities)(GNSS* gnss_struct,
			float* returnNorth, float* returnEast, float* returnDown);
	gnss_error_code_t (*get_and_process_message)(GNSS* gnss_struct);
	gnss_error_code_t (*sleep)(GNSS* gnss_struct)
} GNSS;

void gnss_init(GNSS* return_struct, UART_HandleTypeDef* gnss_uart_handle,
		int16_t* uGNSSArray, int16_t* vGNSSArray, int16_t* zGNSSArray);
gnss_error_code_t gnss_get_location(GNSS* gnss_struct, int32_t* latitude, int32_t* longitude);
gnss_error_code_t gnss_get_running_average_velocities(GNSS* gnss_struct, float* returnNorth, float* returnEast, float* returnDown);
gnss_error_code_t gnss_get_and_process_message(GNSS* gnss_struct);
gnss_error_code_t gnss_sleep(GNSS* gnss_struct);

#endif /* SRC_GPS_H_ */
