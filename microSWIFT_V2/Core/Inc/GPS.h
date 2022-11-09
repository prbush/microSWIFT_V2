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
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "u_ubx_protocol.h"
#include "u_error_common.h"

// Macros
#define MAX_POSSIBLE_VELOCITY 10000

#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define MAX_ACCEPTABLE_TACC 50
#define MAX_ACCEPTABLE_VACC 50 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 600 // scale is 0.01, max acceptable is 6.0

// Error/ success codes
#define GPS_SUCCESS 0
#define GPS_UNKNOWN_ERROR -1
#define GPS_LOCATION_INVALID -2
#define GPS_VELOCITY_INVALID -3
#define GPS_NO_SAMPLES_ERROR -4
#define GPS_TIMEOUT_ERROR -5
#define GPS_BUSY_ERROR -6
#define GPS_NO_MESSAGE_RECEIVED -7

#ifdef __cplusplus

class GPS {

public:
	GPS(UART_HandleTypeDef* uart_handle);
	virtual ~GPS();
	int32_t init(void);
	int32_t getUBX_NAV_PVT(void);
	int32_t getLocation(int32_t& latitude, int32_t& longitude);
//	int32_t getVelocity(int32_t* north, int32_t* east, int32_t* down, uint32_t* spdAccuracy, int32_t* gndSpeed);
	int32_t processMessage(void);
	int32_t getRunningAverage(float* velocityValue, int32_t whichVelocity);

	bool sleep(void);

	// Strictly for testing, remove in production
	void testFunct(void);

private:
	// Until we know better, these will be initialized to 10k float values each
	float** vNorthArray;
	float** vEastArray;
	float** vDownArray;
	// Keep a running track of sum -- to be used in getRunningAverage
	float vNorthSum = 0;
	float vEastSum = 0;
	float vDownSum = 0;
	// Increment with each sample or running average
	uint16_t totalSamples = 0;
	// We'll keep track of how many times we had to sub in a running average
	uint16_t totalSamplesAveraged = 0; // Just do a %10 in the end
	// How many times we've had to skip a sample - gets reset with valid data
	uint16_t numberCyclesWithoutData = 0;

	// Hold onto the current UBX_NAV_PVT message
	char UBX_NAV_PVTmessageBuf[92];

	// Hold the current lat/long for whatever we might need it for (modem)
	int32_t currentLatitude = 0;
	int32_t currentLongitude = 0;

	// Flags
	bool latLongIsValid = false;
	bool velocityIsValid = false;
	bool clockHasBeenSet = false;

	UART_HandleTypeDef* gps_uart_handle;
};

#endif // __cplusplus

#endif /* SRC_GPS_H_ */
