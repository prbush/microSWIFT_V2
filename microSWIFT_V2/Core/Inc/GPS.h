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
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "u_ubx_protocol.h"
#include "u_error_common.h"
#include "gps_error_codes.h"

// Macros
#define MAX_POSSIBLE_VELOCITY 10000	// 10 m/s

#define UBX_NAV_PVT_MESSAGE_CLASS 0x01
#define UBX_NAV_PVT_MESSAGE_ID 0x07
#define UBX_NAV_PVT_MESSAGE_LENGTH 92
#define MAX_ACCEPTABLE_TACC 50 // TODO: figure out a good value for this
#define MAX_ACCEPTABLE_SACC 100 // need to confirm with Jim what this should be
#define MAX_ACCEPTABLE_PDOP 600 // scale is 0.01, max acceptable is 6.0
<<<<<<< HEAD
#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins
=======
#define MAX_EMPTY_CYCLES 5*60*10 // no data for 10 mins TODO: verify this
>>>>>>> 290f347085cc7ca6c738cada52e5504d1047082a


#ifdef __cplusplus

class GPS {

public:
	GPS(UART_HandleTypeDef* uart_handle);
	virtual ~GPS();
<<<<<<< HEAD
	int32_t init(void);
	int32_t getUBX_NAV_PVT(void);
	int32_t getLocation(int32_t& latitude, int32_t& longitude);
//	int32_t getVelocity(int32_t* north, int32_t* east, int32_t* down, uint32_t* spdAccuracy, int32_t* gndSpeed);
	int32_t processMessage(void);
	int32_t getRunningAverage(float& returnNorth, float& returnEast, float& returnDown);
=======
	gps_error_code_t init(void);
	gps_error_code_t getAndProcessMessage(void);
	gps_error_code_t getLocation(int32_t& latitude, int32_t& longitude);
	gps_error_code_t getRunningAverage(float& returnNorth, float& returnEast, float& returnDown);
>>>>>>> 290f347085cc7ca6c738cada52e5504d1047082a

	gps_error_code_t sleep(void);

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
	char UBX_NAV_PVT_message_buf[128];

	// Hold the current lat/long for whatever we might need it for (modem)
	int32_t currentLatitude = 0;
	int32_t currentLongitude = 0;

	// Flags
	bool latLongIsValid = false;
	bool velocityIsValid = false;
	bool clockHasBeenSet = false;
	bool validMessageProcessed = false;

	UART_HandleTypeDef* gps_uart_handle;
};

#endif // __cplusplus

#endif /* SRC_GPS_H_ */
