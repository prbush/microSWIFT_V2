/*
 * gps_error_codes.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Phil
 */

#ifndef INC_GPS_ERROR_CODES_H_
#define INC_GPS_ERROR_CODES_H_

typedef enum {
	// Error/ success codes
	GPS_SUCCESS = 0,
	GPS_UNKNOWN_ERROR = -1,
	GPS_LOCATION_INVALID = -2,
	GPS_VELOCITY_INVALID = -3,
	GPS_NO_SAMPLES_ERROR = -4,
	GPS_TIMEOUT_ERROR = -5,
	GPS_BUSY_ERROR = -6,
	GPS_NO_MESSAGE_RECEIVED = -7
} gps_error_code_t;




#endif /* INC_GPS_ERROR_CODES_H_ */
