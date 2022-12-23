/*
 * gps_error_codes.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Phil
 */

#ifndef INC_GNSS_ERROR_CODES_H_
#define INC_GNSS_ERROR_CODES_H_

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
	GNSS_CONFIG_ERROR = -9
} gnss_error_code_t;




#endif /* INC_GNSS_ERROR_CODES_H_ */
