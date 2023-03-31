/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "stdbool.h"

// Number of samples in each sampling window
#define TOTAL_SAMPLES_PER_WINDOW 8192
// Time in seconds between each sampling window
#define TOTAL_TIME_BETWEEN_WINDOWS 1800
// Sampling rate in Hz for the GNSS sensor
#define GNSS_SAMPLING_RATE 5
// If the IMU will be utilized or not
#define IMU_ENABLED false
// Time delay between power-on self-test and start of operation
#define START_OF_OPERATION_DELAY 0

typedef struct microSWIFT_configuration {
	uint32_t total_samples_per_window;
	uint32_t total_time_between_windows;
	uint32_t start_of_operation_delay;
	uint8_t  gnss_sampling_rate;
	bool imu_enabled;
} microSWIFT_configuration;


#endif /* INC_CONFIGURATION_H_ */
