/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "stdbool.h"

// Time in minutes representing a sample period. Any time left after sampling
// and transmission will be spent in sleep mode
#define DUTY_CYCLE 60
// Number of samples in each sampling window
#define TOTAL_SAMPLES_PER_WINDOW 8192
// Sampling rate in Hz for the GNSS sensor
#define GNSS_SAMPLING_RATE 5
// The max time to try to get an Iridium message off in seconds
#define IRIDIUM_MAX_TRANSMIT_TIME 600
// If the IMU will be utilized or not
#define IMU_ENABLED false
// If there is a CT sensor present
#define CT_ENABLED true

typedef struct microSWIFT_configuration {
	uint32_t duty_cycle;
	uint32_t samples_per_window;
	uint32_t iridium_max_transmit_time;
	uint32_t gnss_sampling_rate;
} microSWIFT_configuration;


#endif /* INC_CONFIGURATION_H_ */
