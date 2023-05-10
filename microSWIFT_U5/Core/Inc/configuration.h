/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "stdbool.h"
#include "stdint.h"

#define DBUG//TODO:remove this

// Time in minutes representing a sample period. Any time left after sampling
// and transmission will be spent in sleep mode
#define DUTY_CYCLE 60

#ifdef DBUG

#define TOTAL_SAMPLES_PER_WINDOW 256
#define IRIDIUM_MAX_TRANSMIT_TIME 10
#define GNSS_MAX_ACQUISITION_WAIT_TIME 1

#else
// Number of samples in each sampling window
#define TOTAL_SAMPLES_PER_WINDOW 8192
// The max time in MINUTES to try to get an Iridium message off
#define IRIDIUM_MAX_TRANSMIT_TIME 10
// The max time in MINUTES without good data from GNSS before commanding to sleep
// !! Must be greater than 0
#define GNSS_MAX_ACQUISITION_WAIT_TIME 10
#endif

// Sampling rate in Hz for the GNSS sensor
#define GNSS_SAMPLING_RATE 5
// The number of samples for the CT sensor to take. Result will be averaged
#define TOTAL_CT_SAMPLES 10
// If the IMU will be utilized or not
#define IMU_ENABLED false
// If there is a CT sensor present
#define CT_ENABLED true

typedef struct microSWIFT_configuration {
	uint32_t duty_cycle;
	uint32_t samples_per_window;
	uint32_t iridium_max_transmit_time;
	uint32_t gnss_max_acquisition_wait_time;
	uint32_t gnss_sampling_rate;
	uint32_t total_ct_samples;
} microSWIFT_configuration;


#endif /* INC_CONFIGURATION_H_ */
