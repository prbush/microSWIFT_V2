/*
 * configuration.h
 *
 *  Created on: Mar 27, 2023
 *      Author: Phil
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "main.h"
#include "stdbool.h"
#include "stdint.h"

/*
 * Debugging settings
 */

// For testing and debugging with a very short sample window
//#define DEBUGGING_FAST_CYCLE

// If a 1 min sleep window is desired
//#define SHORT_SLEEP

/*
 * Configuration options
 */

// Define if the old type V3D RockBlock modem is used
//#define ROCK_BLOCK_V3D

// For debugging, redefine sample window parameters to be faster
#ifdef DEBUGGING_FAST_CYCLE

#define TOTAL_SAMPLES_PER_WINDOW 1024
#define IRIDIUM_MAX_TRANSMIT_TIME 30
#define GNSS_MAX_ACQUISITION_WAIT_TIME 10
#define SAMPLE_WINDOWS_PER_HOUR 1

#else
// Number of samples in each sampling window
#define TOTAL_SAMPLES_PER_WINDOW 4096

// The max time in MINUTES to try to get an Iridium message off
#define IRIDIUM_MAX_TRANSMIT_TIME 6

// The max time in MINUTES without good data from GNSS before commanding to sleep
// !! Must be greater than 0
// **** In the case of SAMPLE_WINDOWS_PER_HOUR > 1, this will only apply to the very
//      first sample window. Subsequent windows will calculate the GNSS acq time
#define GNSS_MAX_ACQUISITION_WAIT_TIME 15

// Are we doing 1 or two sample windows per hour
// !! Must be a number such that SAMPLE_WINDOWS_PER_HOUR % 60 == 0
#define SAMPLE_WINDOWS_PER_HOUR 2
#endif // DEBUGGING_FAST_CYCLE

// Sampling rate in Hz for the GNSS sensor
// !! Must be either 4 or 5
#define GNSS_SAMPLING_RATE 4

// Determine whether or not the GNSS sensor should be set to high performance mode
#define GNSS_HIGH_PERFORMANCE_MODE_ENABLED false

// The number of samples for the CT sensor to take. Result will be averaged
#define TOTAL_CT_SAMPLES 10

// If the IMU will be utilized or not
#define IMU_ENABLED false

// If there is a CT sensor present
#define CT_ENABLED false

// If there is a Blue Robotics I2C temperature sensor presen
#define TEMPERATURE_ENABLED false

// If we are saving raw data to flash
#define FLASH_STORAGE_ENABLED false

// If true, will clear out flash memory on initial power up or hard reset.
#define CLEAR_USER_FLASH false

// If true, Type 99 messages will be transmitted indicating status of flash operation
#define VERBOSE_FLASH false

// Whether or not to use the Independent watchdog
#define WATCHDOG_ENABLED true

typedef struct microSWIFT_configuration{
	uint32_t samples_per_window;
	uint32_t iridium_max_transmit_time;
	uint32_t gnss_max_acquisition_wait_time;
	uint32_t gnss_sampling_rate;
	uint32_t total_ct_samples;
	uint32_t windows_per_hour;
	uint32_t reset_reason;
	bool 	 gnss_high_performance_mode;
} microSWIFT_configuration;


#endif /* INC_CONFIGURATION_H_ */
