/*
 * Battery.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_BATTERY_H_
#define SRC_BATTERY_H_

#include "stm32u5xx_hal.h"
#include "tx_api.h"
#include "NEDWaves/rtwhalf.h"
#include "app_threadx.h"

#define NUMBER_OF_ADC_SAMPLES 1200
#define BATTERY_CALIBRATION_OFFSET 0

typedef enum battery_error_code {
	BATTERY_SUCCESS = 0,
	BATTERY_CONVERSION_ERROR = -1,
	BATTERY_TIMEOUT_ERROR = -2,
	BATTERY_ADC_ERROR = -3
}battery_error_code_t;

typedef struct Battery {
	ADC_HandleTypeDef* 		adc_handle;
	TX_EVENT_FLAGS_GROUP* control_flags;

	float									voltage;
	uint32_t							calibration_offset;

	battery_error_code_t 	(*start_conversion)(struct Battery* self);
	battery_error_code_t 	(*get_voltage)(struct Battery* self, real16_T* voltage);
}Battery;


void battery_init(Battery* self, ADC_HandleTypeDef* adc_handle, TX_EVENT_FLAGS_GROUP* control_flags);
battery_error_code_t battery_start_conversion(Battery* self);
battery_error_code_t battery_get_voltage(Battery* self, real16_T* voltage);

#endif /* SRC_BATTERY_H_ */
