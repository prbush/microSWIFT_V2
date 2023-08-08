/*
 * Battery.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "Peripherals/battery.h"

extern uint32_t* adc_buf;

void battery_init(Battery* self, ADC_HandleTypeDef* adc_handle, TX_EVENT_FLAGS_GROUP* control_flags)
{
	self->adc_handle = adc_handle;
	self->control_flags = control_flags;
	self->voltage = 0;
	self->start_conversion = battery_start_conversion;
	self->get_voltage = battery_get_voltage;
}

battery_error_code_t battery_start_conversion(Battery* self)
{
	battery_error_code_t return_code;
	if (HAL_ADCEx_Calibration_Start(self->adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		return_code = BATTERY_ADC_ERROR;
		return BATTERY_ADC_ERROR;
	}
	// Need at least a 4 ADC clock cycle delay after calibration before doing anything else with the
	// ADC -- 10ms ought to do.
	HAL_Delay(10);
	self->calibration_offset = HAL_ADCEx_Calibration_GetValue(self->adc_handle, ADC_SINGLE_ENDED);

//	HAL_ADC_PollForConversion(self->adc_handle, 500);
//	uint32_t value = HAL_ADC_GetValue(self->adc_handle);

//	if (HAL_ADC_Start_DMA(self->adc_handle, adc_buf, NUMBER_OF_ADC_SAMPLES) != HAL_OK) {
	if (HAL_ADC_Start_IT(self->adc_handle) != HAL_OK) {
		return_code = BATTERY_ADC_ERROR;
		return BATTERY_ADC_ERROR;
	}

	return_code = BATTERY_SUCCESS;
	return BATTERY_ADC_ERROR;
}

battery_error_code_t battery_get_voltage(Battery* self, real16_T* voltage)
{
	uint32_t max_ticks_to_get_voltage = TX_TIMER_TICKS_PER_SECOND * 10;
	ULONG actual_flags;

	if (tx_event_flags_get(self->control_flags, BATTERY_VOLTAGE_CONVERSION_COMPLETE,
			TX_OR_CLEAR, &actual_flags, max_ticks_to_get_voltage) != TX_SUCCESS)
	{

		uint64_t sum = 0;
		for (int i = 0; i < NUMBER_OF_ADC_SAMPLES; i++) {
			sum += (adc_buf[i] * ADC_MICROVOLTS_PER_BIT) + self->calibration_offset;
		}
		double adc_voltage = ((double)sum / (double)NUMBER_OF_ADC_SAMPLES);
		adc_voltage /= 1000000;

		return BATTERY_TIMEOUT_ERROR;
	}

	HAL_ADC_Stop_IT(self->adc_handle);
	self->voltage /= 1000000;
	*voltage = floatToHalf(self->voltage);
	return BATTERY_SUCCESS;
}

