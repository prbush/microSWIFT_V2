/*
 * Battery.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "Peripherals/battery.h"

extern uint32_t* adc_buf;

void battery_init(Battery* self, ADC_HandleTypeDef* adc_handle, TX_EVENT_FLAGS_GROUP* control_flags,
		TX_EVENT_FLAGS_GROUP* error_flags)
{
	self->adc_handle = adc_handle;
	self->control_flags = control_flags;
	self->error_flags = error_flags;
	self->voltage = 0;
	self->start_conversion = battery_start_conversion;
	self->get_voltage = battery_get_voltage;
	self->shutdown_adc = battery_shutdown_adc;
}

battery_error_code_t battery_start_conversion(Battery* self)
{
	battery_error_code_t return_code;

	/** Enable the VREF clock
	*/
	__HAL_RCC_VREF_CLK_ENABLE();

	/** Configure the internal voltage reference buffer voltage scale
	*/
	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);

	/** Enable the Internal Voltage Reference buffer
	*/
	HAL_SYSCFG_EnableVREFBUF();

	/** Configure the internal voltage reference buffer high impedance mode
	*/
	HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
	// Enable VDDA, power supply to ADC
	HAL_PWREx_EnableVddA();

	HAL_Delay(1);

	if (HAL_ADCEx_Calibration_Start(self->adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		return_code = BATTERY_ADC_ERROR;
		return return_code;
	}
	// Need at least a 4 ADC clock cycle delay after calibration before doing anything else with the
	// ADC -- 10ms ought to do.
//	HAL_Delay(10);
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
	self->calibration_offset = HAL_ADCEx_Calibration_GetValue(self->adc_handle, ADC_SINGLE_ENDED);

	if (HAL_ADC_Start_IT(self->adc_handle) != HAL_OK) {
		return_code = BATTERY_ADC_ERROR;
		return return_code;
	}

	return_code = BATTERY_SUCCESS;
	return return_code;
}

battery_error_code_t battery_get_voltage(Battery* self, real16_T* voltage)
{
	uint32_t max_ticks_to_get_voltage = TX_TIMER_TICKS_PER_SECOND;
	ULONG actual_flags;

	// If it either takes too long or an EDC error is detected, set the battery voltage to the error value
	if ((tx_event_flags_get(self->control_flags, BATTERY_VOLTAGE_CONVERSION_COMPLETE,
			TX_OR_CLEAR, &actual_flags, max_ticks_to_get_voltage) != TX_SUCCESS)
			||
			(tx_event_flags_get(self->error_flags, ADC_CONVERSION_ERROR,
						TX_OR_CLEAR, &actual_flags, max_ticks_to_get_voltage) == TX_SUCCESS))
	{

		// Stop the ADC conversions
		HAL_ADC_Stop_IT(self->adc_handle);
		// Set the error bit pattern
		voltage->bitPattern = BATTERY_ERROR_VOLTAGE_VALUE;
		return BATTERY_TIMEOUT_ERROR;
	}

	*voltage = floatToHalf(self->voltage);
	return BATTERY_SUCCESS;
}

void battery_shutdown_adc(void)
{

	HAL_SYSCFG_DisableVREFBUF();

	__HAL_RCC_VREF_CLK_DISABLE();

	HAL_PWREx_DisableVddA();
}

