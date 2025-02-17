/*
 * temp_sensor.c
 *
 * !!! MOSTLY COPPIED FROM BLUE ROBOTICS !!!
 *
 * https://github.com/bluerobotics/BlueRobotics_TSYS01_Library?tab=readme-ov-file
 *
 *  Created on: Feb 9, 2024
 *      Author: Phil
 */

#include "Peripherals/temp_sensor.h"
// Object instance pointer
static Temperature* self;

// Struct functions
static void						temperature_on(void);
static void						temperature_off(void);
static temperature_error_code_t	temperature_reset_i2c(void);
static temperature_error_code_t temperature_self_test(void);
static temperature_error_code_t temperature_get_readings(void);

// Helper functions
static bool						init_sensor(void);
static float 					calculate_temp(void);
static void						reset_struct_fields(bool reset_calibration);

void temperature_init(Temperature* struct_ptr, I2C_HandleTypeDef* i2c_handle, TX_EVENT_FLAGS_GROUP* control_flags,
		TX_EVENT_FLAGS_GROUP* error_flags, GPIO_TypeDef *gpio_bus, uint16_t pwr_gpio, bool clear_calibration_data)
{
	self = struct_ptr;

	self->i2c_handle = i2c_handle;
	self->control_flags = control_flags;
	self->error_flags = error_flags;
	self->pwr_gpio = pwr_gpio;
	self->gpio_bus = gpio_bus;

	self->on = temperature_on;
	self->off = temperature_off;
	self->reset_i2c = temperature_reset_i2c;
	self->self_test = temperature_self_test;
	self->get_readings = temperature_get_readings;

	reset_struct_fields(clear_calibration_data);
}


static void	temperature_on(void)
{
	HAL_GPIO_WritePin(self->gpio_bus, self->pwr_gpio, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	tx_thread_sleep(1);
}


static void	temperature_off(void)
{
	HAL_GPIO_WritePin(self->gpio_bus, self->pwr_gpio, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

static temperature_error_code_t	temperature_reset_i2c(void)
{

	temperature_error_code_t return_code = TEMPERATURE_SUCCESS;

	HAL_I2C_DeInit(self->i2c_handle);

	tx_thread_sleep(1);

	self->i2c_handle->Instance = self->i2c_handle->Instance;
	self->i2c_handle->Init.Timing = 0x40000A0B;
	self->i2c_handle->Init.OwnAddress1 = 0;
	self->i2c_handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	self->i2c_handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	self->i2c_handle->Init.OwnAddress2 = 0;
	self->i2c_handle->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	self->i2c_handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	self->i2c_handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(self->i2c_handle) != HAL_OK)
  {
  	return_code = TEMPERATURE_COMMUNICATION_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(self->i2c_handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
  	return_code = TEMPERATURE_COMMUNICATION_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(self->i2c_handle, 0) != HAL_OK)
  {
  	return_code = TEMPERATURE_COMMUNICATION_ERROR;
  }

  return return_code;
}


static temperature_error_code_t temperature_self_test(void)
{
	if (init_sensor()) {
		return TEMPERATURE_SUCCESS;
	} else {
		return TEMPERATURE_COMMUNICATION_ERROR;
	}
}


static temperature_error_code_t temperature_get_readings(void)
{
	temperature_error_code_t return_code = TEMPERATURE_SUCCESS;
	uint8_t command;
	uint8_t read_data[3] = {0};
	float readings_accumulator = 0;

	if (!init_sensor()) {
		return TEMPERATURE_COMMUNICATION_ERROR;
	}

	for (int i = 0; i < TOTAL_TEMPERATURE_SAMPLES; i++) {

		command = TSYS01_ADC_TEMP_CONV;
		if (HAL_I2C_Master_Transmit(self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
				!= HAL_OK) {
			return_code = TEMPERATURE_CONVERSION_ERROR;
			return return_code;
		}

		tx_thread_sleep(1);

		command = TSYS01_ADC_READ;
		if (HAL_I2C_Master_Transmit(self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
				!= HAL_OK) {
			return_code = TEMPERATURE_CONVERSION_ERROR;
			return return_code;
		}

		if (HAL_I2C_Master_Receive(self->i2c_handle, TSYS01_ADDR, &(read_data[0]), sizeof(read_data), 10)
				!= HAL_OK) {
			return_code = TEMPERATURE_CONVERSION_ERROR;
			return return_code;
		}

		self->D1 = (read_data[0] << 16) | (read_data[1] << 8) | read_data[2];

		readings_accumulator += calculate_temp();
	}

	self->converted_temp = readings_accumulator / TOTAL_TEMPERATURE_SAMPLES;

	return return_code;
}


static bool	init_sensor(void)
{
	uint8_t command = TSYS01_RESET;
	uint8_t read_data[2] = {0};
	// Reset the TSYS01, per datasheet
	if (HAL_I2C_Master_Transmit(self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
			!= HAL_OK) {
		return false;
	}

	tx_thread_sleep(1);
		// Read calibration values
	for ( uint8_t i = 0 ; i < 8 ; i++ ) {
		tx_thread_sleep(1);
		command = TSYS01_PROM_READ + (i*2);
		if (HAL_I2C_Master_Transmit(self->i2c_handle, TSYS01_ADDR, &command, sizeof(command), 10)
				!= HAL_OK) {
			return false;
		}

		if (HAL_I2C_Master_Receive(self->i2c_handle, TSYS01_ADDR, &(read_data[0]), sizeof(read_data), 10)
				!= HAL_OK) {
			return false;
		}

		self->C[i] = (float)((read_data[0] << 8) | read_data[1]);
	}

	return true;
}


static float calculate_temp(void)
{
	float temp = 0.0;
	self->adc = self->D1/256;
	temp = (-2.0f) * self->C[1] / 1000000000000000000000.0f
								 * pow(self->adc,4) + 4.0f * self->C[2] / 10000000000000000.0f
								 * pow(self->adc,3) + (-2.0f) * self->C[3] / 100000000000.0f
								 * pow(self->adc,2) + 1.0f * self->C[4] / 1000000.0f
								 * self->adc + (-1.5f) * self->C[5] / 100.0f ;
	return temp;
}

static void	reset_struct_fields(bool reset_calibration)
{
	if (reset_calibration) {
		memset(self->C, 0, sizeof(self->C));
	}

	self->converted_temp = 0.0f;
	self->D1 = 0;
	self->adc = 0;
}
