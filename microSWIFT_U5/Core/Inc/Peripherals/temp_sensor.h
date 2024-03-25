/*
 * temp_sensor.h
 *
 * !!! MOSTLY COPPIED FROM BLUE ROBOTICS !!!
 *
 * https://github.com/bluerobotics/BlueRobotics_TSYS01_Library?tab=readme-ov-file
 *
 *  Created on: Feb 9, 2024
 *      Author: Phil
 */

#ifndef INC_PERIPHERALS_TEMP_SENSOR_H_
#define INC_PERIPHERALS_TEMP_SENSOR_H_

#include "stm32u5xx_hal.h"
#include "tx_api.h"
#include "main.h"
#include "stm32u5xx_hal_i2c.h"
#include "stm32u5xx_hal_i2c_ex.h"
#include "stdbool.h"

#define TSYS01_ADDR                        0x77 << 1
#define TSYS01_RESET                       0x1E
#define TSYS01_ADC_READ                    0x00
#define TSYS01_ADC_TEMP_CONV               0x48
#define TSYS01_PROM_READ                   0XA0

#define TEMPERATURE_AVERAGED_ERROR_CODE			0X70e2

typedef enum tmperature_error_code {
	TEMPERATURE_SUCCESS = 0,
	TEMPERATURE_CONVERSION_ERROR = -1,
	TEMPERATURE_TIMEOUT_ERROR = -2,
	TEMPERATURE_COMMUNICATION_ERROR = -3
}temperature_error_code_t;

typedef struct Temperature {
	I2C_HandleTypeDef* 			i2c_handle;
	TX_EVENT_FLAGS_GROUP*   	control_flags;
	TX_EVENT_FLAGS_GROUP*  		error_flags;
	GPIO_TypeDef*				gpio_bus;

	void 						(*on)(void);
	void						(*off)(void);
	temperature_error_code_t	(*reset_i2c)(void);
	temperature_error_code_t	(*self_test)(void);
	temperature_error_code_t	(*get_reading)(void);

	float						converted_temp;
	float 						C[8]; // Cal data array
	uint32_t					D1; // Read data (unconverted temp)
	uint32_t					adc;

	uint16_t					pwr_gpio;
}Temperature;


void temperature_init(Temperature* struct_ptr, I2C_HandleTypeDef* i2c_handle, TX_EVENT_FLAGS_GROUP* control_flags,
		TX_EVENT_FLAGS_GROUP* error_flags, GPIO_TypeDef *gpio_bus, uint16_t pwr_gpio, bool clear_calibration_data);

#endif /* INC_PERIPHERALS_TEMP_SENSOR_H_ */
