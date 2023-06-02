/*
 * rf_switch.c
 *
 *  Created on: May 10, 2023
 *      Author: Phil
 */

#include "Peripherals/rf_switch.h"

/**
 * Initialize the RF switch struct. RF switch will initialize to the GNSS port.
 *
 * @return gnss_error_code_t
 */
void rf_switch_init(RF_Switch* self)
{
	self->en_gpio_group = GPIOD;
	self->vctl_gpio_group = GPIOD;
	self->en_gpio_pin = RF_SWITCH_EN_Pin;
	self->vctl_gpio_pin = RF_SWITCH_VCTL_Pin;

	self->power_on = rf_switch_power_on;
	self->power_off = rf_switch_power_off;
	self->set_gnss_port = rf_switch_set_gnss_port;
	self->set_iridium_port = rf_switch_set_iridium_port;
}

/**
 * @brief	Turn on the rf switch FET
 *
 * @return 	void
 */
void rf_switch_power_on(RF_Switch* self)
{
	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
				GPIO_PIN_SET);
}

/**
 * @brief	Turn off the rf switch FET
 *
 * @return 	void
 */
void rf_switch_power_off(RF_Switch* self)
{
	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
				GPIO_PIN_RESET);
}


/**
 * @brief	Command the RF switch to RF1 (GNSS) port
 *
 * @return 	void
 */
void rf_switch_set_gnss_port(RF_Switch* self)
{
	self->vctl_pin_current_state = GPIO_PIN_SET;

	HAL_GPIO_WritePin(self->vctl_gpio_group, self->vctl_gpio_pin,
			self->vctl_pin_current_state);
	HAL_Delay(1);

	self->current_port = RF_GNSS_PORT;
}

/**
 * @brief	Command the RF switch to RF2 (Iridium) port
 *
 * @return 	void
 */
void rf_switch_set_iridium_port(RF_Switch* self)
{
	self->vctl_pin_current_state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(self->vctl_gpio_group, self->vctl_gpio_pin,
			self->vctl_pin_current_state);
	HAL_Delay(1);

	self->current_port = RF_IRIDIUM_PORT;
}

