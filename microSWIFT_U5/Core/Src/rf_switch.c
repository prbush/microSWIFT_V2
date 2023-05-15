/*
 * rf_switch.c
 *
 *  Created on: May 10, 2023
 *      Author: Phil
 */

#include "rf_switch.h"

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

	self->get_current_port = rf_switch_get_current_port;
	self->set_gnss_port = rf_switch_set_gnss_port;
	self->set_iridium_port = rf_switch_set_iridium_port;
	self->set_no_port = rf_switch_set_no_port;

	self->set_gnss_port(self);
}

/**
 * @brief	Return the current state of the RF switch. Just to be overly sure, the expected state will
 * 			be commanded.
 *
 * @return 	One of possible rf_switch_selection_t
 */
rf_switch_selection_t rf_switch_get_current_port(RF_Switch* self)
{
	if (self->en_pin_current_state == GPIO_PIN_RESET &&
			self->vctl_pin_current_state == GPIO_PIN_SET) {
		self->set_gnss_port(self);
	}
	else if (self->en_pin_current_state == GPIO_PIN_RESET &&
			self->vctl_pin_current_state == GPIO_PIN_RESET) {
		self->set_iridium_port(self);
	} else {
		self->set_no_port(self);
	}

	return self->current_port;
}

/**
 * @brief	Command the RF switch to RF1 (GNSS) port
 *
 * @return 	void
 */
void rf_switch_set_gnss_port(RF_Switch* self)
{
	self->en_pin_current_state = GPIO_PIN_RESET;
	self->vctl_pin_current_state = GPIO_PIN_SET;

	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
			self->en_pin_current_state);
	HAL_GPIO_WritePin(self->vctl_gpio_group, self->vctl_gpio_pin,
			self->vctl_pin_current_state);

	self->current_port = RF_GNSS_PORT;
}

/**
 * @brief	Command the RF switch to RF2 (Iridium) port
 *
 * @return 	void
 */
void rf_switch_set_iridium_port(RF_Switch* self)
{
	self->en_pin_current_state = GPIO_PIN_RESET;
	self->vctl_pin_current_state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
			self->en_pin_current_state);
	HAL_GPIO_WritePin(self->vctl_gpio_group, self->vctl_gpio_pin,
			self->vctl_pin_current_state);

	self->current_port = RF_IRIDIUM_PORT;
}

/**
 * @brief	Command the RF switch to isolate the antenna from either port.
 *
 * @return 	void
 */
void rf_switch_set_no_port(RF_Switch* self)
{
	self->en_pin_current_state = GPIO_PIN_SET;
	self->vctl_pin_current_state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
			self->en_pin_current_state);
	HAL_GPIO_WritePin(self->vctl_gpio_group, self->vctl_gpio_pin,
			self->vctl_pin_current_state);

	self->current_port = RF_NO_PORT;
}
