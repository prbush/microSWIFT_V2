/*
 * rf_switch.c
 *
 *  Created on: May 10, 2023
 *      Author: Phil
 */

#include "Peripherals/rf_switch.h"
// Internal object pointer
static RF_Switch* self;
// Object functions
static void rf_switch_power_on(void);
static void rf_switch_power_off(void);
static void rf_switch_set_gnss_port(void);
static void rf_switch_set_iridium_port(void);



/**
 * Initialize the RF switch struct. RF switch will initialize to the GNSS port.
 *
 * @return gnss_error_code_t
 */
void rf_switch_init(RF_Switch* struct_ptr)
{
	// Assign object pointer
	self = struct_ptr;

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
static void rf_switch_power_on(void)
{
	self->en_pin_current_state = GPIO_PIN_SET;

	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
			self->en_pin_current_state);
}

/**
 * @brief	Turn off the rf switch FET
 *
 * @return 	void
 */
static void rf_switch_power_off(void)
{
	self->en_pin_current_state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(self->en_gpio_group, self->en_gpio_pin,
			self->en_pin_current_state);
}


/**
 * @brief	Command the RF switch to RF1 (GNSS) port
 *
 * @return 	void
 */
static void rf_switch_set_gnss_port(void)
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
static void rf_switch_set_iridium_port(void)
{
	self->vctl_pin_current_state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(self->vctl_gpio_group, self->vctl_gpio_pin,
			self->vctl_pin_current_state);
	HAL_Delay(1);

	self->current_port = RF_IRIDIUM_PORT;
}

