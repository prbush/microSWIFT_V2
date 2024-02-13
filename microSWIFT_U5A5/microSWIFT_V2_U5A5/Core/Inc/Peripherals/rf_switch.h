/*
 * rf_switch.h
 *
 *  Created on: May 10, 2023
 *      Author: Phil
 */

#ifndef SRC_RF_SWITCH_H_
#define SRC_RF_SWITCH_H_

#include "stm32u5xx_hal.h"
#include "main.h"

// RF1 = GNSS
// RF2 = Modem

typedef enum rf_switch_selection{
	RF_GNSS_PORT = 1,
	RF_IRIDIUM_PORT = 2,
	RF_NO_PORT = 3
}rf_switch_selection_t;

typedef struct RF_Switch{
	GPIO_TypeDef* en_gpio_group;
	GPIO_TypeDef* vctl_gpio_group;
	uint16_t en_gpio_pin;
	uint16_t vctl_gpio_pin;
	GPIO_PinState en_pin_current_state;
	GPIO_PinState vctl_pin_current_state;
	rf_switch_selection_t current_port;
	void 					(*power_on)(void);
	void 					(*power_off)(void);
	void 					(*set_gnss_port)(void);
	void 					(*set_iridium_port)(void);
} RF_Switch;

void rf_switch_init(RF_Switch* struct_ptr);

#endif /* SRC_RF_SWITCH_H_ */
