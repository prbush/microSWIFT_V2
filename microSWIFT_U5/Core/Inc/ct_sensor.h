/*
 * CT.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_CT_H_
#define SRC_CT_H_

#include "app_threadx.h"
#include "tx_api.h"
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_dma.h"
#include "stdio.h"
#include "stdbool.h"

// 20,000 milliseconds -> 20 seconds
#ifdef DBUG
#define WARMUP_TIME 20
#else
#define WARMUP_TIME 20000
#endif
// The total length of a response sentence from the sensor
#define CT_DATA_ARRAY_SIZE 291
#define CT_DEFAULT_BAUD_RATE 9600
#define TEMP_MEASUREMENT_START_INDEX 70
#define TEMP_OFFSET_FROM_UNITS 6
#define SALINITY_OFFSET_FROM_UNITS 4

typedef enum {
	CT_SUCCESS = 0,
	CT_UART_ERROR = -1,
	CT_PARSING_ERROR = -2,
	CT_SELF_TEST_FAIL = -3,
	CT_NOT_ENOUGH_SAMPLES = -4,
	CT_DONE_SAMPLING = -5
}ct_error_code_t;

typedef struct ct_samples{
	double salinity;
	double temp;
} ct_samples;

typedef struct CT{
	// Our global configuration struct
	microSWIFT_configuration* global_config;
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* ct_uart_handle;
	DMA_HandleTypeDef* ct_dma_handle;
	// Event flags
	TX_EVENT_FLAGS_GROUP* control_flags;
	TX_EVENT_FLAGS_GROUP* error_flags;
	// The buffer written to by CT sensor
	char* data_buf;
	// Arrays to hold conductivity/temp values
	ct_samples* samples_buf;
	ct_samples averages;
	// Keep track of the number of samples
	uint32_t total_samples;
	// Function pointers
	ct_error_code_t (*parse_sample)  (struct CT* self);
	ct_error_code_t (*get_averages)  (struct CT* self);
	void		    (*on_off) 		 (struct CT* self, GPIO_PinState pin_state);
	ct_error_code_t (*self_test) 	 (struct CT* self, bool add_warmup_time);
	ct_error_code_t (*reset_ct_uart) (struct CT* self, uint16_t baud_rate);
} CT;

void ct_init(CT* self, microSWIFT_configuration* global_config, UART_HandleTypeDef* ct_uart_handle,
		DMA_HandleTypeDef* ct_dma_handle, TX_EVENT_FLAGS_GROUP* control_flags,
		TX_EVENT_FLAGS_GROUP* error_flags, char* data_buf, ct_samples* samples_buf);
ct_error_code_t ct_parse_sample(CT* self);
ct_error_code_t ct_get_averages(CT* self);
void 			ct_on_off(CT* self, GPIO_PinState pin_state);
ct_error_code_t ct_self_test(CT* self, bool add_warmup_time);
ct_error_code_t reset_ct_uart(CT* self, uint16_t baud_rate);

#endif /* SRC_CT_H_ */
