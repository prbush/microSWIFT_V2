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

typedef enum {
	CT_SUCCESS = 0,
	CT_UART_ERROR = -1,
	CT_PARSING_ERROR = -2,
}ct_error_code_t;

typedef struct ct_samples{
	double conductivity;
	double temp;
} ct_samples;

typedef struct CT{
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* ct_uart_handle;
	DMA_HandleTypeDef* ct_dma_handle;
	// Event flags
	TX_EVENT_FLAGS_GROUP* event_flags;
	// The buffer written to by CT sensor
	char* data_buf;
	// Arrays to hold conductivity/temp values
	ct_samples samples_buf[10];
	ct_samples averages;
	// Keep track of the number of samples
	uint32_t total_samples;
	// Function pointers
	ct_error_code_t (*parse_sample) (struct CT* self);
	ct_samples (*get_averages) (struct CT* self);
	ct_error_code_t (*shutdown) (struct CT* self);
	ct_error_code_t (*self_test) (struct CT* self);
} CT;

void ct_init(CT* self, UART_HandleTypeDef* ct_uart_handle, DMA_HandleTypeDef* ct_dma_handle,
		TX_EVENT_FLAGS_GROUP* event_flags, char* data_buf);
ct_error_code_t ct_parse_sample(CT* self);
ct_samples ct_get_averages(CT* self);
ct_error_code_t ct_shutdown(CT* self);
ct_error_code_t ct_self_test(CT* self);

#endif /* SRC_CT_H_ */
