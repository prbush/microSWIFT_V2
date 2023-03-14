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
#define WARMUP_TIME 20000
#define REQUIRED_CT_SAMPLES 10
#define CT_DATA_ARRAY_SIZE 291

typedef enum {
	CT_SUCCESS = 0,
	CT_UART_ERROR = -1,
	CT_PARSING_ERROR = -2,
	CT_SELF_TEST_FAIL = -3,
	CT_NOT_ENOUGH_SAMPLES = -4
}ct_error_code_t;

typedef struct ct_samples{
	double conductivity;
	double temp;
} ct_samples;

typedef struct CT{
	// The UART and DMA handle for the GNSS interface
	UART_HandleTypeDef* ct_uart_handle;
	DMA_HandleTypeDef* ct_dma_handle;
	// Handle for the millisecond timer
	TIM_HandleTypeDef* millis_timer;
	// Event flags
	TX_EVENT_FLAGS_GROUP* event_flags;
	// The buffer written to by CT sensor
	char* data_buf;
	// Arrays to hold conductivity/temp values
	ct_samples samples_buf[REQUIRED_CT_SAMPLES];
	ct_samples averages;
	// Keep track of the number of samples
	uint32_t total_samples;
	// Function pointers
	ct_error_code_t (*parse_sample) (struct CT* self);
	ct_error_code_t (*get_averages) (struct CT* self);
	ct_error_code_t (*shutdown) (struct CT* self);
	ct_error_code_t (*self_test) (struct CT* self);
	ct_error_code_t (*reset_ct_uart) (struct CT* self, uint16_t baud_rate);
} CT;

void ct_init(CT* self, UART_HandleTypeDef* ct_uart_handle, DMA_HandleTypeDef* ct_dma_handle,
		TIM_HandleTypeDef* millis_timer, TX_EVENT_FLAGS_GROUP* event_flags, char* data_buf);
ct_error_code_t ct_parse_sample(CT* self);
ct_error_code_t ct_get_averages(CT* self);
ct_error_code_t ct_shutdown(CT* self);
ct_error_code_t ct_self_test(CT* self);
ct_error_code_t reset_ct_uart(CT* self, uint16_t baud_rate);

#endif /* SRC_CT_H_ */
