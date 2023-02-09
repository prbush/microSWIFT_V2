/*
 * CT.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "ct_sensor.h"

/**
 * Initialize the CT struct
 *
 * @return void
 */
void ct_init(CT* self, UART_HandleTypeDef* ct_uart_handle, DMA_HandleTypeDef* ct_dma_handle,
		TX_EVENT_FLAGS_GROUP* event_flags, char* data_buf)
{
	for (int i = 0; i < 10; i++) {
		self->samples_buf[i].conductivity = 0.0;
		self->samples_buf[i].temp = 0.0;
	}
	self->averages.conductivity = 0.0;
	self->averages.temp = 0.0;
	self->total_samples = 0;
	self->ct_uart_handle = ct_uart_handle;
	self->ct_dma_handle = ct_dma_handle;
	self->event_flags = event_flags;
	self->data_buf = data_buf;
	self->parse_sample = ct_parse_sample;
	self->get_averages = ct_get_averages;
	self->shutdown = ct_shutdown;
	self->self_test = ct_self_test;
}

/**
 *
 *
 * @return ct_error_code_t
 */
ct_error_code_t ct_parse_sample(CT* self)
{

}

/**
 *
 *
 * @return ct_samples struct containing the averages conductivity
 *         and temperature values
 */
ct_samples ct_get_averages(CT* self)
{

}

/**
 *
 *
 * @return ct_error_code_t
 */
ct_error_code_t ct_shutdown(CT* self)
{

}

/**
 *
 *
 * @return ct_error_code_t
 */
ct_error_code_t ct_self_test(CT* self)
{
	char* index = strstr(self->data_buf, "mS/cm");
	index += 7;
	double num = atof(index);

}

