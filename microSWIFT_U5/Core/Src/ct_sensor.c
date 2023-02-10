/*
 * CT.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "ct_sensor.h"

static ct_error_code_t reset_ct_uart(CT* self, uint16_t baud_rate);

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

	memset(&(self->data_buf[0]),0,sizeof(290 * 2));
}

/**
 *
 *
 * @return ct_error_code_t
 */
ct_error_code_t ct_parse_sample(CT* self)
{
	ULONG actual_flags;
	ct_error_code_t return_code = CT_PARSING_ERROR;
	// The first round is a guaranteed fail since the message hasn't
	// been requested yet, so start at -1 to account for that
	int fail_counter = -1;

	while(++fail_counter < 10) {
		// See if we got the message, otherwise retry
		if (tx_event_flags_get(self->event_flags, CT_READY, TX_OR_CLEAR,
				&actual_flags, TX_NO_WAIT) != TX_SUCCESS) {

			reset_ct_uart(self, 9600);
			HAL_UART_Receive_DMA(self->ct_uart_handle,
				(uint8_t*)&(self->data_buf[0]), CT_DATA_ARRAY_SIZE);
			__HAL_DMA_DISABLE_IT(self->ct_dma_handle, DMA_IT_HT);
			// Messages are sent at 0.5Hz from the sensor, we want to grab 2
			HAL_Delay(2125);
			continue;
		}

		char* index = strstr(self->data_buf, "mS/cm");
		if (index == NULL || index > &(self->data_buf[0]) + 240){
			continue;
		}

		index += 7;
		double conductivity = atof(index);

		if (conductivity == 0.0){
			continue;
		}

		index = strstr(self->data_buf, "Deg.C");
		if (index == NULL){
			continue;
		}
		index += 6;
		double temperature = atof(index);

		if (temperature == 0.0){
			continue;
		}

		return_code = CT_SUCCESS;
		break;

		self->samples_buf[self->total_samples].conductivity = conductivity;
		self->samples_buf[self->total_samples].temp = temperature;
		return_code = CT_SUCCESS;
		break;

	}

	return return_code;
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
	ULONG actual_flags;
	ct_error_code_t return_code = CT_SELF_TEST_FAIL;
	// The first round is a guaranteed fail since the message hasn't
	// been requested yet, so start at -1 to account for that
	int fail_counter = -1;

	while(++fail_counter < 10) {
		// See if we got the message, otherwise retry
		if (tx_event_flags_get(self->event_flags, CT_READY, TX_OR_CLEAR,
				&actual_flags, TX_NO_WAIT) != TX_SUCCESS) {

			reset_ct_uart(self, 9600);
			HAL_UART_Receive_DMA(self->ct_uart_handle,
				(uint8_t*)&(self->data_buf[0]), CT_DATA_ARRAY_SIZE);
			__HAL_DMA_DISABLE_IT(self->ct_dma_handle, DMA_IT_HT);
			// Messages are sent at 0.5Hz from the sensor, we want to grab 2
			HAL_Delay(9000);
			continue;
		}

		char* index = strstr(self->data_buf, "mS/cm");
		if (index == NULL || index > &(self->data_buf[0]) + 240){
			continue;
		}

		index += 7;
		double conductivity = atof(index);

		if (conductivity == 0.0){
			continue;
		}

		index = strstr(self->data_buf, "Deg.C");
		if (index == NULL){
			continue;
		}
		index += 6;
		double temperature = atof(index);

		if (temperature == 0.0){
			continue;
		}

		return_code = CT_SUCCESS;
		break;
	}

	return return_code;
}

/**
 * Reinitialize the GNSS UART port. Required when switching between Tx and Rx.
 *
 * @param self - GNSS struct
 * @param baud_rate - baud rate to set port to
 */
static ct_error_code_t reset_ct_uart(CT* self, uint16_t baud_rate)
{

	if (HAL_UART_DeInit(self->ct_uart_handle) != HAL_OK) {
		return CT_UART_ERROR;
	}

	self->ct_uart_handle->Instance = UART4;
	self->ct_uart_handle->Init.BaudRate = baud_rate;
	self->ct_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	self->ct_uart_handle->Init.StopBits = UART_STOPBITS_1;
	self->ct_uart_handle->Init.Parity = UART_PARITY_NONE;
	self->ct_uart_handle->Init.Mode = UART_MODE_TX_RX;
	self->ct_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->ct_uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
	self->ct_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	self->ct_uart_handle->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	self->ct_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(self->ct_uart_handle) != HAL_OK)
	{
		return CT_UART_ERROR;
	}
	if (HAL_UARTEx_SetTxFifoThreshold(self->ct_uart_handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return CT_UART_ERROR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(self->ct_uart_handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return CT_UART_ERROR;
	}
	if (HAL_UARTEx_DisableFifoMode(self->ct_uart_handle) != HAL_OK)
	{
		return CT_UART_ERROR;
	}

	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_1);

	return CT_SUCCESS;
}

