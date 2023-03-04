/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include <iridium.h>

// Static functions
static uint16_t get_checksum(uint8_t* payload, size_t payload_size);

/**
 * Initialize the CT struct
 *
 * @return void
 */
void iridium_init(Iridium* self, UART_HandleTypeDef* iridium_uart_handle,
		DMA_HandleTypeDef* iridium_rx_dma_handle, TIM_HandleTypeDef* ten_min_timer,
		DMA_HandleTypeDef* iridium_tx_dma_handle,TX_EVENT_FLAGS_GROUP* event_flags,
		uint8_t* message_buffer, uint8_t* response_buffer)
{
	self->iridium_uart_handle = iridium_uart_handle;
	self->iridium_rx_dma_handle = iridium_rx_dma_handle;
	self->iridium_tx_dma_handle = iridium_tx_dma_handle;
	self->ten_min_timer = ten_min_timer;
	self->event_flags = event_flags;
	self->message_buffer = message_buffer;
	self->response_buffer = response_buffer;
	self->current_lat = 0;
	self->current_long = 0;
	self->current_flash_page = 0; //TODO: figure out how to do this
	self->current_message_transmit_attempts = 0;
	self->config = iridium_config;
	self->self_test = iridium_self_test;
	self->transmit_message = iridium_transmit_message;
	self->get_location = iridium_get_location;
	self->sleep = iridium_sleep;
	self->store_in_flash = iridium_store_in_flash;
	self->reset_uart = iridium_reset_iridium_uart;
	// TODO: figure out if this is a good idea
	memset(&(self->message_buffer[0]), 0, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_config(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_self_test(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	// Quick ack message
	const char* ack = "AT\r";
	// Disable flow control message
	const char* disable_flow_control = "AT&K0\r";
	// will become location of 'O' in "OK\r" response from modem
	char * needle;
	uint8_t receive_fail_counter;
	uint16_t num_bytes_received = 0;
	uint32_t elapsed_time;
	bool add_warm_up_time = true;
	// Zero out the buffer
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

	// Start the timer
	HAL_TIM_Base_Start(self->ten_min_timer);
	elapsed_time = __HAL_TIM_GET_COUNTER(self->ten_min_timer);
	while (elapsed_time < IRIDIUM_CAP_CHARGE_TIME) {
		elapsed_time = __HAL_TIM_GET_COUNTER(self->ten_min_timer);
	}
	HAL_TIM_Base_Stop(self->ten_min_timer);

	receive_fail_counter = 0;
	for (receive_fail_counter = 0; receive_fail_counter < 10; receive_fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(ack[0]),
				strlen(ack), 1000);

		HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
				&(self->response_buffer[0]), IRIDIUM_MAX_RESPONSE_SIZE, &num_bytes_received, 1250);

		needle = strstr((char*)self->response_buffer, "OK");
		if (needle != NULL) {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	}

	if (receive_fail_counter == 2){
		return IRIDIUM_SELF_TEST_FAILED;
	}

//	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	num_bytes_received = 0;

	for (receive_fail_counter = 0; receive_fail_counter < 10; receive_fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(disable_flow_control[0]),
				strlen(disable_flow_control), 1000);

		HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
				&(self->response_buffer[0]), IRIDIUM_MAX_RESPONSE_SIZE, &num_bytes_received, 1250);

		needle = strstr((char*)self->response_buffer, "OK");
		if (needle != NULL) {
			break;
		}
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	}

	if (receive_fail_counter == 2) return_code = IRIDIUM_SELF_TEST_FAILED;

	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

	// TODO: toggle OnOff pin here

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_transmit_message(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_get_location(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_sleep(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_store_in_flash(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_reset_iridium_uart(Iridium* self, uint16_t baud_rate)
{

	if (HAL_UART_DeInit(self->iridium_uart_handle) != HAL_OK) {
		return IRIDIUM_UART_ERROR;
	}
	self->iridium_uart_handle->Instance = UART5;
	self->iridium_uart_handle->Init.BaudRate = baud_rate;
	self->iridium_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	self->iridium_uart_handle->Init.StopBits = UART_STOPBITS_1;
	self->iridium_uart_handle->Init.Parity = UART_PARITY_NONE;
	self->iridium_uart_handle->Init.Mode = UART_MODE_TX_RX;
	self->iridium_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->iridium_uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
	self->iridium_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	self->iridium_uart_handle->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	self->iridium_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(self->iridium_uart_handle) != HAL_OK)
	{
	  return IRIDIUM_UART_ERROR;
	}
	if (HAL_UARTEx_SetTxFifoThreshold(self->iridium_uart_handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	  return IRIDIUM_UART_ERROR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(self->iridium_uart_handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	  return IRIDIUM_UART_ERROR;
	}
	if (HAL_UARTEx_DisableFifoMode(self->iridium_uart_handle) != HAL_OK)
	{
	  return IRIDIUM_UART_ERROR;
	}

	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_2);
	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_3);

	return IRIDIUM_SUCCESS;
}

static uint16_t get_checksum(uint8_t* payload, size_t payload_size)
{

}

