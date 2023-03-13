/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include <iridium.h>

// Static functions
static uint16_t get_checksum(uint8_t* payload, size_t payload_size);
static iridium_error_code_t get_ack_message(Iridium* self);

// static variables
static bool timer_timeout;
// const strings for Iridium AT commands
static const char* ack = "AT\r";
static const char* disable_flow_control = "AT&K0\r";
static const char* sbd = "AT+SBDWB=340\r";
static const char* ready = "READY\r";

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
	self->storage_queue = (Iridium_message_queue*) SRAM4_START_ADDR;
	self->config = iridium_config;
	self->self_test = iridium_self_test;
	self->transmit_message = iridium_transmit_message;
	self->get_location = iridium_get_location;
	self->on_off = iridium_on_off;
	self->store_in_flash = iridium_store_in_flash;
	self->reset_uart = iridium_reset_iridium_uart;
	self->queue_create = iridium_storage_queue_create;
	self->queue_add = iridium_storage_queue_add;
	self->queue_get = iridium_storage_queue_get;
	self->queue_flush = iridium_storage_queue_flush;
	// TODO: figure out if this is a good idea
	memset(&(self->message_buffer[0]), 0, IRIDIUM_MESSAGE_PAYLOAD_SIZE);

	self->queue_create(self);
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

	uint32_t elapsed_time = 0;
	// Zero out the buffer
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

	// Start the timer
	HAL_TIM_Base_Start(self->ten_min_timer);
	// Wait an appropriate amount of time for the caps to charge
	while (elapsed_time < IRIDIUM_CAP_CHARGE_TIME) {
		HAL_Delay(1000);
		elapsed_time = __HAL_TIM_GET_COUNTER(self->ten_min_timer);
	}
	HAL_TIM_Base_Stop(self->ten_min_timer);

	// Make sure we can get an acknowledgment from the modem
	if (get_ack_message(self) != IRIDIUM_SUCCESS){
		return_code = IRIDIUM_UART_ERROR;
	}

	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

	self->on_off(self, false);

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
	ULONG actual_flags;
	uint32_t elapsed_time = 0;
	bool message_tx_success = false;

	// Wait until the Waves thread is done, at which point the message will be ready
	tx_event_flags_get(self->event_flags, WAVES_DONE, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	// Make sure we can get an acknowledgment from the modem
	if (get_ack_message(self) != IRIDIUM_SUCCESS){
			return IRIDIUM_UART_ERROR;
	}
	// get the most recent location
	self->get_location(self);

	// Start the timer
	HAL_TIM_Base_Start(self->ten_min_timer);
	timer_timeout = false;
	// Go until we have reached the max time
	while (!timer_timeout) {
		//HAL_UART_Transmit(self->iridium_uart_handle
	}

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
iridium_error_code_t iridium_on_off(Iridium* self, bool on)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	// TODO: toggle GPIO pin here
	//		 can probably change return type to void
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

	HAL_FLASH_Unlock();




	HAL_FLASH_Lock();
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

static iridium_error_code_t get_ack_message(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
		// Quick ack message
		const char* ack = "AT\r";
		// Disable flow control message
		const char* disable_flow_control = "AT&K0\r";
		// will become location of 'O' in "OK\r" response from modem
		char * needle;
		uint8_t receive_fail_counter;
		uint8_t ack_buffer[ACK_MESSAGE_SIZE];
		uint16_t num_bytes_received = 0;
		// Zero out the buffer
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

		receive_fail_counter = 0;
		for (receive_fail_counter = 0; receive_fail_counter < MAX_RETRIES; receive_fail_counter++) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(ack[0]),
					strlen(ack), 1000);

			HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
					&(ack_buffer[0]), ACK_MESSAGE_SIZE, &num_bytes_received, ACK_MAX_WAIT_TIME);

			needle = strstr((char*)self->response_buffer, "OK");
			if (needle != NULL) {
				memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
				break;
			}
			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			memset(&(ack_buffer[0]), 0, ACK_MESSAGE_SIZE);
		}

		if (receive_fail_counter == MAX_RETRIES){
			return IRIDIUM_SELF_TEST_FAILED;
		}


		for (receive_fail_counter = 0; receive_fail_counter < MAX_RETRIES; receive_fail_counter++) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(disable_flow_control[0]),
					strlen(disable_flow_control), 1000);

			HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
					&(ack_buffer[0]), ACK_MESSAGE_SIZE, &num_bytes_received, ACK_MAX_WAIT_TIME);

			needle = strstr((char*)self->response_buffer, "OK");
			if (needle != NULL) {
				break;
			}
			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			memset(&(ack_buffer[0]), 0, ACK_MESSAGE_SIZE);
		}

		if (receive_fail_counter == MAX_RETRIES) return_code = IRIDIUM_SELF_TEST_FAILED;

		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

		return return_code;
}

void iridium_storage_queue_create(Iridium* self)
{
	memset(self->storage_queue, 0, STORAGE_QUEUE_SIZE);
	for (int i = 0; i < 48; i++){
		for (int j = 0; j < 340; j++) {
			self->storage_queue->msg_queue[i][j] = 0x1;
		}
	}

	for (int i = 0; i < 48; i++){
		for (int j = 0; j < 340; j++) {
			if (self->storage_queue->msg_queue[i][j] == 0x1){
				self->storage_queue->msg_queue[i][j]++;
			}
		}
	}

}

iridium_error_code_t iridium_storage_queue_add(Iridium* self,uint8_t* payload)
{

}

iridium_error_code_t iridium_storage_queue_get(Iridium* self,uint8_t* retreived_payload)
{

}

iridium_error_code_t iridium_storage_queue_flush(Iridium* self)
{

}
