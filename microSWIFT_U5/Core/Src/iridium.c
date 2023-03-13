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
static iridium_error_code_t send_msg_from_queue(Iridium* self);
static void transmit_timeout_callback(TIM_HandleTypeDef *htim);
static iridium_error_code_t transmit_message(uint8_t* payload);

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
	HAL_TIM_RegisterCallback(self->ten_min_timer, HAL_TIM_PERIOD_ELAPSED_CB_ID,
			transmit_timeout_callback);
	self->event_flags = event_flags;
	self->message_buffer = message_buffer;
	self->response_buffer = response_buffer;
	self->transmit_timeout = &timer_timeout;
	self->current_lat = 0;
	self->current_long = 0;
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
	bool message_tx_success = false;
	bool all_messages_sent = false;

	// Make sure we can get an acknowledgment from the modem
	if (get_ack_message(self) != IRIDIUM_SUCCESS){
		return IRIDIUM_UART_ERROR;
	}
	// get the most recent location
	self->get_location(self);

	// Start the timer
	self->transmit_timeout = false;
	HAL_TIM_Base_Start(self->ten_min_timer);

	// Send the message that was just generated
	while (!self->transmit_timeout && !message_tx_success) {
		return_code = transmit_message(self->message_buffer);
	}

	// If we have time, send messages from the queue
	while (!self->transmit_timeout && !all_messages_sent) {

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

	// TODO: This will get implimented later to take the contents of the
	//       storage queue and stuff it in flash at some high page(s)

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

	LL_DMA_ResetChannel(GPDMA1, IRIDIUM_LL_TX_DMA_HANDLE);
	LL_DMA_ResetChannel(GPDMA1, IRIDIUM_LL_RX_DMA_HANDLE);

	return IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static uint16_t get_checksum(uint8_t* payload, size_t payload_size)
{
	uint64_t sum = 0;
	for (int i = 0; i < payload_size; i++) {
		sum += payload[i];
	}

	return (uint16_t)sum;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static iridium_error_code_t get_ack_message(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
		// Quick ack message
		const char* ack = "AT\r";
		// Disable flow control message
		const char* disable_flow_control = "AT&K0\r";
		// will become location of 'O' in "OK\r" response from modem
		char * needle;
		uint8_t fail_counter;
		uint8_t ack_buffer[ACK_MESSAGE_SIZE];
		uint16_t num_bytes_received = 0;
		// Zero out the buffer
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

		for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
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

		if (fail_counter == MAX_RETRIES){
			return IRIDIUM_SELF_TEST_FAILED;
		}


		for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
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

		if (fail_counter == MAX_RETRIES) return_code = IRIDIUM_SELF_TEST_FAILED;

		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

		return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
void iridium_storage_queue_create(Iridium* self)
{
	// place the storage queue in SRAM 4
	self->storage_queue = (Iridium_message_queue*) SRAM4_START_ADDR;
	// Zero out the queue space
	memset(self->storage_queue->msg_queue, 0, STORAGE_QUEUE_SIZE);
	self->storage_queue->num_msgs_enqueued = 0;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_storage_queue_add(Iridium* self, uint8_t* payload)
{
	if (self->storage_queue->num_msgs_enqueued == MAX_SRAM4_MESSAGES) {
		return IRIDIUM_STORAGE_QUEUE_FULL;
	}

	for (int i = 0; i < MAX_SRAM4_MESSAGES; i ++) {
		if (!self->storage_queue->msg_queue[i].valid) {
			// copy the message over
			memcpy(self->storage_queue->msg_queue[i].payload, payload, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
			// Make the entry valid
			self->storage_queue->msg_queue[i].valid = true;
			self->storage_queue->num_msgs_enqueued++;
			break;
		}
	}

	return IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_storage_queue_get(Iridium* self, uint8_t* retreived_payload)
{
	float significant_wave_height = 0.0;
	float msg_wave_float = 0.0;
	real16_T msg_wave_height;
	msg_wave_height.bitPattern = 0;
	uint8_t next_msg_index = 0;

	if (self->storage_queue->num_msgs_enqueued == 0) {
		return IRIDIUM_STORAGE_QUEUE_EMPTY;
	}

	for (int i = 0; i < MAX_SRAM4_MESSAGES; i++) {
		if (self->storage_queue->msg_queue[i].valid) {
			msg_wave_height.bitPattern = self->storage_queue->msg_queue[i].payload[4] +
					(self->storage_queue->msg_queue[i].payload[5] << 8);
			msg_wave_float = halfToFloat(msg_wave_height);
			if (msg_wave_float >= significant_wave_height) {
				significant_wave_height = msg_wave_float;
				next_msg_index = i;
			}
		}
	}

	retreived_payload = &(self->storage_queue->msg_queue[next_msg_index].payload[0]);

	return IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
void iridium_storage_queue_flush(Iridium* self)
{
	memset(self->storage_queue->msg_queue, 0, STORAGE_QUEUE_SIZE);
	self->storage_queue->num_msgs_enqueued = 0;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static iridium_error_code_t send_msg_from_queue(Iridium* self) {
	// MUST remember to mark msg as invalid once sent!!
}

/**
 * Callback function ISR for timer period elapsed timeout
 *
 * @param htim - timer handle that called this callback
 * @return void
 */
static void transmit_timeout_callback(TIM_HandleTypeDef *htim) {
	// Make sure we were called by the Iridium timer
	if (htim->Instance == IRIDIUM_TIMER_INSTANCE) {
		timer_timeout = true;
	}
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static iridium_error_code_t transmit_message(uint8_t* payload)
{
//	static const char* sbd = "AT+SBDWB=340\r";
//	static const char* ready = "READY\r";
	int fail_counter;
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(sbd[0]),
				strlen(sbd), 1000);

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

}
