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
static iridium_error_code_t transmit_message(Iridium* self, uint8_t* payload);
static uint8_t get_signal_strength(Iridium* self);
// ISR callback functions
static void transmit_timeout_callback(TIM_HandleTypeDef *htim);
static void uart_receive_dma_callback(UART_HandleTypeDef *huart);

// static variables
static bool timer_timeout = false;
static bool dma_message_received = false;
// const strings for Iridium AT commands
static const char* ack = "AT\r";
static const char* disable_flow_control = "AT&K0\r";
static const char* load_sbd = "AT+SBDWB=327\r";
static const char* send_sbd = "AT+SBDIX\r";
static const char* signal_strength = "AT+CSQ\r";

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
	self->transmit_timeout = timer_timeout;
	self->dma_message_received = dma_message_received;
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
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

//	HAL_UART_RegisterCallback(self->iridium_uart_handle, HAL_UART_RX_COMPLETE_CB_ID,
//			uart_receive_dma_callback);

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
		return_code = transmit_message(self, self->message_buffer);
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

//	HAL_GPIO_WritePin(GPIOD, 7, on);
	HAL_GPIO_TogglePin(GPIOD, 7);
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
		// will mark location of 'O' in "OK\r" response from modem
		char * needle;
		uint8_t fail_counter;
		uint16_t num_bytes_received = 0;
		ULONG actual_flags;
		// Zero out the response buffer
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
//		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

		for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(ack[0]),
					strlen(ack), ONE_SECOND);
//			HAL_UART_Receive_DMA(self->iridium_uart_handle,
//					&(self->response_buffer[0]), ACK_MESSAGE_SIZE);
//			tx_event_flags_get(self->event_flags, IRIDIUM_MSG_RECVD, TX_OR_CLEAR,
//					&actual_flags, TX_TIMER_TICKS_PER_SECOND);

			HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
							&(self->response_buffer[0]), ACK_MESSAGE_SIZE,
							&num_bytes_received, ONE_SECOND);

//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			needle = strstr((char*)&(self->response_buffer[0]), "OK");
			if (needle != NULL) {
				memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
				break;
			}

			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			self->on_off(self, false);
			HAL_Delay(100);
		}

		if (fail_counter == MAX_RETRIES){
			return IRIDIUM_SELF_TEST_FAILED;
		}

		for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(disable_flow_control[0]),
					strlen(disable_flow_control), ONE_SECOND);

//			HAL_UART_Receive_DMA(self->iridium_uart_handle,
//					&(self->response_buffer[0]), DISABLE_FLOW_CTRL_SIZE);
//			tx_event_flags_get(self->event_flags, IRIDIUM_MSG_RECVD, TX_OR_CLEAR,
//					&actual_flags, TX_TIMER_TICKS_PER_SECOND);

			HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
										&(self->response_buffer[0]), DISABLE_FLOW_CTRL_SIZE,
										&num_bytes_received, ONE_SECOND);


//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			needle = strstr((char*)&(self->response_buffer[0]), "OK");
			if (needle != NULL) {
				break;
			}

			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			//self->on_off(self, false);
			HAL_Delay(100);
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
			memcpy(self->storage_queue->msg_queue[i].payload, payload,
					IRIDIUM_MESSAGE_PAYLOAD_SIZE);
			// Make the entry valid
			self->storage_queue->msg_queue[i].valid = true;
			self->storage_queue->num_msgs_enqueued++;
			break;
		}
	}

	return IRIDIUM_SUCCESS;
}

/**
 * Find the next message to be sent from the storage queue based on significant wave height.
 * NOTE: does not mark message as invalid -- this must be done by the sender.
 *
 * @param self- Iridium struct
 * @param msg_index - return parameter for next message index
 * @return iridium_error_code_t - either IRIDIUM_STORAGE_QUEUE_EMPTY or IRIDIUM_SUCCESS
 */
iridium_error_code_t iridium_storage_queue_get(Iridium* self, uint8_t* msg_index)
{
	float significant_wave_height = 0.0;
	float msg_wave_float = 0.0;
	real16_T msg_wave_height;
	msg_wave_height.bitPattern = 0;

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
				*msg_index = i;
			}
		}
	}

	return IRIDIUM_SUCCESS;
}

/**
 * Flushes the message storage queue.
 *
 * @param self- Iridium struct
 * @return void
 */
void iridium_storage_queue_flush(Iridium* self)
{
	memset(self->storage_queue->msg_queue, 0, STORAGE_QUEUE_SIZE);
	self->storage_queue->num_msgs_enqueued = 0;
}

/**
 * Static helper message to send message from the queue.
 *
 * @param self- Iridium struct
 * @return iridium_error_code_t
 */
static iridium_error_code_t send_msg_from_queue(Iridium* self) {
	// MUST remember to mark msg as invalid once sent!!
}

/**
 * Helper method to transmit a message via Iridium modem.
 *
 * @param payload - pointer to a 340 byte SBD message
 * @return iridium_error_code_t - Either IRIDIUM_UART_ERROR or IRIDIUM_SUCCESS
 */
static iridium_error_code_t transmit_message(Iridium* self, uint8_t* payload)
{
	char* needle;
	char response_code;
	int SBDIX_response_code;
	int fail_counter;
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	uint16_t num_bytes_received;
	uint16_t checksum;
	uint8_t* checksum_ptr = (uint8_t*)&checksum;
	uint32_t adaptive_delay_time[5] = {ONE_SECOND * 3, ONE_SECOND * 5,
			ONE_SECOND * 30, ONE_SECOND * 60, ONE_SECOND * 180};

	// get the checksum and place the bits in the last 2 bytes of the payload
	checksum = get_checksum(payload, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
	payload[CHECKSUM_SECOND_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
	checksum_ptr++;
	payload[CHECKSUM_FIRST_BYTE_INDEX] = ((uint8_t)*checksum_ptr);

	// Tell the modem we are sending over a 340 byte message
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(load_sbd[0]),
				strlen(load_sbd), ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDWB_READY_RESPONSE_SIZE, ONE_SECOND);

		needle = strstr((char*)&(self->response_buffer[0]), "READY");
		if (needle != NULL) {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}

		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

		//TODO: define return code
	}

	fail_counter = 0;
	// Send over the message and checksum
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(payload[0]),
				IRIDIUM_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH, ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDWB_LOAD_RESPONSE_SIZE, ONE_SECOND);

		response_code = self->response_buffer[SBDWB_RESPONSE_CODE_INDEX];

		// Response of 0 means success
		if (response_code == '0') {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}

		// Response of 2 means checksum didn't match, so try calculating it again
		if (response_code == '2') {
			checksum = get_checksum(payload, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
			payload[CHECKSUM_SECOND_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
			checksum_ptr++;
			payload[CHECKSUM_FIRST_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
		}

		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
		//TODO: define return code
	}

	fail_counter = 0;
	// Tell the modem to send the message
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		// TODO: add location at the end of the message
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(send_sbd[0]),
				strlen(send_sbd), ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDIX_RESPONSE_SIZE, ONE_SECOND * 30);

		response_code = self->response_buffer[SBDIX_RESPONSE_CODE_INDEX];

		// A response code of 0-4 indicates success
		if (response_code == '0' || response_code == '1' || response_code == '3'
				|| response_code == '4') {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}

		// If response did not indicate success, the response code will be defined by 2 chars
		SBDIX_response_code = (self->response_buffer[SBDIX_RESPONSE_CODE_INDEX] - '0') +
				(self->response_buffer[SBDIX_RESPONSE_CODE_INDEX + 1] - '0');

		switch (response_code) {

			case 11:

				break;

			case 16:

				break;

			case 33:

				break;

			case 34:

				break;

			case 36:

				break;

			case 38:

				break;

			case 65:

				break;

			default: // Any other response
				break;
		}

		break;

		//TODO: define return code
	}

	// TODO: stuff
	return return_code;

	//TODO: make each step into a separate function
}

/**
 * Helper method to get the current signal strength.
 *
 * @param self - Iridium struct
 * @return value of signal strength (0xFF to indicate error)
 */
static uint8_t get_signal_strength(Iridium* self)
{
	uint8_t return_val = 0xFF;
	uint8_t fail_counter;
	uint16_t num_bytes_received;
	char* token;
	char delimeter[2] = ":";

	// Request signal strength from the modem
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(signal_strength[0]),
				strlen(signal_strength), ONE_SECOND);

		HAL_UARTEx_ReceiveToIdle(self->iridium_uart_handle,
				&(self->response_buffer[0]), ACK_MESSAGE_SIZE, &num_bytes_received, ONE_SECOND * 10);

		token = strtok(&(self->response_buffer[0]), delimeter);
		if (strstr(token, "+CSQ") == NULL) {
			continue;
		}

		token = strtok(NULL, delimeter);
		if (*token <= ASCII_FIVE && *token >= ASCII_ZERO) {
			return_val = *token - ASCII_ZERO;
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}

		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	}

	return return_val;
}

//////////////////////////////////////////////////////////////////////////////////
////////////////// ISR callback functions ////////////////////////////////////////
/**
 * Callback function ISR for timer period elapsed timeout
 *
 * @param htim - timer handle that called this callback
 * @return void
 */
static void transmit_timeout_callback(TIM_HandleTypeDef *htim)
{
	// Make sure we were called by the Iridium timer
	if (htim->Instance == IRIDIUM_TIMER_INSTANCE) {
		timer_timeout = true;
	}
}

static void uart_receive_dma_callback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == IRIDIUM_UART_INSTANCE) {
			dma_message_received = true;
	}
}
