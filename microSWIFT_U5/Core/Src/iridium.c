/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include <iridium.h>

// Static functions
static void get_checksum(uint8_t* payload, size_t payload_size);
static iridium_error_code_t send_basic_command_message(Iridium* self,
		const char* command, uint8_t response_size, uint32_t wait_time);
static iridium_error_code_t send_msg_from_queue(Iridium* self);
static iridium_error_code_t internal_transmit_message(Iridium* self, uint8_t* payload);
static uint8_t get_signal_strength(Iridium* self);
static void cycle_power(Iridium* self);

// static variables
static const char* ack = "AT\r";
static const char* disable_flow_control = "AT&K0\r";
static const char* enable_ring_indications = "AT+SBDMTA=1\r";
static const char* store_config = "AT&W0\r";
static const char* select_power_up_profile = "AT&Y0\r";
static const char* clear_MO = "AT+SBDD0\r";
static const char* load_sbd = "AT+SBDWB=327\r";
static const char* send_sbd = "AT+SBDI\r";
static const char* signal_strength = "AT+CSQ\r";

/**
 * Initialize the CT struct
 *
 * @return void
 */
void iridium_init(Iridium* self, UART_HandleTypeDef* iridium_uart_handle,
		DMA_HandleTypeDef* iridium_rx_dma_handle, TIM_HandleTypeDef* timer,
		DMA_HandleTypeDef* iridium_tx_dma_handle,TX_EVENT_FLAGS_GROUP* event_flags,
		uint8_t* message_buffer, uint8_t* response_buffer)
{
	self->iridium_uart_handle = iridium_uart_handle;
	self->iridium_rx_dma_handle = iridium_rx_dma_handle;
	self->iridium_tx_dma_handle = iridium_tx_dma_handle;
	self->timer = timer;
	self->event_flags = event_flags;
	self->message_buffer = message_buffer;
	self->response_buffer = response_buffer;
	self->current_lat = 0;
	self->current_long = 0;
	self->timer_timeout = false;
	self->config = iridium_config;
	self->self_test = iridium_self_test;
	self->transmit_message = iridium_transmit_message;
	self->transmit_error_message = iridium_transmit_error_message;
	self->get_location = iridium_get_location;
	self->on_off = iridium_on_off;
	self->store_in_flash = iridium_store_in_flash;
	self->reset_uart = iridium_reset_iridium_uart;
	self->reset_timer = iridium_reset_timer;
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
	int fail_counter;

	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		// Get an ack message
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE, ONE_SECOND)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
			continue;
		}
		// disable flow control
		if (send_basic_command_message(self, disable_flow_control, DISABLE_FLOW_CTRL_SIZE,
				ONE_SECOND) == IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
			continue;
		}
		// enable SBD ring indications
		if (send_basic_command_message(self, enable_ring_indications, ENABLE_RI_SIZE,
				ONE_SECOND) == IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
			continue;
		}
		// Store this configuration as profile 0
		if (send_basic_command_message(self, store_config, STORE_CONFIG_SIZE, ONE_SECOND)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
			continue;
		}
		// set profile 0 as the power-up profile
		if (send_basic_command_message(self, select_power_up_profile, SELECT_PWR_UP_SIZE,
				ONE_SECOND) == IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
			continue;
		} else {
			break;
		}
	}

	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	return (fail_counter == MAX_RETRIES) ? IRIDIUM_UART_ERROR : IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_self_test(Iridium* self)
{
	int fail_counter;
	uint32_t elapsed_time = 0;
	// Power the unit by pulling the sleep pin to ground.
	self->on_off(self, true);
	// Start the timer
	HAL_TIM_Base_Start(self->timer);
	// Wait an appropriate amount of time for the caps to charge
	while (elapsed_time < IRIDIUM_CAP_CHARGE_TIME) {
		HAL_Delay(1000);
		elapsed_time = __HAL_TIM_GET_COUNTER(self->timer);
	}
	HAL_TIM_Base_Stop(self->timer);

	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		// Get an ack message
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE, ONE_SECOND)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
		} else {
			break;
		}
	}

	return (fail_counter == MAX_RETRIES) ? IRIDIUM_UART_ERROR : IRIDIUM_SUCCESS;
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
 * Function to change GPIO pin state for the sleep/ on-off pin.
 * The unit is powered on when the pin is high, off when the pin
 * is pulled to ground.
 *
 * @return void
 */
void iridium_on_off(Iridium* self, bool on)
{
	HAL_GPIO_WritePin(GPIOD, IRIDIUM_OnOff_Pin, on);
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
	self->iridium_uart_handle->Instance = IRIDIUM_UART_INSTANCE;
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
iridium_error_code_t iridium_reset_timer(Iridium* self, uint8_t timeout_in_minutes)
{
	if (HAL_TIM_Base_DeInit(self->timer) != HAL_OK) {
		return IRIDIUM_TIMER_ERROR;
	}

	self->timer->Instance = IRIDIUM_TIMER_INSTANCE;
	self->timer->Init.Prescaler = 12000;
	self->timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	self->timer->Init.Period = 59999;
	self->timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	self->timer->Init.RepetitionCounter = timeout_in_minutes;
	self->timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(self->timer) != HAL_OK) {
		return IRIDIUM_TIMER_ERROR;
	}

	return IRIDIUM_SUCCESS;
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
			// Try following that! Gets the absolute value of msg_wave_height
			msg_wave_float = (float)fabs((double)halfToFloat(msg_wave_height));

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
static iridium_error_code_t send_msg_from_queue(Iridium* self)
{
	iridium_error_code_t return_code;
	uint8_t* payload_index = 0;
	return_code = self->queue_get(self, payload_index);
	if (return_code == IRIDIUM_STORAGE_QUEUE_EMPTY) {
		return return_code;
	}
	// try transmitting the message
	return_code = internal_transmit_message(self, self->storage_queue->msg_queue[*payload_index].payload);
	// If the message successfully transmitted, mark it as invalid
	if (return_code == IRIDIUM_SUCCESS) {
		self->storage_queue->msg_queue[*payload_index].valid = false;
		self->storage_queue->num_msgs_enqueued--;
	}
	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static void get_checksum(uint8_t* payload, size_t payload_size)
{
	uint16_t checksum = 0;
	uint8_t* checksum_ptr = (uint8_t*)&checksum;
	// calculate checksum
	for (int i = 0; i < payload_size; i++) {
		checksum += payload[i];
	}
	// place checksum in the last two bytes of the payload array
	payload[CHECKSUM_SECOND_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
	checksum_ptr++;
	payload[CHECKSUM_FIRST_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static iridium_error_code_t send_basic_command_message(Iridium* self,
		const char* command, uint8_t response_size, uint32_t wait_time)
{
	char * needle;

	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

	HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(command[0]),
			strlen(command), wait_time);
	HAL_UART_Receive(self->iridium_uart_handle, &(self->response_buffer[0]),
			response_size, wait_time);

	needle = strstr((char*)&(self->response_buffer[0]), "OK");
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	return (needle == NULL) ? IRIDIUM_COMMAND_RESPONSE_ERROR : IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_transmit_message(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	iridium_error_code_t queue_return_code = IRIDIUM_SUCCESS;
	int fail_counter;
	bool message_tx_success = false;
	bool all_messages_sent = false;

	// Make sure we can get an acknowledgment from the modem
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE, ONE_SECOND)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
		} else {
			break;
		}
	}

	if (fail_counter == MAX_RETRIES) {
		return IRIDIUM_UART_ERROR;
	}

	// reset the timer and clear the interrupt flag
	self->reset_timer(self, IRIDIUM_MAX_TRANSMIT_PERIOD);
	self->timer_timeout = false;
	__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);
	// Start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(self->timer);
	// Send the message that was just generated
	while (!self->timer_timeout && !message_tx_success) {
		return_code = internal_transmit_message(self, self->message_buffer);
		message_tx_success = return_code == IRIDIUM_SUCCESS;
	}

	// Message failed to send. If there is space in the queue, store it,
	// otherwise return IRIDIUM_STORAGE_QUEUE_FULL
	if (self->timer_timeout && !message_tx_success) {

			// reset the timer and clear the flag for the next time
			HAL_TIM_Base_Stop_IT(self->timer);
			self->timer_timeout = false;
			__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);

			return self->queue_add(self, self->message_buffer);
	}

	// If we made it here, there's still time left, try sending a queued message
	// First, make sure we actually have messages in the queue
	all_messages_sent = self->storage_queue->num_msgs_enqueued == 0;
	// If we have time, send messages from the queue
	while (!self->timer_timeout && !all_messages_sent) {
		queue_return_code = send_msg_from_queue(self);
		all_messages_sent = self->storage_queue->num_msgs_enqueued == 0;
	}

	// reset the timer and clear the flag for the next time
	HAL_TIM_Base_Stop_IT(self->timer);
	self->timer_timeout = false;
	__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);
	return return_code;
}

/**
 * Helper method to transmit a message via Iridium modem. Remember, ASCII chars
 * are actually ints.
 *
 *
 * @param payload - pointer to a SBD message payload
 * @return iridium_error_code_t - Either IRIDIUM_UART_ERROR or IRIDIUM_SUCCESS
 */
static iridium_error_code_t internal_transmit_message(Iridium* self, uint8_t* payload)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	char* needle;
	char SBDWB_response_code;
	int SBDI_response_code;
	int fail_counter;
	bool network_available = false;
	uint32_t adaptive_delay_time[5] = {ONE_SECOND * 3, ONE_SECOND * 5,
			ONE_SECOND * 30, ONE_SECOND * 60, ONE_SECOND * 180};

	// get the checksum
	get_checksum(payload, IRIDIUM_MESSAGE_PAYLOAD_SIZE);

	for (fail_counter = 0; fail_counter < MAX_RETRIES && !self->timer_timeout; fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(load_sbd[0]),
				strlen(load_sbd), ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDWB_READY_RESPONSE_SIZE, ONE_SECOND);

		needle = strstr((char*)&(self->response_buffer[0]), "READY");
		// Success case
		if (needle != NULL) {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}

		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	}

	if (fail_counter == MAX_RETRIES) { return IRIDIUM_UART_ERROR;}

	for (fail_counter = 0; fail_counter < MAX_RETRIES && !self->timer_timeout; fail_counter++) {
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(payload[0]),
				IRIDIUM_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH, ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDWB_LOAD_RESPONSE_SIZE, ONE_SECOND);

		SBDWB_response_code = self->response_buffer[SBDWB_RESPONSE_CODE_INDEX];
		// Success case
		if (SBDWB_response_code == '0') {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			break;
		}

		// Response of 2 means checksum didn't match, so try calculating it again
		if (SBDWB_response_code == '2') {
			get_checksum(payload, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
		}

		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	}

	if (fail_counter == MAX_RETRIES) { return IRIDIUM_UART_ERROR;}

	for (fail_counter = 0; fail_counter < MAX_RETRIES && !self->timer_timeout; fail_counter++) {
		// TODO: add location at the end of the message
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(send_sbd[0]),
				strlen(send_sbd), ONE_SECOND);
		// We will only grab the response up to and including MO status
		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDI_RESPONSE_SIZE, ONE_SECOND * 30);
		// Grab the MO status
		SBDI_response_code = atoi((char*)&(self->response_buffer[SBDI_RESPONSE_CODE_INDEX]));

		switch (SBDI_response_code) {

			case 0: // No message transfered to modem
				return_code = IRIDIUM_UART_ERROR;
				// force outer loop to break
				fail_counter = MAX_RETRIES;
				break;

			case 1: // success case
				if (send_basic_command_message(self, clear_MO, SBDD_RESPONSE_SIZE, ONE_SECOND * 10) ==
						IRIDIUM_COMMAND_RESPONSE_ERROR) {
					cycle_power(self);
				}
				// force outer loop to break
				fail_counter = MAX_RETRIES;
				break;

			case 2: // Message Tx unsuccessful, try again
				HAL_Delay(adaptive_delay_time[fail_counter % 5]);
				network_available = HAL_GPIO_ReadPin(GPIOD, IRIDIUM_NetAv_Pin);
				while (network_available == false && !self->timer_timeout) {
					network_available = HAL_GPIO_ReadPin(GPIOD, IRIDIUM_NetAv_Pin);
					HAL_Delay(5);
				}
				self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
				continue;

			default: // Response didn't make sense
				// Since response was unexpected, cycle power and reset UART
				return_code = IRIDIUM_UNKNOWN_ERROR;
				cycle_power(self);
				self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
				break;
		}
	}

	if (fail_counter == MAX_RETRIES && return_code != IRIDIUM_UNKNOWN_ERROR) {
		return_code = IRIDIUM_TRANSMIT_ERROR;
	}

	return return_code;
}

/**
 * Helper method to get the current signal strength.
 *
 * @param self - Iridium struct
 * @param error_message - null terminated error message of 320 bytes or less. Note that this
 * 						  string is mutable to handle the case of terminating a too-long string
 * @return IRIDIUM_SUCCESS to indicate message was sent
 * 		   IRIDIUM_TRANSMIT_ERROR if it didn't send
 * 		   IRIDIUM_UART_ERROR if something went wrong trying to talk to the modem
 */
iridium_error_code_t iridium_transmit_error_message(Iridium* self, char* error_message)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	uint32_t str_length = strlen(error_message);
	bool message_tx_success = false;

	// If the error message is too long, we'll just cut it off
	if (str_length > ERROR_MESSAGE_MAX_LENGTH - 1) {
		error_message[ERROR_MESSAGE_MAX_LENGTH - 1] = 0;
	}

	// reset the timer and clear the interrupt flag
	self->reset_timer(self, IRIDIUM_MAX_TRANSMIT_PERIOD);
	__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);
	// Start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(self->timer);
	// Send the message that was just generated
	while (!self->timer_timeout && !message_tx_success) {
		return_code = internal_transmit_message(self, self->message_buffer);
		message_tx_success = return_code == IRIDIUM_SUCCESS;
	}

	// Message failed to send.
	if (self->timer_timeout && !message_tx_success) {

			// reset the timer and clear the flag for the next time
			HAL_TIM_Base_Stop_IT(self->timer);
			self->timer_timeout = false;
			__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);

			return_code = IRIDIUM_TRANSMIT_ERROR;
	}

	return return_code;
}

/**
 * Helper method to cycle power to the modem via the sleep pin.
 * Iridium developers manual states a 2 second wait period should
 * be adhered to when cycling power.
 *
 * @param self - Iridium struct
 * @return void
 */
static void cycle_power(Iridium* self)
{
	self->on_off(self, false);
	HAL_Delay(2100);
	self->on_off(self, true);
	HAL_Delay(2100);
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

