/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "Peripherals/iridium.h"

// Static functions
static void get_checksum(uint8_t* payload, size_t payload_size);
static iridium_error_code_t send_basic_command_message(Iridium* self,
		const char* command, uint8_t response_size, uint32_t wait_time);
static iridium_error_code_t send_msg_from_queue(Iridium* self);
static iridium_error_code_t internal_transmit_message(Iridium* self,
		uint8_t* payload, uint16_t payload_size);
static void cycle_power(Iridium* self);

// static variables
static const char* ack = "AT\r";
static const char* disable_flow_control = "AT&K0\r";
static const char* enable_ring_indications = "AT+SBDMTA=1\r";
static const char* store_config = "AT&W0\r";
static const char* select_power_up_profile = "AT&Y0\r";
static const char* clear_MO = "AT+SBDD0\r";
static const char* send_sbd = "AT+SBDI\r";

/**
 * Initialize the CT struct
 *
 * @return void
 */
void iridium_init(Iridium* self, microSWIFT_configuration* global_config,
		UART_HandleTypeDef* iridium_uart_handle, DMA_HandleTypeDef* iridium_rx_dma_handle,
		TIM_HandleTypeDef* timer, DMA_HandleTypeDef* iridium_tx_dma_handle,
		TX_EVENT_FLAGS_GROUP* control_flags, TX_EVENT_FLAGS_GROUP* error_flags,
		RTC_HandleTypeDef* rtc_handle, sbd_message_type_52* current_message,
		uint8_t* error_message_buffer, uint8_t* response_buffer,
		Iridium_message_storage* storage_queue)
{
	self->global_config = global_config;
	self->iridium_uart_handle = iridium_uart_handle;
	self->iridium_rx_dma_handle = iridium_rx_dma_handle;
	self->iridium_tx_dma_handle = iridium_tx_dma_handle;
	self->timer = timer;
	self->control_flags = control_flags;
	self->error_flags = error_flags;
	self->rtc_handle = rtc_handle;
	self->current_message = current_message;
	self->error_message_buffer = error_message_buffer;
	self->response_buffer = response_buffer;
	self->storage_queue = storage_queue;
	self->current_lat = 0.0;
	self->current_lon = 0.0;
	self->timer_timeout = false;
	self->config = iridium_config;
	self->self_test = iridium_self_test;
	self->transmit_message = iridium_transmit_message;
	self->transmit_error_message = iridium_transmit_error_message;
	self->get_timestamp = iridium_get_timestamp;
	self->sleep = iridium_sleep;
	self->on_off = iridium_on_off;
	self->store_in_flash = iridium_store_in_flash;
	self->reset_uart = iridium_reset_iridium_uart;
	self->reset_timer = iridium_reset_timer;
	self->queue_add = iridium_storage_queue_add;
	self->queue_get = iridium_storage_queue_get;
	self->queue_flush = iridium_storage_queue_flush;

	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_config(Iridium* self)
{
	int fail_counter;
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

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

	if (fail_counter == MAX_RETRIES) {
		return_code = IRIDIUM_UART_ERROR;
		tx_event_flags_set(self->error_flags, MODEM_ERROR, TX_OR);
	}

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_self_test(Iridium* self, uint32_t warmup_time)
{
	int fail_counter;
	uint32_t start_time = 0, elapsed_time = 0;
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	// Power the unit by pulling the sleep pin to ground.
	self->on_off(self, GPIO_PIN_SET);
	self->sleep(self, GPIO_PIN_SET);

	start_time = HAL_GetTick();
	// Wait an appropriate amount of time for the caps to charge
	while (elapsed_time < warmup_time) {
		HAL_Delay(1000);
		elapsed_time = HAL_GetTick() - start_time;
	}

	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		// Get an ack message
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE, ONE_SECOND)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
		} else {
			break;
		}
	}

	if (fail_counter == MAX_RETRIES) {
		return_code = IRIDIUM_UART_ERROR;
		tx_event_flags_set(self->error_flags, MODEM_ERROR, TX_OR);
	}

	return return_code;
}

/**
 * Function to change GPIO pin state for the sleep/ on-off pin.
 * Note that power is still available at the unit and the capacitors
 * will remain charged, but there is a current consumption.
 *
 * @param self - Iridium struct
 * @param pin_state - GPIO_PIN_SET for on
 * 					  GPIO_PIN_RESET for off
 *
 * @return void
 */
void iridium_sleep(Iridium* self, GPIO_PinState pin_state)
{
	HAL_GPIO_WritePin(GPIOD, IRIDIUM_OnOff_Pin, pin_state);
}

/**
 * Function to change GPIO pin state for the power FET. Note that
 * the capacitors will not remain charged when the device is powered
 * down using this function
 *
 * @param self - Iridium struct
 * @param pin_state - GPIO_PIN_SET for on
 * 					  GPIO_PIN_RESET for off
 *
 * @return void
 */
void iridium_on_off(Iridium* self, GPIO_PinState pin_state)
{
	if (pin_state == GPIO_PIN_SET) {
		HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, pin_state);
		// Wait 10ms between powering 5V bus FET and the Iridium power FET
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, pin_state);
	} else {
		HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, pin_state);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, pin_state);
	}
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
	// For debugging, not practical to set the timeout to 0
	if (timeout_in_minutes <= 0) {
		self->timer->Init.Period = 1;
		self->timer->Init.RepetitionCounter = 0;
	}
	else {
		self->timer->Init.Period = 59999;
		self->timer->Init.RepetitionCounter = timeout_in_minutes - 1;
	}

	self->timer->Instance = IRIDIUM_TIMER_INSTANCE;
	self->timer->Init.Prescaler = 12000;
	self->timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	self->timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	self->timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(self->timer) != HAL_OK) {
		return IRIDIUM_TIMER_ERROR;
	}

	__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);

	return IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_storage_queue_add(Iridium* self, sbd_message_type_52* payload)
{
	if (self->storage_queue->num_msgs_enqueued == MAX_NUM_MSGS_STORED) {
		return IRIDIUM_STORAGE_QUEUE_FULL;
	}

	for (int i = 0; i < MAX_NUM_MSGS_STORED; i ++) {
		if (!self->storage_queue->msg_queue[i].valid) {
			// copy the message over
			memcpy(&(self->storage_queue->msg_queue[i].payload), payload,
					sizeof(sbd_message_type_52));
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

	for (int i = 0; i < MAX_NUM_MSGS_STORED; i++) {
		if (self->storage_queue->msg_queue[i].valid) {
			msg_wave_height = self->storage_queue->msg_queue[i].payload.Hs;
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
	// Zero out the whole thing
	for (int i = 0; i < MAX_NUM_MSGS_STORED; i++) {
		self->storage_queue->msg_queue[i].valid = false;
	}
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
	uint8_t payload_index = 0;
	return_code = self->queue_get(self, &payload_index);
	if (return_code == IRIDIUM_STORAGE_QUEUE_EMPTY) {
		return return_code;
	}
	// try transmitting the message
	return_code = internal_transmit_message(self,
			(uint8_t*)&(self->storage_queue->msg_queue[payload_index].payload),
			sizeof(sbd_message_type_52) - IRIDIUM_CHECKSUM_LENGTH);
	// If the message successfully transmitted, mark it as invalid
	if (return_code == IRIDIUM_SUCCESS) {
		self->storage_queue->msg_queue[payload_index].valid = false;
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
	payload[payload_size + 1] = ((uint8_t)*checksum_ptr);
	checksum_ptr++;
	payload[payload_size] = ((uint8_t)*checksum_ptr);
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
	iridium_error_code_t queue_return_code __attribute__((unused));
	int fail_counter;
	bool message_tx_success = false;
	bool all_messages_sent = false;
	uint32_t timer_minutes = 0;

	// Make sure we can get an acknowledgment from the modem
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE, ONE_SECOND)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
		} else {
			break;
		}
	}
	// If we were unable to get an ack from the modem, return IRIDIUM_UART_ERROR
	if (fail_counter == MAX_RETRIES) {
		return IRIDIUM_UART_ERROR;
	}

	// We'll add extra time to the transmit window if there are a bunch of messages in the queue
	if (self->storage_queue->num_msgs_enqueued >= 5 && self->storage_queue->num_msgs_enqueued < 10) {
		timer_minutes = self->global_config->iridium_max_transmit_time + 5;
	}
	else if (self->storage_queue->num_msgs_enqueued >= 10) {
		timer_minutes = self->global_config->iridium_max_transmit_time + 10;
	} else {
		timer_minutes = self->global_config->iridium_max_transmit_time;
	}

	// reset the timer and clear the interrupt flag
	self->timer_timeout = false;
	self->reset_timer(self, timer_minutes);
	// Start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(self->timer);
	// Send the message that was just generated
	while (!self->timer_timeout && !message_tx_success) {
		return_code = internal_transmit_message(self, (uint8_t*)self->current_message,
				sizeof(sbd_message_type_52) - IRIDIUM_CHECKSUM_LENGTH);

		if (return_code == IRIDIUM_UART_ERROR) {
			cycle_power(self);
		}
		message_tx_success = return_code == IRIDIUM_SUCCESS;
	}

	// If we made it here, there's still time left, try sending a queued message
	// First, make sure we actually have messages in the queue
	all_messages_sent = self->storage_queue->num_msgs_enqueued == 0;
	// If we have time, send messages from the queue
	while (!self->timer_timeout && !all_messages_sent) {
		queue_return_code = send_msg_from_queue(self);
		all_messages_sent = self->storage_queue->num_msgs_enqueued == 0;
	}

	// Message failed to send. If there is space in the queue, store it,
	// otherwise return IRIDIUM_STORAGE_QUEUE_FULL
	if (self->timer_timeout && !message_tx_success) {
		// reset the timer and clear the flag for the next time
		HAL_TIM_Base_Stop_IT(self->timer);
		self->timer_timeout = false;
		__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);

		return self->queue_add(self, self->current_message);
	}



	// reset the timer and clear the flag for the next time
	HAL_TIM_Base_Stop_IT(self->timer);
	self->timer_timeout = false;
	__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);
	return return_code;
}

/**
 ********************************************************************************************
 * TODO: When we get to the point of implimenting two-way comms, this will need significant
 *       overhaul
 ********************************************************************************************
 * Helper method to transmit a message via Iridium modem.
 *
 * @param self - pointer to Iridium struct
 * @param payload - pointer to a SBD message payload
 * @return IRIDIUM_SUCCESS to indicate message was sent
 * 		   IRIDIUM_TRANSMIT_ERROR if it didn't send
 * 		   IRIDIUM_UART_ERROR if something went wrong trying to talk to the modem
 */
static iridium_error_code_t internal_transmit_message(Iridium* self,
		uint8_t* payload, uint16_t payload_size)
{
	iridium_error_code_t return_code = IRIDIUM_TRANSMIT_TIMEOUT;
	char* needle;
	char payload_size_str[4];
	char load_sbd[15] = "AT+SBDWB=";
	char SBDWB_response_code;
	int SBDI_response_code;
	int fail_counter;
//	int transmit_fail_counter = 0;
//	int adaptive_delay_index;
//	uint32_t adaptive_delay_time[5] = {ONE_SECOND * 3, ONE_SECOND * 5,
//			ONE_SECOND * 28, ONE_SECOND * 58, ONE_SECOND * 178};
//	int delay_time;
//	bool sleep_break;
	bool checksum_match;

	// Assemble the load_sbd string
	itoa(payload_size, payload_size_str, 10);
	strcat(load_sbd, payload_size_str);
	load_sbd[12] = '\r';

	while (!self->timer_timeout) {
		// get the checksum
		get_checksum((uint8_t*)payload, payload_size);
		// reset flags
//		sleep_break = false;
		checksum_match = true;

		// Tell the modem we want to send a message
		for (fail_counter = 0; fail_counter < MAX_RETRIES && !self->timer_timeout; fail_counter++) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(load_sbd[0]),
					strlen(load_sbd), ONE_SECOND);

			HAL_UART_Receive(self->iridium_uart_handle,
					&(self->response_buffer[0]), SBDWB_READY_RESPONSE_SIZE, ONE_SECOND);

			needle = strstr((char*)&(self->response_buffer[0]), "READY");
			// Success case
			if (needle != NULL) {
				memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
				self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
				break;
			}
			// Clear the response buffer and reset UART for the next step
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		}

		if (fail_counter == MAX_RETRIES) {
			return_code = IRIDIUM_UART_ERROR;
			continue;
		}

		// Send over the payload + checksum
		while (!self->timer_timeout) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(payload[0]),
					payload_size + IRIDIUM_CHECKSUM_LENGTH, ONE_SECOND * 2);

			HAL_UART_Receive(self->iridium_uart_handle,
					&(self->response_buffer[0]), SBDWB_LOAD_RESPONSE_SIZE, ONE_SECOND);

			SBDWB_response_code = self->response_buffer[SBDWB_RESPONSE_CODE_INDEX];
			// Success case
			if (SBDWB_response_code == '0') {
				memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
				self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
				checksum_match = true;
				break;
			}

			// Response of 2 means checksum didn't match, loop around and try again
			if (SBDWB_response_code == '2') {
				checksum_match = false;
				break;
			}
			// Clear the response buffer and reset UART for the next step
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		}

		if ((fail_counter == MAX_RETRIES) || (!checksum_match)) {
			return_code = IRIDIUM_UART_ERROR;
			continue;
		}

		// Tell the modem to send the message
		while(!self->timer_timeout) {
			HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(send_sbd[0]),
					strlen(send_sbd), ONE_SECOND);
			// We will only grab the response up to and including MO status
			HAL_UART_Receive(self->iridium_uart_handle,
					&(self->response_buffer[0]), SBDI_RESPONSE_SIZE, ONE_SECOND * 35);
			// Grab the MO status
			SBDI_response_code = atoi((char*)&(self->response_buffer[SBDI_RESPONSE_CODE_INDEX]));

			HAL_Delay(ONE_SECOND * 15);

			if (SBDI_response_code == 1) {
				if (send_basic_command_message(self, clear_MO, SBDD_RESPONSE_SIZE, ONE_SECOND * 10) ==
											IRIDIUM_COMMAND_RESPONSE_ERROR) {
					cycle_power(self);
				}
				return IRIDIUM_SUCCESS;
			}

			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			break;
		}
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
	uint16_t error_msg_str_length = strlen(error_message);
	uint16_t payload_iterator = 0;
//	char error_message_payload[IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH];
	float timestamp;
	float* timestamp_ptr = &timestamp;
	float* lat_ptr = &self->current_lat;
	float* lon_ptr = &self->current_lon;
	bool message_tx_success = false;

	// If the error message is too long, we'll just cut it off
	if (error_msg_str_length > ERROR_MESSAGE_MAX_LENGTH - 1) {
		error_message[ERROR_MESSAGE_MAX_LENGTH - 1] = 0;
		error_msg_str_length = ERROR_MESSAGE_MAX_LENGTH - 1;
	}

	// Assemble the error message payload, start by clearing the whole thing out
	memset(&(self->error_message_buffer[0]), 0, IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH);
	// First byte is message type (99)
	self->error_message_buffer[payload_iterator] = ERROR_MESSAGE_TYPE;
	payload_iterator++;
	memcpy(&(self->error_message_buffer[payload_iterator]), error_message, error_msg_str_length);
	// Set the iterator to the index after the string
	payload_iterator += ERROR_MESSAGE_MAX_LENGTH;
	memcpy(&(self->error_message_buffer[payload_iterator]), lat_ptr,
			sizeof(self->current_lat));
	payload_iterator += sizeof(self->current_lat);
	memcpy(&(self->error_message_buffer[payload_iterator]), lon_ptr,
				sizeof(self->current_lon));
	payload_iterator += sizeof(self->current_lon);
	timestamp = self->get_timestamp(self);
	memcpy(&(self->error_message_buffer[payload_iterator]), timestamp_ptr,
			sizeof(float));

	// reset the timer and clear the interrupt flag
	self->reset_timer(self, self->global_config->iridium_max_transmit_time);
	// Start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(self->timer);
	// Send the message that was just generated
	while (!self->timer_timeout && !message_tx_success) {
		return_code = internal_transmit_message(self, (uint8_t*)&(self->error_message_buffer[0]),
				IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE);
		if (return_code == IRIDIUM_UART_ERROR) {
			cycle_power(self);
		}
		message_tx_success = return_code == IRIDIUM_SUCCESS;
	}

	// Message failed to send.
	if (self->timer_timeout && !message_tx_success) {
			// reset the timer and clear the flag for the next time
			HAL_TIM_Base_Stop_IT(self->timer);
			self->timer_timeout = false;
			__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);
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
	self->sleep(self, GPIO_PIN_RESET);
	HAL_Delay(2100);
	self->sleep(self, GPIO_PIN_SET);
	HAL_Delay(2100);
}

/**
 * Helper method to generate a timestamp from the RTC.
 *
 * @param self - Iridium struct
 * @return timestamp as a float
 */
float iridium_get_timestamp(Iridium* self)
{
	uint32_t timestamp = 0;
	bool is_leap_year = false;
	uint8_t num_leap_years_since_2000 = 0;
	uint16_t julian_date_first_of_month = 0;
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	// Get the date and time
	HAL_RTC_GetTime(self->rtc_handle, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(self->rtc_handle, &rtc_date, RTC_FORMAT_BIN);

	// Let's make a timestamp (yay...)
	// Years first
	timestamp += SECONDS_1970_TO_2000;
	timestamp += rtc_date.Year * SECONDS_IN_YEAR;
	num_leap_years_since_2000 = rtc_date.Year / 4;
	timestamp += num_leap_years_since_2000 * SECONDS_IN_DAY;

	// Years are only represented with 2 digits. We'll set 0 as the year 2000, so anything
	// evenly divisible by 4 is a leap year (2000, 2004, 2008, etc)
	is_leap_year = rtc_date.Year % 4 == 0;

	switch (rtc_date.Month) {
		case RTC_MONTH_JANUARY:
			// No months to account for!!!
			break;

		case RTC_MONTH_FEBRUARY:
			julian_date_first_of_month = 32;
			break;

		case RTC_MONTH_MARCH:
			julian_date_first_of_month = (is_leap_year) ? 61 : 60;
			break;

		case RTC_MONTH_APRIL:
			julian_date_first_of_month = (is_leap_year) ? 92 : 91;
			break;

		case RTC_MONTH_MAY:
			julian_date_first_of_month = (is_leap_year) ? 122 : 121;
			break;

		case RTC_MONTH_JUNE:
			julian_date_first_of_month = (is_leap_year) ? 153 : 152;
			break;

		case RTC_MONTH_JULY:
			julian_date_first_of_month = (is_leap_year) ? 183 : 182;
			break;

		case RTC_MONTH_AUGUST:
			julian_date_first_of_month = (is_leap_year) ? 214 : 213;
			break;

		case RTC_MONTH_SEPTEMBER:
			julian_date_first_of_month = (is_leap_year) ? 245 : 244;
			break;

		case RTC_MONTH_OCTOBER:
			julian_date_first_of_month = (is_leap_year) ? 275 : 274;
			break;

		case RTC_MONTH_NOVEMBER:
			julian_date_first_of_month = (is_leap_year) ? 306 : 305;
			break;

		case RTC_MONTH_DECEMBER:
			julian_date_first_of_month = (is_leap_year) ? 336 : 335;
			break;

		default:
			break;
	}
	timestamp += (julian_date_first_of_month) * SECONDS_IN_DAY;
	timestamp += (rtc_date.Date - 1) * SECONDS_IN_DAY;
	timestamp += rtc_time.Hours * SECONDS_IN_HOUR;
	timestamp += rtc_time.Minutes * SECONDS_IN_MIN;
	timestamp += rtc_time.Seconds;
	// Not including fractions of a second
	return (float)timestamp;
}

