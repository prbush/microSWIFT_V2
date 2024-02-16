/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include "Peripherals/iridium.h"
// Object pointer
static Iridium* self;
// Object functions
static iridium_error_code_t iridium_config(void);
static void 								iridium_charge_caps(uint32_t caps_charge_time_ticks);
static iridium_error_code_t iridium_self_test(void);
static void 								iridium_sleep(GPIO_PinState pin_state);
static void 								iridium_on_off(GPIO_PinState pin_state);
static iridium_error_code_t iridium_reset_iridium_uart(uint16_t baud_rate);
static iridium_error_code_t iridium_reset_timer(uint8_t timeout_in_minutes);
static iridium_error_code_t iridium_storage_queue_add(sbd_message_type_52* payload);
static iridium_error_code_t iridium_storage_queue_get(uint8_t* msg_index);
static void 								iridium_storage_queue_flush(void);
static iridium_error_code_t iridium_transmit_message(void);
static iridium_error_code_t iridium_transmit_error_message(char* error_message);
static void 								iridium_cycle_power(void);
static uint32_t 						iridium_get_timestamp(void);


// Helper functions
static iridium_error_code_t iridium_store_in_flash(void);
static void 								get_checksum(uint8_t* payload, size_t payload_size);
static iridium_error_code_t send_basic_command_message(const char* command,
															uint8_t response_size, uint32_t wait_time);
static iridium_error_code_t send_msg_from_queue(void);
static iridium_error_code_t internal_transmit_message(uint8_t* payload,
															uint16_t payload_size);
static void 								reset_struct_fields(void);

// static variables
static const char* ack = "AT\r";
static const char* disable_flow_control = "AT&K0\r";
static const char* enable_ring_indications = "AT+SBDMTA=1\r";
static const char* store_config = "AT&W0\r";
static const char* select_power_up_profile = "AT&Y0\r";
static const char* clear_MO = "AT+SBDD0\r";
static const char* send_sbd = "AT+SBDIX\r";

/**
 * Initialize the CT struct
 *
 * @return void
 */
void iridium_init(Iridium* struct_ptr, microSWIFT_configuration* global_config,
		UART_HandleTypeDef* iridium_uart_handle, DMA_HandleTypeDef* iridium_rx_dma_handle,
		TIM_HandleTypeDef* timer, DMA_HandleTypeDef* iridium_tx_dma_handle,
		TX_EVENT_FLAGS_GROUP* control_flags, TX_EVENT_FLAGS_GROUP* error_flags,
		RTC_HandleTypeDef* rtc_handle, sbd_message_type_52* current_message,
		uint8_t* error_message_buffer, uint8_t* response_buffer,
		Iridium_message_storage* storage_queue)
{
	// Assign the object pointer
	self = struct_ptr;

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

	reset_struct_fields();

	self->config = iridium_config;
	self->charge_caps = iridium_charge_caps;
	self->self_test = iridium_self_test;
	self->transmit_message = iridium_transmit_message;
	self->transmit_error_message = iridium_transmit_error_message;
	self->get_timestamp = iridium_get_timestamp;
	self->sleep = iridium_sleep;
	self->on_off = iridium_on_off;
	self->cycle_power = iridium_cycle_power;
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
static iridium_error_code_t iridium_config(void)
{
	iridium_error_code_t return_code;

	// Get an ack message
	return_code = send_basic_command_message(ack, ACK_MESSAGE_SIZE, TX_TIMER_TICKS_PER_SECOND);
	if (return_code != IRIDIUM_SUCCESS) {
		return return_code;
	}
	// disable flow control
	return_code = send_basic_command_message(disable_flow_control, DISABLE_FLOW_CTRL_SIZE,
			TX_TIMER_TICKS_PER_SECOND);
	if (return_code != IRIDIUM_SUCCESS) {
		return return_code;
	}
	// enable SBD ring indications
	return_code = send_basic_command_message(enable_ring_indications, ENABLE_RI_SIZE,
			TX_TIMER_TICKS_PER_SECOND);
	if (return_code != IRIDIUM_SUCCESS) {
		return return_code;
	}
	// Store this configuration as profile 0
	return_code = send_basic_command_message(store_config, STORE_CONFIG_SIZE, TX_TIMER_TICKS_PER_SECOND);
	if (return_code != IRIDIUM_SUCCESS) {
		return return_code;
	}
	// set profile 0 as the power-up profile
	return_code = send_basic_command_message(select_power_up_profile, SELECT_PWR_UP_SIZE,
			TX_TIMER_TICKS_PER_SECOND);
	if (return_code != IRIDIUM_SUCCESS) {
		return return_code;
	}

	self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);

	return return_code;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static void iridium_charge_caps(uint32_t caps_charge_time_ticks)
{
	// Power the unit by pulling the sleep pin to ground.
	self->on_off(GPIO_PIN_SET);
	self->sleep(GPIO_PIN_SET);

	tx_thread_sleep(caps_charge_time_ticks);
}

/**
 *
 *
 * @return iridium_error_code_t
 */
static iridium_error_code_t iridium_self_test(void)
{
	return send_basic_command_message(ack, ACK_MESSAGE_SIZE, TX_TIMER_TICKS_PER_SECOND);
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
static void iridium_sleep(GPIO_PinState pin_state)
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
static void iridium_on_off(GPIO_PinState pin_state)
{
	if (pin_state == GPIO_PIN_SET) {
		HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, pin_state);
		// Wait 10ms between powering 5V bus FET and the Iridium power FET
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
		HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, pin_state);
	} else {
		HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, pin_state);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
		HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, pin_state);
	}
}

/**
 * Store an unsent message in flash memory.
 * !!!
 * This function has not been implimented yet!
 * !!!
 *
 * @param self - Iridium struct
 *
 * @return iridium_error_code_t
 */
static iridium_error_code_t iridium_store_in_flash(void)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;

	HAL_FLASH_Unlock();

	// TODO: This will get implimented later to take the contents of the
	//       storage queue and stuff it in flash at some high page(s)

	HAL_FLASH_Lock();
	return return_code;
}

/**
 * Reset the Iridium UART.
 *
 * @param self - Iridium struct
 *
 * @return IRIDIUM_SUCCESS on success
 * 		   IRIDIUM_UART_ERROR when an error is encountered
 */
static iridium_error_code_t iridium_reset_iridium_uart(uint16_t baud_rate)
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

	__HAL_DMA_DISABLE_IT(self->iridium_rx_dma_handle, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(self->iridium_tx_dma_handle, DMA_IT_HT);

	return IRIDIUM_SUCCESS;
}

/**
 * Reset the Iridium timer. Will generate an interrupt after timeout_in_minutes.
 *
 * @param self- Iridium struct
 * @param timeout_in_minutes - number of minutes to set the timeout to
 *
 * @return IRIDIUM_SUCCESS or
 * 		   IRIDIUM_TIMER_ERROR on error when trying to initialize timer
 */
static iridium_error_code_t iridium_reset_timer(uint8_t timeout_in_minutes)
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
 * Addd a message to the Iridium storage queue
 *
 * @param self- Iridium struct
 * @param payload - message to store
 *
 * @return 	IRIDIUM_SUCCESS or
 * 			IRIDIUM_STORAGE_QUEUE_FULL if there is no space remaining in the queue
 */
static iridium_error_code_t iridium_storage_queue_add(sbd_message_type_52* payload)
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
 *
 * @return 	IRIDIUM_SUCCESS or
 * 			IRIDIUM_STORAGE_QUEUE_EMPTY if the queue is empty
 */
static iridium_error_code_t iridium_storage_queue_get(uint8_t* msg_index)
{
	float significant_wave_height = 0.0;
	float msg_wave_float = 0.0;
	real16_T msg_wave_height;
	msg_wave_height.bitPattern = 0;

	// Queue corruption check
	if (self->storage_queue->magic_number != SBD_QUEUE_MAGIC_NUMBER) {
		self->queue_flush();
		return IRIDIUM_STORAGE_QUEUE_EMPTY;
	}
	// Empty queue check
	if (self->storage_queue->num_msgs_enqueued == 0) {
		return IRIDIUM_STORAGE_QUEUE_EMPTY;
	}

	// Find the largest significant wave height
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
static void iridium_storage_queue_flush(void)
{
	if (self->storage_queue->magic_number != SBD_QUEUE_MAGIC_NUMBER) {
		// Zero out the whole thing
		for (int i = 0; i < MAX_NUM_MSGS_STORED; i++) {
			self->storage_queue->msg_queue[i].valid = false;
			memset(&(self->storage_queue->msg_queue[i].payload), 0, sizeof(sbd_message_type_52));
		}
		self->storage_queue->num_msgs_enqueued = 0;
		// Set the magic number to indicate the queue has been initialized
		self->storage_queue->magic_number = SBD_QUEUE_MAGIC_NUMBER;
	}
}

/**
 * Static helper function to send message from the queue.
 *
 * @param self- Iridium struct
 * @return iridium_error_code_t
 */
static iridium_error_code_t send_msg_from_queue(void)
{
	iridium_error_code_t return_code;
	uint8_t payload_index = 0;
	return_code = self->queue_get(&payload_index);
	if (return_code == IRIDIUM_STORAGE_QUEUE_EMPTY) {
		return return_code;
	}
	// try transmitting the message
	return_code = internal_transmit_message(
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
 * Static helper function to get the checksum of an Iridium message.
 *
 * @param self- Iridium struct
 * @param payload_size - size in bytes of the payload for which the
 * 						 checksum is being calculates
 *
 * @return void
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
 * Static helper function to send a basic command message to the modem
 *
 * @param self- Iridium struct
 * @param command - command string
 * @param response_size - size of the modem response
 * @param wait_time_ticks - maximum length of time the modem could take to respond
 * 							(in scheduler ticks)
 *
 * @return	IRIDIUM_SUCCESS or
 * 			IRIDIUM_UART_ERROR
 */
static iridium_error_code_t send_basic_command_message(const char* command,
		uint8_t response_size, uint32_t wait_time_ticks)
{
	char * needle;
	ULONG actual_flags;

	self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

	HAL_UART_Transmit_DMA(self->iridium_uart_handle, (uint8_t*)&(command[0]),
			strlen(command));

	if (tx_event_flags_get(self->control_flags, IRIDIUM_TX_COMPLETE, TX_OR_CLEAR, &actual_flags,
			1) != TX_SUCCESS) {
		return IRIDIUM_UART_ERROR;
	}

	HAL_UART_Receive_DMA(self->iridium_uart_handle, &(self->response_buffer[0]),
			response_size);

	if (tx_event_flags_get(self->control_flags, IRIDIUM_MSG_RECVD, TX_OR_CLEAR, &actual_flags,
			wait_time_ticks) != TX_SUCCESS) {
		return IRIDIUM_UART_ERROR;
	}

	needle = strstr((char*)&(self->response_buffer[0]), "OK");
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	return (needle == NULL) ? IRIDIUM_COMMAND_RESPONSE_ERROR : IRIDIUM_SUCCESS;
}

/**
 * Transmit a message from the modem. First, the current window's message will be sent,
 * and if time is remaining, messages from the queue will be sent after.
 *
 * @param self- Iridium struct
 *
 * @return	IRIDIUM_SUCCESS or
 */
static iridium_error_code_t iridium_transmit_message(void)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	iridium_error_code_t queue_return_code __attribute__((unused));
	bool message_tx_success = false;
	bool all_messages_sent = false;
	uint32_t timer_minutes = self->global_config->iridium_max_transmit_time;

	register_watchdog_refresh();

	// reset the timer and clear the interrupt flag
	self->reset_timer(timer_minutes);
	// Start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(self->timer);

	if (self->skip_current_message) {

		register_watchdog_refresh();

		all_messages_sent = false;

		while (!self->timer_timeout && !all_messages_sent) {
			queue_return_code = send_msg_from_queue();
			all_messages_sent = (queue_return_code == IRIDIUM_STORAGE_QUEUE_EMPTY);
		}

		HAL_TIM_Base_Stop_IT(self->timer);
		__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);

		return queue_return_code;

	} else {

		register_watchdog_refresh();

		// Send the message that was just generated
		while (!self->timer_timeout && !message_tx_success) {
			register_watchdog_refresh();
			return_code = internal_transmit_message((uint8_t*)self->current_message,
					sizeof(sbd_message_type_52) - IRIDIUM_CHECKSUM_LENGTH);

			if (return_code == IRIDIUM_UART_ERROR) {
				self->cycle_power();
				self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
			}
			message_tx_success = return_code == IRIDIUM_SUCCESS;
		}

		self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);

		// If we made it here, there's may still be time left, try sending a queued message
		// First, make sure we actually have messages in the queue
		all_messages_sent = self->storage_queue->num_msgs_enqueued == 0;
		// If we have time, send messages from the queue
		while (!self->timer_timeout && !all_messages_sent) {
			register_watchdog_refresh();
			queue_return_code = send_msg_from_queue();

			if (queue_return_code == IRIDIUM_UART_ERROR) {
				self->cycle_power();
				self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
			}
			all_messages_sent = self->storage_queue->num_msgs_enqueued == 0;
		}

		// Message failed to send. If there is space in the queue, store it,
		// otherwise return IRIDIUM_STORAGE_QUEUE_FULL
		if (self->timer_timeout && !message_tx_success) {
			// reset the timer and clear the flag for the next time
			HAL_TIM_Base_Stop_IT(self->timer);
			__HAL_TIM_CLEAR_FLAG(self->timer, TIM_FLAG_UPDATE);

			return self->queue_add(self->current_message);
		}
	}

	// reset the timer and clear the flag for the next time
	HAL_TIM_Base_Stop_IT(self->timer);
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
 *
 * @return IRIDIUM_SUCCESS to indicate message was sent
 * 		   IRIDIUM_TRANSMIT_ERROR if it didn't send
 * 		   IRIDIUM_UART_ERROR if something went wrong trying to talk to the modem
 */
static iridium_error_code_t internal_transmit_message(uint8_t* payload,
		uint16_t payload_size)
{
	iridium_error_code_t return_code = IRIDIUM_TRANSMIT_TIMEOUT;
	ULONG actual_flags;
	char* needle;
	char* sbdix_search_term = "+SBDIX: ";
	char payload_size_str[4];
	char load_sbd[15] = "AT+SBDWB=";
	char SBDWB_response_code;
	int SBDIX_response_code;
	int fail_counter;
	int tx_response_time;
	bool checksum_match;
	bool message_response_received;

	// Assemble the load_sbd string
	itoa(payload_size, payload_size_str, 10);
	strcat(load_sbd, payload_size_str);
	strcat(load_sbd, "\r");

	while (!self->timer_timeout) {
		register_watchdog_refresh();

		message_response_received = false;
		tx_response_time = 0;

		// get the checksum
		get_checksum((uint8_t*)payload, payload_size);

		// Tell the modem we want to send a message
		for (fail_counter = 0; fail_counter < MAX_RETRIES && !self->timer_timeout; fail_counter++) {
			register_watchdog_refresh();

			HAL_UART_Transmit_DMA(self->iridium_uart_handle, (uint8_t*)&(load_sbd[0]),
					strlen(load_sbd));
			if (tx_event_flags_get(self->control_flags, IRIDIUM_TX_COMPLETE, TX_OR_CLEAR, &actual_flags,
					1) != TX_SUCCESS) {
				return IRIDIUM_UART_ERROR;
			}

			HAL_UART_Receive_DMA(self->iridium_uart_handle,
					&(self->response_buffer[0]), SBDWB_READY_RESPONSE_SIZE);
			if (tx_event_flags_get(self->control_flags, IRIDIUM_MSG_RECVD, TX_OR_CLEAR, &actual_flags,
						TX_TIMER_TICKS_PER_SECOND) != TX_SUCCESS) {
				return IRIDIUM_UART_ERROR;
			}

			needle = strstr((char*)&(self->response_buffer[0]), "READY");
			// Success case
			if (needle != NULL) {
				memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
				self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
				break;
			}
			// Clear the response buffer and reset UART for the next step
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
		}

		if (fail_counter == MAX_RETRIES) {
			return_code = IRIDIUM_UART_ERROR;
			continue;
		}

		// Send over the payload + checksum
		for (fail_counter = 0; fail_counter < MAX_RETRIES && !self->timer_timeout; fail_counter++) {
			register_watchdog_refresh();
			HAL_UART_Transmit_DMA(self->iridium_uart_handle, (uint8_t*)&(payload[0]),
					payload_size + IRIDIUM_CHECKSUM_LENGTH);
			if (tx_event_flags_get(self->control_flags, IRIDIUM_TX_COMPLETE, TX_OR_CLEAR, &actual_flags,
					2) != TX_SUCCESS) {
				return IRIDIUM_UART_ERROR;
			}

			HAL_UART_Receive_DMA(self->iridium_uart_handle,
					&(self->response_buffer[0]), SBDWB_LOAD_RESPONSE_SIZE);
			if (tx_event_flags_get(self->control_flags, IRIDIUM_MSG_RECVD, TX_OR_CLEAR, &actual_flags,
					TX_TIMER_TICKS_PER_SECOND) != TX_SUCCESS) {
				return IRIDIUM_UART_ERROR;
			}

			SBDWB_response_code = self->response_buffer[SBDWB_RESPONSE_CODE_INDEX];
			// Success case
			if (SBDWB_response_code == '0') {
				memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
				self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
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
			self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
		}

		if ((fail_counter == MAX_RETRIES) || (!checksum_match)) {
			return_code = IRIDIUM_UART_ERROR;
			continue;
		}

		// Tell the modem to send the message
		register_watchdog_refresh();
		HAL_UART_Transmit_DMA(self->iridium_uart_handle, (uint8_t*)&(send_sbd[0]),
				strlen(send_sbd));
		if (tx_event_flags_get(self->control_flags, IRIDIUM_TX_COMPLETE, TX_OR_CLEAR, &actual_flags,
				1) != TX_SUCCESS) {
			return IRIDIUM_UART_ERROR;
		}
		register_watchdog_refresh();
		// We will only grab the response up to and including MO status
		HAL_UART_Receive_DMA(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDIX_RESPONSE_SIZE);
		// Since it takes longer than the maximum watchdog refresh interval, we'll check once a second to see
		// if we have received the response from the modem, refreshing the watchdog along the way
		while (!message_response_received && (tx_response_time < 45)) {
			message_response_received = (tx_event_flags_get(self->control_flags, IRIDIUM_MSG_RECVD, TX_OR_CLEAR,
					&actual_flags, TX_TIMER_TICKS_PER_SECOND) == TX_SUCCESS);
			register_watchdog_refresh();
			tx_response_time++;
		}

		register_watchdog_refresh();
		// Grab the MO status
		needle = strstr((char*)&(self->response_buffer[0]), sbdix_search_term);
		needle += strlen(sbdix_search_term);
		SBDIX_response_code = atoi(needle);

		if (SBDIX_response_code <= 4) {
			// Success case
			send_basic_command_message(clear_MO, SBDD_RESPONSE_SIZE, TX_TIMER_TICKS_PER_SECOND * 10);
			register_watchdog_refresh();
			return IRIDIUM_SUCCESS;
		}

		// If message Tx failed, put the modem to sleep and delay for a total of 30 seconds
		self->sleep(GPIO_PIN_RESET);
		register_watchdog_refresh();
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 25);
		register_watchdog_refresh();
		self->sleep(GPIO_PIN_SET);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 5);
		register_watchdog_refresh();

		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
		self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
		return_code = IRIDIUM_TRANSMIT_UNSUCCESSFUL;
	}

	register_watchdog_refresh();
	return return_code;
}

/**
 * Helper method to get the current signal strength.
 *
 * @param self - Iridium struct
 * @param error_message - null terminated error message of 320 bytes or less. Note that this
 * 						  string is mutable to handle the case of terminating a too-long string
 *
 * @return IRIDIUM_SUCCESS to indicate message was sent
 * 		   IRIDIUM_TRANSMIT_ERROR if it didn't send
 * 		   IRIDIUM_UART_ERROR if something went wrong trying to talk to the modem
 */
static iridium_error_code_t iridium_transmit_error_message(char* error_message)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	uint16_t error_msg_str_length = strlen(error_message);
	uint16_t payload_iterator = 0;
	float timestamp;
	float* timestamp_ptr = &timestamp;
	float* lat_ptr = &self->current_lat;
	float* lon_ptr = &self->current_lon;
	bool message_tx_success = false;

	register_watchdog_refresh();
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
	timestamp = self->get_timestamp();
	memcpy(&(self->error_message_buffer[payload_iterator]), timestamp_ptr,
			sizeof(float));

	// reset the timer and clear the interrupt flag
	self->reset_timer(self->global_config->iridium_max_transmit_time);
	// Start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(self->timer);
	// Send the message that was just generated
	while (!self->timer_timeout && !message_tx_success) {
		return_code = internal_transmit_message((uint8_t*)&(self->error_message_buffer[0]),
				IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE);
		if (return_code == IRIDIUM_UART_ERROR) {
			self->cycle_power();
			self->reset_uart(IRIDIUM_DEFAULT_BAUD_RATE);
		}
		message_tx_success = return_code == IRIDIUM_SUCCESS;
	}

	// Message failed to send.
	if (self->timer_timeout && !message_tx_success) {
			// reset the timer and clear the flag for the next time
			HAL_TIM_Base_Stop_IT(self->timer);
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
static void iridium_cycle_power(void)
{
	self->sleep(GPIO_PIN_RESET);
	tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND * 2) + 1);
	self->sleep(GPIO_PIN_SET);
	tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND * 2) + 1);
}

/**
 * Helper method to generate a timestamp from the RTC.
 *
 * @param self - Iridium struct
 * @return timestamp as a float
 */
static uint32_t iridium_get_timestamp(void)
{
	time_t timestamp = 0;
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;
	struct tm time= {0};

	// Get the date and time
	HAL_RTC_GetTime(self->rtc_handle, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(self->rtc_handle, &rtc_date, RTC_FORMAT_BIN);

	time.tm_sec 	= rtc_time.Seconds;
	time.tm_min 	= rtc_time.Minutes;
	time.tm_hour 	= rtc_time.Hours;
	time.tm_mday 	= rtc_date.Date;
	time.tm_mon 	= rtc_date.Month - 1;
	time.tm_year 	= (rtc_date.Year + 2000) - 1900;

	timestamp = mktime(&time);

	return (uint32_t)timestamp;
}

/**
 *
 *
 * @return void
 */
static void reset_struct_fields(void)
{
	self->current_lat = 0.0;
	self->current_lon = 0.0;
	self->timer_timeout = false;
	self->skip_current_message = false;
}
