/*
 * Iridium.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#include <iridium.h>

// Static functions
static uint16_t get_checksum(uint8_t* payload, size_t payload_size);
static iridium_error_code_t send_basic_command_message(Iridium* self, const char* command, uint8_t response_size);
static iridium_error_code_t send_msg_from_queue(Iridium* self);
static iridium_error_code_t transmit_message(Iridium* self, uint8_t* payload);
static uint8_t get_signal_strength(Iridium* self);
static void cycle_power(Iridium* self);
// ISR callback functions
static void transmit_timeout_callback(TIM_HandleTypeDef *htim);
//static void uart_receive_dma_callback(UART_HandleTypeDef *huart);

//// static variables
static bool timer_timeout = false;
//static bool dma_message_received = false;
// const strings for Iridium AT commands
static const char* ack = "AT\r";
static const char* disable_flow_control = "AT&K0\r";
static const char* enable_ring_indications = "AT+SBDMTA=1\r";
static const char* store_config = "AT&W0\r";
static const char* select_power_up_profile = "AT&Y0\r";
static const char* clear_MO = "AT+SBDD0\r";
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
//	self->transmit_timeout = timer_timeout;
//	self->dma_message_received = dma_message_received;
	self->transmit_timeout = false;
	self->dma_message_received = false;
	self->current_lat = 0;
	self->current_long = 0;
	self->config = iridium_config;
	self->self_test = iridium_self_test;
	self->transmit_message = iridium_transmit_message;
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
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE) == IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			continue;
		}
		// disable flow control
		if (send_basic_command_message(self, disable_flow_control, DISABLE_FLOW_CTRL_SIZE)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			continue;
		}
		// enable SBD ring indications
		if (send_basic_command_message(self, enable_ring_indications, ENABLE_RI_SIZE)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			continue;
		}
		// Store this configuration as profile 0
		if (send_basic_command_message(self, store_config, STORE_CONFIG_SIZE)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
			continue;
		}
		// set profile 0 as the power-up profile
		if (send_basic_command_message(self, select_power_up_profile, SELECT_PWR_UP_SIZE)
				== IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
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
	// Zero out the buffer
//	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);

	// Power the unit by pulling the sleep pin to ground.
	self->on_off(self, true);

	// Start the timer
	HAL_TIM_Base_Start(self->ten_min_timer);
	// Wait an appropriate amount of time for the caps to charge
	while (elapsed_time < IRIDIUM_CAP_CHARGE_TIME) {
		HAL_Delay(1000);
		elapsed_time = __HAL_TIM_GET_COUNTER(self->ten_min_timer);
	}
	HAL_TIM_Base_Stop(self->ten_min_timer);

	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		// Get an ack message
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE) == IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		} else {
			break;
		}
	}

//	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	return (fail_counter == MAX_RETRIES) ? IRIDIUM_UART_ERROR : IRIDIUM_SUCCESS;
}

/**
 *
 *
 * @return iridium_error_code_t
 */
iridium_error_code_t iridium_transmit_message(Iridium* self)
{
	iridium_error_code_t return_code = IRIDIUM_SUCCESS;
	int fail_counter;
	bool message_tx_success = false;
	bool all_messages_sent = false;

	// Make sure we can get an acknowledgment from the modem
	for (fail_counter = 0; fail_counter < MAX_RETRIES; fail_counter++) {
		if (send_basic_command_message(self, ack, ACK_MESSAGE_SIZE) == IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
//			self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		} else {
			break;
		}
	}
	if (fail_counter == MAX_RETRIES) {return IRIDIUM_UART_ERROR;}
	// get the most recent location
	self->get_location(self);

	// Start the timer
	self->transmit_timeout = false;
	self->reset_timer(self);

	HAL_TIM_Base_Start(self->ten_min_timer);

	// Send the message that was just generated
	while (!self->transmit_timeout && !message_tx_success) {
		return_code = transmit_message(self, self->message_buffer);
		message_tx_success = return_code == IRIDIUM_SUCCESS ? true : false;
	}

	// If we have time, send messages from the queue
	while (!self->transmit_timeout && !all_messages_sent) {

	}
	self->transmit_timeout = false;

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
iridium_error_code_t iridium_reset_timer(Iridium* self)
{
	if (HAL_TIM_Base_DeInit(self->ten_min_timer) != HAL_OK) {
		return IRIDIUM_TIMER_ERROR;
	}

	self->ten_min_timer->Instance = IRIDIUM_TIMER_INSTANCE;
	self->ten_min_timer->Init.Prescaler = 12000;
	self->ten_min_timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	self->ten_min_timer->Init.Period = 59;
	self->ten_min_timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	self->ten_min_timer->Init.RepetitionCounter = IRIDIUM_MAX_TRANSMIT_PERIOD;
	self->ten_min_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(self->ten_min_timer) != HAL_OK) {
		return IRIDIUM_TIMER_ERROR;
	}

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
static iridium_error_code_t send_basic_command_message(Iridium* self, const char* command, uint8_t response_size)
{
	char * needle;

	self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

	HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(command[0]),
			strlen(command), ONE_SECOND);
	HAL_UART_Receive(self->iridium_uart_handle, &(self->response_buffer[0]),
			response_size, ONE_SECOND);

	needle = strstr((char*)&(self->response_buffer[0]), "OK");

	memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

	return (needle == NULL) ? IRIDIUM_COMMAND_RESPONSE_ERROR : IRIDIUM_SUCCESS;
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
 * Helper method to transmit a message via Iridium modem. Remember, ASCII chars
 * are actually ints.
 * !! Warning, goto statements ahead, please don't lose your mind.
 *
 * @param payload - pointer to a SBD message payload
 * @return iridium_error_code_t - Either IRIDIUM_UART_ERROR or IRIDIUM_SUCCESS
 */
static iridium_error_code_t transmit_message(Iridium* self, uint8_t* payload)
{
	char* needle;
	char SBDWB_response_code;
	int SBDIX_response_code;
	int fail_counter[3] = {0,0,0};
	bool network_available = false;
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

	/* Control flow will use labels and goto statements to jump around */
	LOAD_SBD_LABEL: // Tell the modem we are sending over a SBD message
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(load_sbd[0]),
				strlen(load_sbd), ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDWB_READY_RESPONSE_SIZE, ONE_SECOND);

		needle = strstr((char*)&(self->response_buffer[0]), "READY");

		// Success case
		if (needle != NULL) {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			fail_counter[0] = 0;
			goto SBDWB_LABEL;
		}

		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

		if (++fail_counter[0] == 10) {
			goto RETURN_UART_ERROR_LABEL;
		}

		cycle_power(self);
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		goto LOAD_SBD_LABEL;

		/* end LOAB_SBD_LABEL */
	SBDWB_LABEL: // send over the payload and checksum
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(payload[0]),
				IRIDIUM_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH, ONE_SECOND);

		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDWB_LOAD_RESPONSE_SIZE, ONE_SECOND);

		SBDWB_response_code = self->response_buffer[SBDWB_RESPONSE_CODE_INDEX];

		// Success case
		if (SBDWB_response_code == '0') {
			memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);
			fail_counter[1] = 0;
			goto SBDIX_LABEL;
		}

		// Response of 2 means checksum didn't match, so try calculating it again
		if (SBDWB_response_code == '2') {
			checksum = get_checksum(payload, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
			payload[CHECKSUM_SECOND_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
			checksum_ptr++;
			payload[CHECKSUM_FIRST_BYTE_INDEX] = ((uint8_t)*checksum_ptr);
		}

		memset(&(self->response_buffer[0]), 0, IRIDIUM_MAX_RESPONSE_SIZE);

		if (++fail_counter[1] == 10) {
			goto RETURN_UART_ERROR_LABEL;
		}

		cycle_power(self);
		self->reset_uart(self, IRIDIUM_DEFAULT_BAUD_RATE);
		goto LOAD_SBD_LABEL;

		/* end SBDWB_LABEL */
	SBDIX_LABEL: // tell the modem to transmit the payload
		// TODO: add location at the end of the message
		HAL_UART_Transmit(self->iridium_uart_handle, (uint8_t*)&(send_sbd[0]),
				strlen(send_sbd), ONE_SECOND);

		// We will only grab the response up to and including MO status
		HAL_UART_Receive(self->iridium_uart_handle,
				&(self->response_buffer[0]), SBDIX_RESPONSE_SIZE, ONE_SECOND * 30);
		// Grab the MO status
		SBDIX_response_code = atoi((char*)&(self->response_buffer[SBDIX_RESPONSE_CODE_INDEX]));

		// A response code of 0-4 indicates success
		if (SBDIX_response_code >= 0 && SBDIX_response_code <= 4) {

			// Clear the MO buffer (or cycle power if that fails)
			if (send_basic_command_message(self, clear_MO, SBDD_RESPONSE_SIZE) ==
					IRIDIUM_COMMAND_RESPONSE_ERROR) {
				cycle_power(self);
			}

			goto RETURN_SUCCESS_LABEL;
		}

		switch (SBDIX_response_code) {

			case 11:

				break;

			case 16:

				break;

			case 32:

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

		if (++fail_counter[2] == 2) { // TODO: change this
			goto RETURN_TX_FAILURE_LABEL;
		}

		network_available = false;
		while (network_available == false) {
			network_available = HAL_GPIO_ReadPin(GPIOD, IRIDIUM_NetAv_Pin);
			HAL_Delay(100);
		}

		if (send_basic_command_message(self, clear_MO, SBDD_RESPONSE_SIZE) ==
				IRIDIUM_COMMAND_RESPONSE_ERROR) {
			cycle_power(self);
		}

		goto LOAD_SBD_LABEL;
		/* end SBDIX_LABEL */

	// Return labels -- this is just to be pedantic so return cases are easy to track
	RETURN_SUCCESS_LABEL:
		return IRIDIUM_SUCCESS;

	RETURN_TX_FAILURE_LABEL:
		return IRIDIUM_TRANSMIT_ERROR;

	RETURN_UART_ERROR_LABEL:
		return IRIDIUM_UART_ERROR;
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
//
//static void uart_receive_dma_callback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == IRIDIUM_UART_INSTANCE) {
//			dma_message_received = true;
//	}
//}
