/*
 * gnss.c
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 *
 *`TODO: Make the arrays!!
 *`TODO: Add values to the arrays in processMessage
 *`TODO: Figure out set rtc and turn it into a function
 *`TODO: figure out signaling for CT sensor
 *`TODO: Power pin toggling in init
 *`TODO: Create teardown function
 */



#include "gnss.h"

static gnss_error_code_t reset_gnss_uart(GNSS* self, uint16_t baud_rate);
static gnss_error_code_t send_config(GNSS* self, uint8_t* config_array,
		size_t message_size);
static gnss_error_code_t stop_start_gnss(GNSS* self, bool send_stop);
static void get_checksum(uint8_t* ck_a, uint8_t* ck_b, uint8_t* buffer,
		uint32_t num_bytes);

/**
 * Initialize the GNSS struct
 *
 * @return void
 */
void gnss_init(GNSS* self, UART_HandleTypeDef* gnss_uart_handle,
		DMA_HandleTypeDef* gnss_dma_handle, TX_EVENT_FLAGS_GROUP* event_flags,
		TX_QUEUE* message_queue, int16_t* GNSS_N_Array, int16_t* GNSS_E_Array,
		int16_t* GNSS_D_Array)
{
	/* TODO:Turn on power pin
	 *      Setup uart?
	 *      Create arrays
	 *      wait until good fix?
	 */
	// initialize everything to 0/false
	self->gnss_uart_handle = gnss_uart_handle;
	self->gnss_dma_handle = gnss_dma_handle;
	self->GNSS_N_Array = GNSS_N_Array;
	self->GNSS_E_Array = GNSS_E_Array;
	self->GNSS_D_Array = GNSS_D_Array;
	self->vNorthSum = 0;
	self->vEastSum = 0;
	self->vDownSum = 0;
	self->currentLatitude = 0;
	self->currentLongitude = 0;
	self->totalSamples = 0;
	self->totalSamplesAveraged = 0;
	self->numberCyclesWithoutData = 0;
	self->isConfigured = false;
	self->latLongIsValid = false;
	self->velocityIsValid = false;
	self->clockHasBeenSet = false;
	self->validMessageProcessed = false;
	self->event_flags = event_flags;
	self->message_queue = message_queue;
	self->config = gnss_config;
	self->get_location = gnss_get_location;
	self->get_running_average_velocities = gnss_get_running_average_velocities;
	self->gnss_process_message = gnss_process_message;
	self->sleep = gnss_sleep;
}

/**
 * Configure the MAX-M10S chip by sending a series of UBX_CFG_VALSET messages
 *
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
gnss_error_code_t gnss_config(GNSS* self){
	// The configuration message, type UBX_CFG_VALSET
	uint8_t config[129] =
	{0xB5,0x62,0x06,0x8A,0x79,0x00,0x01,0x01,0x00,0x00,0xBA,0x00,0x91,0x20,0x00,
	 0xBE,0x00,0x91,0x20,0x00,0xBB,0x00,0x91,0x20,0x00,0xC9,0x00,0x91,0x20,0x00,
	 0xCD,0x00,0x91,0x20,0x00,0xCA,0x00,0x91,0x20,0x00,0xBF,0x00,0x91,0x20,0x00,
	 0xC3,0x00,0x91,0x20,0x00,0xC0,0x00,0x91,0x20,0x00,0xC4,0x00,0x91,0x20,0x00,
	 0xC8,0x00,0x91,0x20,0x00,0xC5,0x00,0x91,0x20,0x00,0xAB,0x00,0x91,0x20,0x00,
	 0xAF,0x00,0x91,0x20,0x00,0xAC,0x00,0x91,0x20,0x00,0xB0,0x00,0x91,0x20,0x00,
	 0xB4,0x00,0x91,0x20,0x00,0xB1,0x00,0x91,0x20,0x00,0x07,0x00,0x91,0x20,0x01,
	 0x21,0x00,0x11,0x20,0x08,0x04,0x00,0x93,0x10,0x00,0x01,0x00,0x21,0x30,0xC8,
	 0x00,0x02,0x00,0x21,0x30,0x01,0x00,0xE6,0x4D};

	// Send over the configuration settings for RAM
	if (send_config(self, &(config[0]), sizeof(config)) == GNSS_CONFIG_ERROR) {
		return GNSS_CONFIG_ERROR;
	}

	// Only one value (configuration layer) and the checksum change between RAM
	// and Battery-backed-RAM, so we'll adjust that now
	config[7] = 0x02;
	config[127] = 0xE7;
	config[128] = 0xC5;
	// Send over the BBR config settings
	if (send_config(self, &(config[0]), sizeof(config)) == GNSS_CONFIG_ERROR) {
		return GNSS_CONFIG_ERROR;
	}

//	// Stop GNSS signal processing so we can more easily grab the ACK/NAK msg
	if ((stop_start_gnss(self, true)) == GNSS_CONFIG_ERROR) {
		return GNSS_CONFIG_ERROR;
	}
	// Wait to make sure we don't get any more GNSS messages
	HAL_Delay(200);
	// Start GNSS signal processing
	if ((stop_start_gnss(self, false)) == GNSS_CONFIG_ERROR) {
		return GNSS_CONFIG_ERROR;
	}
	HAL_Delay(200);
	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);
	reset_gnss_uart(self, 9600);

	self->isConfigured = true;
	return GNSS_SUCCESS;
}

/**
 * Process the messages in the buffer.
 *
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
gnss_error_code_t gnss_process_message(GNSS* self)
{
	ULONG num_msgs_enqueued, available_space;
	UINT ret;
//	const char message[UBX_NAV_PVT_MESSAGE_LENGTH];
	uint8_t *message;
	uint8_t **msg_ptr = &message;
	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
    int32_t message_class = 0;
    int32_t message_id = 0;
    int32_t num_payload_bytes = 0;
    // start with 0'd out buffers;
    memset(message, 0, UBX_NAV_PVT_MESSAGE_LENGTH);
//    memset(payload, 0, sizeof(payload));
    // Reset the valid message processed flag
    self->validMessageProcessed = false;

    // Grab a message
    tx_queue_receive(self->message_queue, (VOID*)msg_ptr, TX_WAIT_FOREVER);
	// How many messages are on the queue
	tx_queue_info_get(self->message_queue, TX_NULL, &num_msgs_enqueued,
						&available_space, TX_NULL, TX_NULL, TX_NULL);
	// More messages may be added while processing, but we'll leave those
	// for the next time around and only get those on the queue right now.
	while (num_msgs_enqueued > 0) {
	    // Try to decode message
	    num_payload_bytes = uUbxProtocolDecode((const char*)&(message[0]),
	    		UBX_NAV_PVT_MESSAGE_LENGTH, &message_class, &message_id,
				payload, sizeof(payload), NULL);
	    // UBX_NAV_PVT payload is 92 bytes, if we don't have that many, its
	    // a bad message
		if (num_payload_bytes != UBX_NAV_PVT_PAYLOAD_LENGTH) {
		    // Grab a message
		    tx_queue_receive(self->message_queue, (VOID*)msg_ptr, TX_WAIT_FOREVER);
			// How many messages are on the queue
			tx_queue_info_get(self->message_queue, TX_NULL, &num_msgs_enqueued,
								&available_space, TX_NULL, TX_NULL, TX_NULL);
			continue;
		}

		// Grab a bunch of things from the message
		int32_t tAcc = payload[12] + (payload[13]<<8) + (payload[14]<<16) + (payload[15]<<24);
		uint16_t year = payload[4] + (payload[5]<<8);
		uint8_t month = payload[6];
		uint8_t day = payload[7];
		uint8_t hour = payload[8];
		uint8_t min = payload[9];
		uint8_t sec = payload[10];
		int32_t lon = payload[24] + (payload[25]<<8) + (payload[26]<<16) + (payload[27]<<24);
		int32_t lat = payload[28] + (payload[29]<<8) + (payload[30]<<16) + (payload[31]<<24);
		int16_t pDOP =  payload[76] + (payload[77]<<8);
		int32_t sAcc = payload[68] + (payload[69] << 8) + (payload[70] << 16) + (payload[71] << 24);
		int32_t vnorth = payload[48] + (payload[49] << 8) + (payload[50] << 16) + (payload[51] << 24);
		int32_t veast = payload[52] + (payload[53] << 8) + (payload[54] << 16) + (payload[55] << 24);
		int32_t vdown = payload[56] + (payload[57] << 8) + (payload[58] << 16) + (payload[59] << 24);

		// Start by setting the clock if needed
		if (!self->clockHasBeenSet) {
			// Grab time accuracy estimate
			// TODO: check valid time flag first


			if (tAcc < MAX_ACCEPTABLE_TACC) {
				// TODO: Figure this out and then make it a function
				// set clock

				bool isAM = true;
				if (hour > 12) { hour %= 12; isAM = false;}

				RTC_TimeTypeDef time = {hour, min, sec,
						(isAM) ?
						RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM, 0, 0};
				// Set RTC time

				self->clockHasBeenSet = true;
			}
		}

		// Check Lat/Long accuracy, assign to class fields if good
		if (pDOP < MAX_ACCEPTABLE_PDOP) {
			self->currentLatitude = lat;
			self->currentLongitude = lon;
		}

		// Grab velocities, start by checking speed accuracy estimate (sAcc)
		if (sAcc > MAX_ACCEPTABLE_SACC) {
			// This message was not within acceptable parameters,
			--num_msgs_enqueued;
			continue;
		}

		// vAcc was within acceptable range, still need to check
		// individual velocities are less than MAX_POSSIBLE_VELOCITY
		if ((vnorth > MAX_POSSIBLE_VELOCITY) ||
			(veast > MAX_POSSIBLE_VELOCITY) ||
			(vdown > MAX_POSSIBLE_VELOCITY)) {
			// One or more velocity component was greater than the
			// max possible velocity. Loop around and try again
			--num_msgs_enqueued;
			continue;
		}

		// All velocity values are good to go, convert them to
		// shorts and store them in the arrays
		self->GNSS_N_Array[self->totalSamples] = (int16_t)vnorth;
		self->GNSS_E_Array[self->totalSamples] = (int16_t)veast;
		self->GNSS_D_Array[self->totalSamples] = (int16_t)vdown;

		self->numberCyclesWithoutData = 0;
		self->totalSamples++;
		self->validMessageProcessed = true;

	}

    if (!self->validMessageProcessed) {
    	// We weren't able to get a valid message from the buffer, so we'll sub
    	// a running average iff there are more than 0 valid samples so far
    	if (++self->numberCyclesWithoutData > MAX_EMPTY_CYCLES) {
    		// Some sort of bail out condition
    		// We might send a message with a specific payload to indicate
    		// things went wrong.

    		//TODO: figure out what we'll do in this situation. Likely revert
    		//      to taking measurements from the IMU
    	} else {
    		// We'll replace the values with running average
    		int16_t north = 0, east = 0, down = 0;
    		// make sure there are samples to average
    		gnss_error_code_t ret = self->get_running_average_velocities(
    				self, &north, &east, &down);
    		if (ret == GNSS_SUCCESS) {
    			// got the averaged values
    			self->GNSS_N_Array[self->totalSamples] = north;
    			self->GNSS_E_Array[self->totalSamples] = east;
    			self->GNSS_D_Array[self->totalSamples] = down;
    		} else {
    			// Wasn't able to get a running average -- this will return
    			// GNSS_NO_SAMPLES_ERROR
    			return ret;
			}
    	}
    }
    // Lastly, flush the queue and start fresh
    ret = tx_queue_flush(self->message_queue);
    if (ret != TX_SUCCESS) {
    	// Maybe something got weird from the ISR, try again
    	ret = tx_queue_flush(self->message_queue);
    }
    // A valid message was processed
    return GNSS_SUCCESS;
}


///**
// * Process the messages in the buffer.
// *
// * @return GPS error code (marcos defined in gps_error_codes.h)
// */
//gnss_error_code_t gnss_process_message(GNSS* self, TX_QUEUE* message_queue)
//{
//	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
//	const char* buf_start = (const char*)&(process_buf[0]);
//	const char** buf_end = &buf_start;
//	size_t buf_length = sizeof(process_buf);
//    int32_t message_class = 0;
//    int32_t message_id = 0;
//    int32_t num_payload_bytes = 0;
//
//
//
//	for (int32_t i = uUbxProtocolDecode(buf_start, buf_length,
//	    		 &message_class, &message_id, payload, sizeof(payload), buf_end);
//			     i > 0;
//				 i = uUbxProtocolDecode(buf_start, buf_length,
//		         &message_class, &message_id, payload, sizeof(payload), buf_end))
//	{
//	    // UBX_NAV_PVT payload is 92 bytes, if we don't have that many, its
//	    // a bad message
//		if (num_payload_bytes != UBX_NAV_PVT_PAYLOAD_LENGTH) {
//			// We'll replace the values with running average
//			int16_t north = 0, east = 0, down = 0;
//			// make sure there are samples to average
//			gnss_error_code_t ret = self->get_running_average_velocities(
//					self, &north, &east, &down);
//			if (ret == GNSS_SUCCESS) {
//				// got the averaged values
//				self->GNSS_N_Array[self->totalSamples] = north;
//				self->GNSS_E_Array[self->totalSamples] = east;
//				self->GNSS_D_Array[self->totalSamples] = down * -1;
//				++self->numberCyclesWithoutData;
//				continue;
//			}
//		}
//			// Start by setting the clock if needed
//		if (!self->clockHasBeenSet) {
//			// Grab time accuracy estimate
//			// TODO: check valid time flag first
//			int32_t tAcc = payload[12] +
//					(payload[13]<<8) +
//					(payload[14]<<16) +
//					(payload[15]<<24);
//
//			if (tAcc < MAX_ACCEPTABLE_TACC) {
//				// TODO: Figure this out and then make it a function
//				// set clock
//				uint16_t year = payload[4] + (payload[5]<<8);
//				uint8_t month = payload[6];
//				uint8_t day = payload[7];
//				uint8_t hour = payload[8];
//				bool isAM = true;
//				if (hour > 12) { hour %= 12; isAM = false;}
//				uint8_t min = payload[9];
//				uint8_t sec = payload[10];
//
//				RTC_TimeTypeDef time = {hour, min, sec,
//						(isAM) ?
//						RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM, 0, 0};
//				// Set RTC time
//
//				self->clockHasBeenSet = true;
//			}
//		}
//		// Check Lat/Long accuracy, assign to class fields if good
//		int32_t lon = payload[24] +
//				(payload[25]<<8) +
//				(payload[26]<<16) +
//				(payload[27]<<24);
//		int32_t lat = payload[28] +
//				(payload[29]<<8) +
//				(payload[30]<<16) +
//				(payload[31]<<24);
//		int16_t pDOP =  payload[76] +
//				(payload[77]<<8);
//
//		if (pDOP < MAX_ACCEPTABLE_PDOP) {
//			self->currentLatitude = lat;
//			self->currentLongitude = lon;
//		}
//
//		// Grab velocities, start by checking speed accuracy estimate (sAcc)
//		int32_t sAcc = payload[68] +
//				(payload[69] << 8) +
//				(payload[70] << 16) +
//				(payload[71] << 24);
//
//		if (sAcc > MAX_ACCEPTABLE_SACC) {
//			// This message was not within acceptable parameters,
//			// We'll replace the values with running average
//			int16_t north = 0, east = 0, down = 0;
//			// make sure there are samples to average
//			gnss_error_code_t ret = self->get_running_average_velocities(
//					self, &north, &east, &down);
//			if (ret == GNSS_SUCCESS) {
//				// got the averaged values
//				self->GNSS_N_Array[self->totalSamples] = north;
//				self->GNSS_E_Array[self->totalSamples] = east;
//				self->GNSS_D_Array[self->totalSamples] = down;
//				++self->numberCyclesWithoutData;
//				continue;
//			} else {
//				// Wasn't able to get a running average -- this will return
//				// GNSS_NO_SAMPLES_ERROR
//				// TODO: this condition needs special handling
//				continue;
//			}
//		}
//		// vAcc was within acceptable range, still need to check
//		// individual velocities are less than MAX_POSSIBLE_VELOCITY
//		int32_t vnorth = payload[48] +
//				(payload[49] << 8) +
//				(payload[50] << 16) +
//				(payload[51] << 24);
//		int32_t veast = payload[52] +
//				(payload[53] << 8) +
//				(payload[54] << 16) +
//				(payload[55] << 24);
//		int32_t vdown = payload[56] +
//				(payload[57] << 8) +
//				(payload[58] << 16) +
//				(payload[59] << 24);
//
//		if ((vnorth > MAX_POSSIBLE_VELOCITY) ||
//			(veast > MAX_POSSIBLE_VELOCITY) ||
//			(vdown > MAX_POSSIBLE_VELOCITY)) {
//			// One or more velocity component was greater than the
//			// max possible velocity. Loop around and try again
//			// We'll replace the values with running average
//			int16_t north = 0, east = 0, down = 0;
//			// make sure there are samples to average
//			gnss_error_code_t ret = self->get_running_average_velocities(
//					self, &north, &east, &down);
//			if (ret == GNSS_SUCCESS) {
//				// got the averaged values
//				self->GNSS_N_Array[self->totalSamples] = north;
//				self->GNSS_E_Array[self->totalSamples] = east;
//				self->GNSS_D_Array[self->totalSamples] = down;
//				++self->numberCyclesWithoutData;
//			} else {
//				// Wasn't able to get a running average -- this will return
//				// GNSS_NO_SAMPLES_ERROR
//				// TODO: this condition needs special handling
//			}
//		} else {
//			// All velocity values are good to go, convert them to
//			// shorts and store them in the arrays
//			self->GNSS_N_Array[self->totalSamples] = (int16_t)veast;
//			self->GNSS_E_Array[self->totalSamples] = (int16_t)vnorth;
//			self->GNSS_D_Array[self->totalSamples] = (int16_t)(vdown);
//
//			self->numberCyclesWithoutData = 0;
//			self->totalSamples++;
//			self->validMessageProcessed = true;
//		}
//	}
//    return GNSS_SUCCESS;
//}

/**
 * Get the current lat/long.
 *
 * @param latitude - return parameter for latitude
 * @param longitude - return parameter for longitude
 * @return GPS error code (marcos defined in gps.h)
 */
gnss_error_code_t gnss_get_location(GNSS* self, int32_t* latitude,
		int32_t* longitude)
{
	if (self->latLongIsValid) {
		*latitude = self->currentLatitude;
		*longitude = self->currentLongitude;
		return GNSS_SUCCESS;
	} else {
		return GNSS_LOCATION_INVALID;
	}
}

/**
 * If a velocity field > MAX_POSSIBLE_VELOCITY, or the velocity accuracy estimate (vAcc) is
 * outside acceptable range, this function will substitute a running average.
 *
 * @param returnNorth - return parameter for the running average North value
 * @param returnEast - return parameter for the running average East value
 * @param returnDown - return parameter for the running average Down value
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
gnss_error_code_t gnss_get_running_average_velocities(GNSS* self,
		int16_t* returnNorth, int16_t* returnEast, int16_t* returnDown)
{
	if (self->totalSamples > 0) {
		float substituteNorth = self->vNorthSum /
				((float)self->totalSamples);
		*returnNorth = (int16_t)substituteNorth;
		self->vNorthSum += substituteNorth;

		float substituteEast = self->vEastSum /
				((float)self->totalSamples);
		*returnEast = (int16_t)substituteEast;
		self->vEastSum += substituteEast;

		float substituteDown = self->vDownSum /
				((float)self->totalSamples);
		*returnDown = (int16_t)substituteDown;
		self->vDownSum += substituteDown;

		self->totalSamplesAveraged++;
		return GNSS_SUCCESS;
	} else {
		// No valid samples yet, avoid divide by zero error
		return GNSS_NO_SAMPLES_ERROR;
	}
}

/**
 *
 *
 * @param self - GNSS struct
 */
gnss_error_code_t gnss_sleep(GNSS* self)
{
}

/**
 * Reinitialize the GNSS UART port. Required when switching between Tx and Rx.
 *
 * @param self - GNSS struct
 * @param baud_rate - baud rate to set port to
 */
static gnss_error_code_t reset_gnss_uart(GNSS* self, uint16_t baud_rate)
{
	if (HAL_UART_DeInit(self->gnss_uart_handle) != HAL_OK) {
		return GNSS_UART_ERROR;
	}
	self->gnss_uart_handle->Instance = USART3;
	self->gnss_uart_handle->Init.BaudRate = baud_rate;
	self->gnss_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	self->gnss_uart_handle->Init.StopBits = UART_STOPBITS_1;
	self->gnss_uart_handle->Init.Parity = UART_PARITY_NONE;
	self->gnss_uart_handle->Init.Mode = UART_MODE_TX_RX;
	self->gnss_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->gnss_uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
	self->gnss_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	self->gnss_uart_handle->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	self->gnss_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(self->gnss_uart_handle) != HAL_OK) {
		return GNSS_UART_ERROR;
	}

	if (HAL_UARTEx_SetTxFifoThreshold(self->gnss_uart_handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return GNSS_UART_ERROR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(self->gnss_uart_handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return GNSS_UART_ERROR;
	}
	if (HAL_UARTEx_DisableFifoMode(self->gnss_uart_handle) != HAL_OK)
	{
		return GNSS_UART_ERROR;
	}
	return GNSS_SUCCESS;
}

/**
 * Send a configuration to the GNSS chip. Will retry up to 10 times before
 * returning failure.
 *
 * @param self- GNSS struct
 * @param config_array - byte array containing a UBX_CFG_VALSET msg with up to
 * 		  64 keys
 */
static gnss_error_code_t send_config(GNSS* self, uint8_t* config_array,
		size_t message_size)
{
	uint8_t fail_counter = 0;
	ULONG actual_flags;
	uint8_t msg_buf[600];
	memset(&(msg_buf[0]),0,sizeof(msg_buf));
	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
	const char* buf_start = (const char*)&(msg_buf[0]);
	const char** buf_end = &buf_start;
	size_t buf_length = sizeof(msg_buf);
    int32_t message_class = 0;
    int32_t message_id = 0;
    int32_t num_payload_bytes = 0;
	// The configuration message, type UBX_CFG_VALSET

	while (++fail_counter <= 10) {
		// Send over the configuration settings
		HAL_UART_Transmit(self->gnss_uart_handle, &(config_array[0]),
				message_size, 1000);
		LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);
		// Grab the acknowledgment message
		HAL_UART_Receive_DMA(self->gnss_uart_handle, &(msg_buf[0]),
				sizeof(msg_buf));
		__HAL_DMA_DISABLE_IT(self->gnss_dma_handle, DMA_IT_HT);
		tx_event_flags_get(self->event_flags, GNSS_CONFIG_RECVD, TX_OR_CLEAR,
				&actual_flags, TX_WAIT_FOREVER);

		/* The ack/nak message is guaranteed to be sent within one second, but
		 * we may receive a few navigation messages before the ack is received,
		 * so we have to sift through at least one second worth of messages */
		for (num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
				&message_class, &message_id, payload, sizeof(payload), buf_end);
				num_payload_bytes > 0;
				num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
				&message_class, &message_id, payload, sizeof(payload), buf_end))
		{
			if (message_class == 0x05) {
				// Msg class 0x05 is either an ACK or NAK
				if (message_id == 0x00) {
					// This is a NAK msg, the config did not go through properly
					break;
				} else if (message_id == 0x01) {
					LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);
					reset_gnss_uart(self, 9600);
					return GNSS_SUCCESS;
				}
			}
		}

		// Reinitialize the UART port and restart DMA receive
		reset_gnss_uart(self, 9600);
		// Zero out the buffer to prevent reading old values
		memset(&(msg_buf[0]),0,sizeof(msg_buf));
		buf_start = (const char*)&(msg_buf[0]);
		buf_end = &buf_start;
	}
	// If we made it here, config failed 10 attempts
	reset_gnss_uart(self, 9600);
	return GNSS_CONFIG_ERROR;
}

/**
 * Send a CFG_RST message to the GNSS chip to either start or stop GNSS
 * processing. This message is not acknowledged, so we just have to trust that
 * it worked.
 *
 * @param self- GNSS struct
 * @param send_stop - true: send a stop message; false: sent a start message.
 */
static gnss_error_code_t stop_start_gnss(GNSS* self, bool send_stop)
{
	// 3rd byte -- 0x08 = Controlled GNSS stop, 0x09 = Controlled GNSS start
	uint8_t message_payload[4] = {0x00, 0x00, (send_stop) ? 0x08 : 0x09, 0x00};
	char cfg_rst_message[sizeof(message_payload) +
						 U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES];

	if ((uUbxProtocolEncode(0x06, 0x04, (const char*)&(message_payload[0]),
			sizeof(message_payload),cfg_rst_message)) < 0)
	{
		return GNSS_CONFIG_ERROR;
	}

	if ((HAL_UART_Transmit(self->gnss_uart_handle, (uint8_t*)&(cfg_rst_message[0]),
					sizeof(cfg_rst_message), 1000)) != HAL_OK)
	{
		return GNSS_CONFIG_ERROR;
	}
//	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);
	// Reinitialize the UART port and restart DMA receive
//	reset_gnss_uart(self, 9600);
	return GNSS_SUCCESS;
}

/**
 * Calculate the two checksum bytes for a UBX message
 *
 * @param ck_a - reference to first checksum byte
 * @param ck_b - reference to second checksum byte
 * @param buffer - address of first byte of array
 * @param num_bytes - number of bytes to calculate checksum over
 */
static void get_checksum(uint8_t* ck_a, uint8_t* ck_b, uint8_t* buffer,
		uint32_t num_bytes)
{
	*ck_a = 0;
	*ck_b = 0;
	for (int i = 0; i < num_bytes; i++) {
		*ck_a = *ck_a + buffer[i];
		*ck_b = *ck_b + *ck_a;
	}
}
