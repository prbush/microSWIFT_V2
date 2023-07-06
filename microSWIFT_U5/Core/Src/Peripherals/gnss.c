/*
 * gnss.c
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 *
 */



#include "Peripherals/gnss.h"

// Static helper functions
static gnss_error_code_t send_config(GNSS* self, uint8_t* config_array,
		size_t message_size, uint8_t response_class, uint8_t response_id);
static gnss_error_code_t stop_start_gnss(GNSS* self, bool send_stop);
static void process_frame_sync_messages(GNSS* self, uint8_t* process_buf);
static gnss_error_code_t enable_high_performance_mode(GNSS* self);
static gnss_error_code_t query_high_performance_mode(GNSS* self);
static void get_checksum(uint8_t* ck_a, uint8_t* ck_b, uint8_t* buffer,
		uint32_t num_bytes)__attribute__((unused));
static uint32_t get_timestamp(GNSS* self);
static void reset_struct_fields(GNSS* self);

/**
 * Initialize the GNSS struct
 *
 * @return void
 */
void gnss_init(GNSS* self, microSWIFT_configuration* global_config,
		UART_HandleTypeDef* gnss_uart_handle, DMA_HandleTypeDef* gnss_rx_dma_handle,
		DMA_HandleTypeDef* gnss_tx_dma_handle, TX_EVENT_FLAGS_GROUP* control_flags,
		TX_EVENT_FLAGS_GROUP* error_flags, TIM_HandleTypeDef* timer, uint8_t* ubx_process_buf,
		uint8_t* config_response_buffer, RTC_HandleTypeDef* rtc_handle, float* GNSS_N_Array,
		float* GNSS_E_Array, float* GNSS_D_Array)
{
	// initialize everything
	self->global_config = global_config;
	self->gnss_uart_handle = gnss_uart_handle;
	self->gnss_rx_dma_handle = gnss_rx_dma_handle;
	self->gnss_tx_dma_handle = gnss_tx_dma_handle;
	self->rtc_handle = rtc_handle;
	self->GNSS_N_Array = GNSS_N_Array;
	self->GNSS_E_Array = GNSS_E_Array;
	self->GNSS_D_Array = GNSS_D_Array;
	reset_struct_fields(self);
	self->control_flags = control_flags;
	self->error_flags = error_flags;
	self->minutes_timer = timer;
	self->ubx_process_buf = ubx_process_buf;
	self->config_response_buf =  config_response_buffer;
	self->config = gnss_config;
	self->sync_and_start_reception = gnss_sync_and_start_reception;
	self->get_location = gnss_get_location;
	self->get_running_average_velocities = gnss_get_running_average_velocities;
	self->process_message = gnss_process_message;
	self->sleep = gnss_sleep;
	self->on_off = gnss_on_off;
	self->cycle_power = gnss_cycle_power;
	self->set_rtc = gnss_set_rtc;
	self->reset_uart = gnss_reset_uart;
	self->reset_timer = gnss_reset_timer;
}

/**
 * Configure the MAX-M10S chip by sending a series of UBX_CFG_VALSET messages
 *
 * @return GNSS_SUCCESS or
 * 		   GNSS_CONFIG_ERROR if response was not received
 */
gnss_error_code_t gnss_config(GNSS* self){
	gnss_error_code_t return_code;
	// The configuration message, type UBX_CFG_VALSET. Default is set to 4Hz.
	// !!!! This is output from U-Center 2 software, do not change !!!
	uint8_t config[CONFIGURATION_ARRAY_SIZE] =
    {0xB5,0x62,0x06,0x8A,0x9C,0x00,0x01,0x01,0x00,0x00,
     0xBA,0x00,0x91,0x20,0x00,0xBE,0x00,0x91,0x20,0x00,
     0xBB,0x00,0x91,0x20,0x00,0xC9,0x00,0x91,0x20,0x00,
     0xCD,0x00,0x91,0x20,0x00,0xCA,0x00,0x91,0x20,0x00,
     0xBF,0x00,0x91,0x20,0x00,0xC3,0x00,0x91,0x20,0x00,
     0xC0,0x00,0x91,0x20,0x00,0xC4,0x00,0x91,0x20,0x00,
     0xC8,0x00,0x91,0x20,0x00,0xC5,0x00,0x91,0x20,0x00,
     0xAB,0x00,0x91,0x20,0x00,0xAF,0x00,0x91,0x20,0x00,
     0xAC,0x00,0x91,0x20,0x00,0xB0,0x00,0x91,0x20,0x00,
     0xB4,0x00,0x91,0x20,0x00,0xB1,0x00,0x91,0x20,0x00,
     0x07,0x00,0x91,0x20,0x01,0x21,0x00,0x11,0x20,0x08,
     0x04,0x00,0x93,0x10,0x00,0x01,0x00,0x21,0x30,0xC8,
     0x00,0x02,0x00,0x21,0x30,0x01,0x00,0x07,0x00,0x92,
     0x20,0x00,0x06,0x00,0x92,0x20,0x00,0x0A,0x00,0x92,
     0x20,0x00,0x0D,0x00,0x31,0x10,0x00,0x0F,0x00,0x31,
     0x10,0x01,0x18,0x00,0x31,0x10,0x01,0xA4,0x00,0x11,
     0x20,0x14,0x18,0x5C};

	if (self->global_config->gnss_sampling_rate == 5) {
		config[119] = 0xFA;
		config[162] = 0x4A;
		config[163] = 0xC2;
	}


	// Send over the configuration settings for RAM
	return_code = send_config(self, &(config[0]), CONFIGURATION_ARRAY_SIZE, UBX_CFG_VALSET_CLASS,
			UBX_CFG_VALSET_ID);

	if (return_code != GNSS_SUCCESS) {
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return return_code;
	}

	// Only one value (configuration layer) and the checksum change between RAM
	// and Battery-backed-RAM, so we'll adjust that now
	config[7] = 0x02;

	if (self->global_config->gnss_sampling_rate == 5) {
		config[162] = 0x4B;
		config[163] = 0x5D;
	} else { // 4Hz
		config[162] = 0x19;
		config[163] = 0xF7;
	}


	// Send over the Battery Backed Ram (BBR) config settings
	return_code = send_config(self, &(config[0]), CONFIGURATION_ARRAY_SIZE, UBX_CFG_VALSET_CLASS,
			UBX_CFG_VALSET_ID);

	if (return_code != GNSS_SUCCESS) {

		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return return_code;
	}

	self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);

	// Now set high performance mode (if enabled)
	if (self->global_config->gnss_high_performance_mode) {
		// First check to see if it has already been set
		return_code = enable_high_performance_mode(self);
	}

	return return_code;
}

/**
 *
 *
 * @return gnss_error_code_t
 */
gnss_error_code_t gnss_sync_and_start_reception(GNSS* self, gnss_error_code_t (*start_dma)(GNSS*, uint8_t*, size_t),
		uint8_t* buffer, size_t msg_size)
{
	gnss_error_code_t return_code = GNSS_SELF_TEST_FAILED;
	ULONG actual_flags;
	uint8_t msg_buf[INITIAL_STAGES_BUFFER_SIZE];
	memset(&(msg_buf[0]), 0, INITIAL_STAGES_BUFFER_SIZE);

    // Grabbing and processing 5 samples takes ~ 1 second, so we'll keep trying until we hit
	// the gnss_max_acquisition_wait_time
	while (!self->timer_timeout) {
		register_watchdog_refresh();
		// Grab 5 UBX_NAV_PVT messages
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		// Put a short delay between resetting UART and starting a DMA transfer
		HAL_Delay(1);
		HAL_UART_Receive_DMA(self->gnss_uart_handle, &(msg_buf[0]),
				INITIAL_STAGES_BUFFER_SIZE);

		if (tx_event_flags_get(self->control_flags, GNSS_CONFIG_RECVD, TX_OR_CLEAR,
				&actual_flags, MAX_THREADX_WAIT_TICKS_FOR_CONFIG) != TX_SUCCESS) {
			// If we didn't receive the needed messaged in time, cycle the GNSS sensor
			self->cycle_power(self);
			HAL_UART_DMAStop(self->gnss_uart_handle);
			// Insert a short, prime number delay to sync up UART reception
			HAL_Delay(13);
			continue;
		}

		process_frame_sync_messages(self, msg_buf);
		// this both ensures we have frame sync'd with the GNSS sensor and are safe
		// to kick off circular DMA receive
		if (self->messages_processed == 5 &&
				self->number_cycles_without_data == 0 &&
				self->total_samples == 5)
		{
			return_code = GNSS_SUCCESS;
			break;
		}
	}

	register_watchdog_refresh();

	// Just to be overly sure we're starting the sampling window from a fresh slate
	reset_struct_fields(self);
	self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);

	if (self->timer_timeout) {
		return GNSS_TIME_RESOLUTION_ERROR;
	}

	return_code = start_dma(self, &(buffer[0]), msg_size);
	register_watchdog_refresh();
	// Make sure we start right next time around in case there was an issue starting DMA
	if (return_code == GNSS_UART_ERROR) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		memset(&(buffer[0]), 0, msg_size);

		tx_event_flags_set(self->error_flags, GNSS_ERROR, TX_OR);
	}

	return return_code;
}

/**
 * Process the messages in the buffer.
 *
 * @return gnss_error_code_t
 */
void gnss_process_message(GNSS* self)
{
	uint8_t payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
	const char* buf_start = (const char*)&(self->ubx_process_buf[0]);
	const char* buf_end = buf_start;
	// Our input buffer is a message off the queue, 10 UBX_NAV_PVT msgs
	size_t buf_length = UBX_MESSAGE_SIZE * 2;
	int32_t message_class = 0;
	int32_t message_id = 0;
	int32_t num_payload_bytes = 0;
	int32_t lat, lon, sAcc, vnorth, veast, vdown;
	int16_t pDOP;
	bool is_ubx_nav_pvt_msg, velocities_exceed_max, sAcc_exceeded_max, message_checksum_valid = false;

	// Really gross for loop that processes msgs in each iteration
	for (num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
				 &message_class, &message_id, (char*)payload, sizeof(payload), &buf_end);
			num_payload_bytes > 0;
			num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
				 &message_class, &message_id, (char*)payload, sizeof(payload), &buf_end))
	{

		message_checksum_valid = true;

		// UBX_NAV_PVT payload is 92 bytes, message class is 0x01, message ID is 0x07
		is_ubx_nav_pvt_msg = (num_payload_bytes == UBX_NAV_PVT_PAYLOAD_LENGTH) ||
							 (message_class == UBX_NAV_PVT_MESSAGE_CLASS)  ||
							 (message_id == UBX_NAV_PVT_MESSAGE_ID);

		if (!is_ubx_nav_pvt_msg)
		{
			self->get_running_average_velocities(self);
			self->number_cycles_without_data++;
			buf_length -= buf_end - buf_start;
			buf_start = buf_end;
			continue;
		}

		// Even if we don't end up using the values, we did get a valid message
		self->messages_processed++;

		// Grab a bunch of things from the message
		lon 	= (int32_t) get_four_bytes(payload, UBX_NAV_PVT_LON_INDEX, AS_LITTLE_ENDIAN);
		lat 	= (int32_t) get_four_bytes(payload, UBX_NAV_PVT_LAT_INDEX, AS_LITTLE_ENDIAN);
		pDOP 	= (int16_t) get_two_bytes(payload, UBX_NAV_PVT_PDOP_INDEX, AS_LITTLE_ENDIAN);
		sAcc 	= (int32_t) get_four_bytes(payload, UBX_NAV_PVT_SACC_INDEX, AS_LITTLE_ENDIAN);
		vnorth 	= (int32_t) get_four_bytes(payload, UBX_NAV_PVT_V_NORTH_INDEX, AS_LITTLE_ENDIAN);
		veast 	= (int32_t) get_four_bytes(payload, UBX_NAV_PVT_V_EAST_INDEX, AS_LITTLE_ENDIAN);
		vdown 	= (int32_t) get_four_bytes(payload, UBX_NAV_PVT_V_DOWN_INDEX, AS_LITTLE_ENDIAN);

		// This allows us to make sure we're not in the sampling window if time has not been resolved
		if (!self->is_clock_set) {
			if (self->set_rtc(self, (uint8_t*)payload) != GNSS_SUCCESS){
				continue;
			}
		}

		// We'll always retain the lat/lon and use a flag to indicate if it is any good
		self->current_fix_is_good = (pDOP < MAX_ACCEPTABLE_PDOP);
		self->current_latitude = lat;
		self->current_longitude = lon;

		// vAcc was within acceptable range, still need to check
		// individual velocities are less than MAX_POSSIBLE_VELOCITY
		velocities_exceed_max = (vnorth > MAX_POSSIBLE_VELOCITY) ||
								(veast > MAX_POSSIBLE_VELOCITY) ||
								(vdown > MAX_POSSIBLE_VELOCITY);

		sAcc_exceeded_max = sAcc > MAX_ACCEPTABLE_SACC;

		// Did we have at least 1 good sample?
		if ((self->total_samples == 0) && (!velocities_exceed_max) && (!sAcc_exceeded_max)) {
			self->all_resolution_stages_complete = true;
			self->sample_window_start_time = get_timestamp(self);
		}

		// Make sure we don't overflow our arrays
		if (self->total_samples >= self->global_config->samples_per_window) {
			HAL_UART_DMAStop(self->gnss_uart_handle);
			self->sample_window_stop_time = get_timestamp(self);
			self->all_samples_processed = true;
			self->sample_window_freq = (double)(((double)self->global_config->samples_per_window) /
					(((double)(self->sample_window_stop_time - self->sample_window_start_time))));

			return;
		}

		// Check if the velocity values are any good
		if (sAcc_exceeded_max | velocities_exceed_max) {
			// This message was not within acceptable parameters,
			self->get_running_average_velocities(self);
			buf_length -= buf_end - buf_start;
			buf_start = buf_end;
			continue;
		}

		// All velocity values are good to go
		self->v_north_sum += vnorth;
		self->v_east_sum += veast;
		self->v_down_sum += vdown;

		self->GNSS_N_Array[self->total_samples] = ((float)((float)vnorth)/MM_PER_METER);
		self->GNSS_E_Array[self->total_samples] = ((float)((float)veast)/MM_PER_METER);
		self->GNSS_D_Array[self->total_samples] = ((float)((float)vdown)/MM_PER_METER);

		self->number_cycles_without_data = 0;
		self->total_samples++;

		buf_length -= buf_end - buf_start;
		buf_start = buf_end;
	}

	// If the checksum was invalid, replace with running average
	if (!message_checksum_valid) {
		self->get_running_average_velocities(self);
	}
}

/**
 * Get the current lat/long. We're going to return the lat/long no matter what,
 * but the return code will indicate if it's any good.
 * !!! Only valid when the GNSS is on and procesing samples or shortly thereafter
 *
 * @param latitude - return parameter for latitude
 * @param longitude - return parameter for longitude
 * @return gnss_error_code_t
 */
gnss_error_code_t gnss_get_location(GNSS* self, float* latitude, float* longitude)
{
	gnss_error_code_t return_code = GNSS_SUCCESS;

	if (!self->current_fix_is_good) {
		return_code = GNSS_LOCATION_INVALID;
	}

	*latitude = ((float)self->current_latitude)/((float)LAT_LON_CONVERSION_FACTOR);
	*longitude = ((float)self->current_longitude)/((float)LAT_LON_CONVERSION_FACTOR);

	return return_code;
}

/**
 * If a velocity field > MAX_POSSIBLE_VELOCITY, or the velocity accuracy
 * estimate (vAcc) is outside acceptable range, this function will substitute
 * a running average.
 *
 * @param returnNorth - return parameter for the running average North value
 * @param returnEast - return parameter for the running average East value
 * @param returnDown - return parameter for the running average Down value
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
gnss_error_code_t gnss_get_running_average_velocities(GNSS* self)
{
	gnss_error_code_t return_code = GNSS_SUCCESS;
	float substitute_north, substitute_east, substitute_down;

	if (self->total_samples >= self->global_config->samples_per_window) {

		return_code = GNSS_DONE_SAMPLING;

	}
	// avoid a divide by zero error
	else if (self->total_samples == 0) {

		return_code = GNSS_NO_SAMPLES_ERROR;

	}
	// Good to replace value with running average
	else {

		substitute_north = (((float)self->v_north_sum) / MM_PER_METER) /
						((float)self->total_samples);
		substitute_east = (((float)self->v_east_sum) / MM_PER_METER) /
				((float)self->total_samples);
		substitute_down = (((float)self->v_down_sum) / MM_PER_METER) /
				((float)self->total_samples);

		self->GNSS_N_Array[self->total_samples] = substitute_north;
		self->GNSS_E_Array[self->total_samples] = substitute_east;
		self->GNSS_D_Array[self->total_samples] = substitute_down;

		self->total_samples++;
		self->total_samples_averaged++;
	}

	return return_code;
}


/**
 * Send the sleep command to the GNSS unit. This does not remove power, but
 * puts the device in low-power mode.
 *
 * @param self - GNSS struct
 * @param put_to_sleep - true to command sleep, false to wake up
 *
 * @return GNSS_CONFIG_ERROR - command failed
 * 		   GNSS_SUCCESS - command succeeded
 */
gnss_error_code_t gnss_sleep(GNSS* self, bool put_to_sleep)
{
	return stop_start_gnss(self, put_to_sleep);
}

/**
 * Switch the FET controlling power to the GNSS unit.
 *
 * @param self - GNSS struct
 * @param on - true for tuen on, false for turn off
 *
 * @return void
 */
void gnss_on_off(GNSS* self, GPIO_PinState pin_state)
{
	HAL_GPIO_WritePin(GPIOG, GNSS_FET_Pin, pin_state);
}

/**
 * Cycle power to the unit with a short delay
 *
 * @param self - GNSS struct
 *
 * @return void
 */
void gnss_cycle_power(GNSS* self)
{
	self->on_off(self, GPIO_PIN_RESET);
	HAL_Delay(25);
	self->on_off(self, GPIO_PIN_SET);
	HAL_Delay(25);
}

/**
 * Reinitialize the GNSS UART port. Required when switching between Tx and Rx.
 *
 * @param self - GNSS struct
 * @param baud_rate - baud rate to set port to
 */
gnss_error_code_t gnss_reset_uart(GNSS* self, uint16_t baud_rate)
{

	if (HAL_UART_DeInit(self->gnss_uart_handle) != HAL_OK) {
		return GNSS_UART_ERROR;
	}

	self->gnss_uart_handle->Instance = self->gnss_uart_handle->Instance;
	self->gnss_uart_handle->Init.BaudRate = baud_rate;
	self->gnss_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	self->gnss_uart_handle->Init.StopBits = UART_STOPBITS_1;
	self->gnss_uart_handle->Init.Parity = UART_PARITY_NONE;
	self->gnss_uart_handle->Init.Mode = UART_MODE_TX_RX;
	self->gnss_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->gnss_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	self->gnss_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	self->gnss_uart_handle->FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(self->gnss_uart_handle) != HAL_OK)
	{
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

	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);
	LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_4);

	__HAL_DMA_DISABLE_IT(self->gnss_rx_dma_handle, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(self->gnss_tx_dma_handle, DMA_IT_HT);

	return GNSS_SUCCESS;
}

/**
 * Reset and initialize timer.
 *
 * @param self - GNSS struct
 * @param timeout_in_minutes - timeout in minutes
 */
gnss_error_code_t gnss_reset_timer(GNSS* self, uint8_t timeout_in_minutes)
{
	if (HAL_TIM_Base_DeInit(self->minutes_timer) != HAL_OK) {
		return GNSS_TIMER_ERROR;
	}
	// For debugging, not practical to set the timeout to 0
	if (timeout_in_minutes <= 0) {
		self->minutes_timer->Init.Period = 1;
		self->minutes_timer->Init.RepetitionCounter = 0;
	}
	else {
		self->minutes_timer->Init.Period = 59999;
		self->minutes_timer->Init.RepetitionCounter = timeout_in_minutes - 1;
	}

	self->minutes_timer->Instance = GNSS_TIMER_INSTANCE;
	self->minutes_timer->Init.Prescaler = 12000;
	self->minutes_timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	self->minutes_timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	self->minutes_timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(self->minutes_timer) != HAL_OK) {
		return GNSS_TIMER_ERROR;
	}

	__HAL_TIM_CLEAR_FLAG(self->minutes_timer, TIM_FLAG_UPDATE);

	return GNSS_SUCCESS;
}

/**
 * Set the RTC clock.
 *
 * @param GNSS - GNSS struct
 * @param msg_payload - UBX_NAV_PVT message payload containing
 *        time information.
 *
 * @return GNSS_SUCCESS or
 * 		   GNSS_RTC_ERROR - if setting RTC returned an error
 */
gnss_error_code_t gnss_set_rtc(GNSS* self, uint8_t* msg_payload)
{
	gnss_error_code_t return_code = GNSS_SUCCESS;
	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	uint16_t year = (int16_t) get_two_bytes(msg_payload, UBX_NAV_PVT_YEAR_INDEX,
			AS_LITTLE_ENDIAN);
	uint8_t month = msg_payload[UBX_NAV_PVT_MONTH_INDEX];
	uint8_t day = msg_payload[UBX_NAV_PVT_DAY_INDEX];
	uint8_t hour = msg_payload[UBX_NAV_PVT_HOUR_INDEX];
	uint8_t min = msg_payload[UBX_NAV_PVT_MINUTE_INDEX];
	uint8_t sec = msg_payload[UBX_NAV_PVT_SECONDS_INDEX];
	uint8_t time_flags = msg_payload[UBX_NAV_PVT_VALID_FLAGS_INDEX];

	time_flags &= LOWER_4_BITS_MASK;

	if (time_flags != FULLY_RESOLVED_AND_VALID_TIME) {
		return_code = GNSS_TIME_RESOLUTION_ERROR;
		return return_code;
	}

	// Set the date
	rtc_date.Date = day;
	rtc_date.Month = month;
	rtc_date.Year = year - 2000; // RTC takes a 2 digit year
	// We are not using weekday, but the time will be set incorrectly if this field is not initialized
	// Value for WeekDay will not have any effect on time/date
	rtc_date.WeekDay = RTC_WEEKDAY_MONDAY;
	if (HAL_RTC_SetDate(self->rtc_handle, &rtc_date, RTC_FORMAT_BIN) != HAL_OK) {
		return_code = GNSS_RTC_ERROR;
		self->rtc_error = true;
		tx_event_flags_set(self->error_flags, RTC_ERROR, TX_OR);
		return return_code;
	}
	// Set the time
	rtc_time.Hours = hour;
	rtc_time.Minutes = min;
	rtc_time.Seconds = sec;
	rtc_time.SecondFraction = 0;
	if (HAL_RTC_SetTime(self->rtc_handle, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) {
		return_code = GNSS_RTC_ERROR;
		self->rtc_error = true;
		tx_event_flags_set(self->error_flags, RTC_ERROR, TX_OR);
		return return_code;
	}

	self->is_clock_set = true;
	self->rtc_error = false;

	return return_code;
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
		size_t message_size, uint8_t response_class, uint8_t response_id)
{
	int frame_sync_attempts = 0;
	ULONG actual_flags;
	UINT tx_return;
//	uint8_t msg_buf[CONFIG_BUFFER_SIZE];
	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
	const char* buf_start = (const char*)self->config_response_buf;
	const char* buf_end = buf_start;
	size_t buf_length = 600;
    int32_t message_class = 0;
    int32_t message_id = 0;
    int32_t num_payload_bytes = 0;
    uint8_t response_msg_class;
    uint8_t response_msg_id;

	// Start by waiting until the UART is idle
	while (frame_sync_attempts < MAX_FRAME_SYNC_ATTEMPTS) {
		register_watchdog_refresh();
		HAL_UARTEx_ReceiveToIdle_DMA(self->gnss_uart_handle, self->config_response_buf,
				FRAME_SYNC_RX_SIZE);


		tx_return = tx_event_flags_get(self->control_flags, GNSS_CONFIG_RECVD,
				TX_OR_CLEAR, &actual_flags, 2);
		// If the flag is not present, then we are idle
		if (tx_return == TX_NO_EVENTS) {
			HAL_UART_DMAStop(self->gnss_uart_handle);
			self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
			HAL_Delay(1);
			break;
		} else {
			frame_sync_attempts++;
		}
	}

	if (frame_sync_attempts == MAX_FRAME_SYNC_ATTEMPTS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);

		return GNSS_BUSY_ERROR;
	}

	// Start with a blank msg buf -- this will short cycle the for loop
	// below if a message was not received
	memset(self->config_response_buf, 0, CONFIG_BUFFER_SIZE);

	// Send over the configuration settings
	HAL_UART_Transmit_DMA(self->gnss_uart_handle, &(config_array[0]),
			message_size);
	// Make sure the transmission went through completely
	if (tx_event_flags_get(self->control_flags, GNSS_TX_COMPLETE, TX_OR_CLEAR,
					&actual_flags, TX_TIMER_TICKS_PER_SECOND) != TX_SUCCESS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return GNSS_UART_ERROR;

	}

	HAL_UART_Receive_DMA(self->gnss_uart_handle, self->config_response_buf,
			CONFIG_BUFFER_SIZE);

	// Make sure we receive the response within the right amount of time
	if (tx_event_flags_get(self->control_flags, GNSS_CONFIG_RECVD, TX_OR_CLEAR,
			&actual_flags, TX_TIMER_TICKS_PER_SECOND * 2) != TX_SUCCESS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return GNSS_UART_ERROR;
	}

	/* The ack/nak message is guaranteed to be sent within one second, but
	 * we may receive a few navigation messages before the ack is received,
	 * so we have to sift through at least one second worth of messages */
	for (num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
			&message_class, &message_id, payload, sizeof(payload), &buf_end);
			num_payload_bytes > 0;
			num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
			&message_class, &message_id, payload, sizeof(payload), &buf_end))
	{
		if (message_class == 0x05) {
			// Msg class 0x05 is either an ACK or NAK
			if (message_id == 0x00) {
				// This is a NAK msg, the config did not go through properly
				return GNSS_NAK_MESSAGE_RECEIVED;
			}

			if (message_id == 0x01) {
				// This is an ACK message
				response_msg_class = payload[UBX_ACK_ACK_CLSID_INDEX];
				response_msg_id = payload[UBX_ACK_ACK_MSGID_INDEX];

				// Make sure this is an ack for the CFG_VALSET message type
				if (response_msg_class == response_class &&
						response_msg_id == response_id) {
					// This is an acknowledgement of our configuration message
					self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
					return GNSS_SUCCESS;
				}
			}
		}
		// Adjust pointers to continue searching the buffer
		buf_length -= buf_end - buf_start;
		buf_start = buf_end;
	}

	// If we made it here, the ack message was not in the buffer
	HAL_UART_DMAStop(self->gnss_uart_handle);
	self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
	HAL_Delay(10);
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
	ULONG actual_flags;
	// 3rd byte -- 0x08 = Controlled GNSS stop, 0x09 = Controlled GNSS start
	uint8_t message_payload[4] = {0x00, 0x00, (send_stop) ? 0x08 : 0x09, 0x00};
	char cfg_rst_message[sizeof(message_payload) +
						 U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES];

	if ((uUbxProtocolEncode(0x06, 0x04, (const char*)&(message_payload[0]),
			sizeof(message_payload),cfg_rst_message)) < 0)
	{
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		return GNSS_CONFIG_ERROR;
	}

	if ((HAL_UART_Transmit_DMA(self->gnss_uart_handle, (uint8_t*)&(cfg_rst_message[0]),
					sizeof(cfg_rst_message))) != HAL_OK)
	{
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		return GNSS_CONFIG_ERROR;
	}

	// Make sure the transmission went through completely
	if (tx_event_flags_get(self->control_flags, GNSS_TX_COMPLETE, TX_OR_CLEAR,
					&actual_flags, TX_TIMER_TICKS_PER_SECOND) != TX_SUCCESS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return GNSS_UART_ERROR;

	}

	return GNSS_SUCCESS;
}

/**
 *
 *
 * @param self- GNSS struct
 * @param
 */
static void process_frame_sync_messages(GNSS* self, uint8_t* process_buf)
{
	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
	const char* buf_start = (const char*)&(process_buf[0]);
	const char* buf_end = buf_start;

	size_t buf_length = INITIAL_STAGES_BUFFER_SIZE;
	int32_t message_class = 0;
	int32_t message_id = 0;
	int32_t num_payload_bytes = 0;
	// Reset the counters
	self->messages_processed = 0;
	self->number_cycles_without_data = 0;
	self->total_samples = 0;

	// Really gross for loop that processes msgs in each iteration
	for (num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
				 &message_class, &message_id, payload, sizeof(payload), &buf_end);
			num_payload_bytes > 0;
			num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
				 &message_class, &message_id, payload, sizeof(payload), &buf_end))
	{
		// UBX_NAV_PVT payload is 92 bytes, message class is 0x01,
		// message ID is 0x07
		if (num_payload_bytes != UBX_NAV_PVT_PAYLOAD_LENGTH ||
				message_class != UBX_NAV_PVT_MESSAGE_CLASS  ||
				message_id    != UBX_NAV_PVT_MESSAGE_ID)
		{
			self->number_cycles_without_data++;
			buf_length -= buf_end - buf_start;
			buf_start = buf_end;
			continue;
		}

		// need to keep track of how many messages were processed in the buffer
		self->messages_processed++;
		self->number_cycles_without_data = 0;
		self->total_samples++;

		buf_length -= buf_end - buf_start;
		buf_start = buf_end;
	}
}

/**
 *
 *
 * @param self- GNSS struct
 * @param
 */
static gnss_error_code_t enable_high_performance_mode(GNSS* self)
{
	gnss_error_code_t return_code;
	int config_step_attempts = 0;
	uint8_t enable_high_performance_mode[ENABLE_HIGH_PERFORMANCE_SIZE] =
	{0xB5,0x62,0x06,0x41,0x10,0x00,0x03,0x00,0x04,0x1F,
	 0x54,0x5E,0x79,0xBF,0x28,0xEF,0x12,0x05,0xFD,0xFF,
	 0xFF,0xFF,0x8F,0x0D,0xB5,0x62,0x06,0x41,0x1C,0x00,
	 0x04,0x01,0xA4,0x10,0xBD,0x34,0xF9,0x12,0x28,0xEF,
	 0x12,0x05,0x05,0x00,0xA4,0x40,0x00,0xB0,0x71,0x0B,
	 0x0A,0x00,0xA4,0x40,0x00,0xD8,0xB8,0x05,0xDE,0xAE};

	while (config_step_attempts < MAX_CONFIG_STEP_ATTEMPTS) {
		register_watchdog_refresh();
		return_code = query_high_performance_mode(self);

		switch (return_code) {
			case GNSS_NAK_MESSAGE_RECEIVED:
				// Zero out the config response buffer
				memset(self->config_response_buf, 0, CONFIG_BUFFER_SIZE);
				config_step_attempts = 0;

				// Now send over the command to enable high performance mode
				while (config_step_attempts < MAX_CONFIG_STEP_ATTEMPTS) {
					register_watchdog_refresh();
					return_code = send_config(self, &(enable_high_performance_mode[0]),
							ENABLE_HIGH_PERFORMANCE_SIZE, 0x06, 0x41);

					if (return_code != GNSS_SUCCESS) {
						config_step_attempts++;
					} else {
						break;
					}
				}

				if (config_step_attempts == MAX_CONFIG_STEP_ATTEMPTS) {
					HAL_UART_DMAStop(self->gnss_uart_handle);
					self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
					HAL_Delay(10);
					return_code = GNSS_HIGH_PERFORMANCE_ENABLE_ERROR;
					return return_code;
				}

				// Must cycle power before the high performance mode will kick in
				self->cycle_power(self);

				HAL_Delay(10);

				// Zero out the config response buffer
				memset(self->config_response_buf, 0, CONFIG_BUFFER_SIZE);
				config_step_attempts = 0;

				// Now check to see if the changes stuck
				while (config_step_attempts < MAX_CONFIG_STEP_ATTEMPTS) {
					register_watchdog_refresh();
					return_code = query_high_performance_mode(self);

					if (return_code != GNSS_SUCCESS) {
						config_step_attempts++;
					} else {
						break;
					}
				}

				if (config_step_attempts == MAX_CONFIG_STEP_ATTEMPTS) {
					HAL_UART_DMAStop(self->gnss_uart_handle);
					self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
					HAL_Delay(10);
					return_code = GNSS_CONFIG_ERROR;
					return return_code;
				}

				return_code = GNSS_SUCCESS;
				return return_code;

			case GNSS_SUCCESS:
				return return_code;

			case GNSS_UART_ERROR:
				config_step_attempts++;
				break;

			case GNSS_CONFIG_ERROR:
				config_step_attempts++;
				break;

			default:
				return GNSS_UNKNOWN_ERROR;
		}
	}

	if (config_step_attempts == MAX_CONFIG_STEP_ATTEMPTS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return_code = GNSS_CONFIG_ERROR;
		return return_code;
	}

	return GNSS_UNKNOWN_ERROR;
}

/**
 *
 *
 * @param self- GNSS struct
 * @param
 */
static gnss_error_code_t query_high_performance_mode(GNSS* self)
{
	gnss_error_code_t return_code;
	ULONG actual_flags;
	uint8_t msg_buf[CONFIG_BUFFER_SIZE];
	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
	const char* buf_start = (const char*)self->config_response_buf;
	const char* buf_end = buf_start;
	size_t buf_length = 600;
	int32_t message_class = 0;
	int32_t message_id = 0;
	int32_t num_payload_bytes = 0;
	uint8_t high_performance_mode_query[HIGH_PERFORMANCE_QUERY_SIZE] =
	{0xB5,0x62,0x06,0x8B,0x14,0x00,0x00,0x04,0x00,0x00,
	 0x01,0x00,0xA4,0x40,0x03,0x00,0xA4,0x40,0x05,0x00,
	 0xA4,0x40,0x0A,0x00,0xA4,0x40,0x4C,0x15};
	uint8_t high_performance_mode_response[HIGH_PERFORMANCE_RESPONSE_SIZE] =
	{0x01,0x04,0x00,0x00,0x01,0x00,0xA4,0x40,0x00,0xB0,
	 0x71,0x0B,0x03,0x00,0xA4,0x40,0x00,0xB0,0x71,0x0B,
	 0x05,0x00,0xA4,0x40,0x00,0xB0,0x71,0x0B,0x0A,0x00,
	 0xA4,0x40,0x00,0xD8,0xB8,0x05};

	// First, check to see if high performance mode has already been set
	HAL_UART_Transmit_DMA(self->gnss_uart_handle, &(high_performance_mode_query[0]),
			HIGH_PERFORMANCE_QUERY_SIZE);

	if (tx_event_flags_get(self->control_flags, GNSS_TX_COMPLETE, TX_OR_CLEAR,
					&actual_flags, TX_TIMER_TICKS_PER_SECOND) != TX_SUCCESS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return GNSS_UART_ERROR;

	}

	// Zero out the config response buffer
	memset(self->config_response_buf, 0, CONFIG_BUFFER_SIZE);
	memset(&(payload[0]), 0, UBX_NAV_PVT_PAYLOAD_LENGTH);

	// Grab the response (or lack thereof)
	HAL_UART_Receive_DMA(self->gnss_uart_handle, &(self->config_response_buf[0]),
		sizeof(msg_buf));

	if (tx_event_flags_get(self->control_flags, GNSS_CONFIG_RECVD, TX_OR_CLEAR,
			&actual_flags, TX_TIMER_TICKS_PER_SECOND * 2) != TX_SUCCESS) {
		HAL_UART_DMAStop(self->gnss_uart_handle);
		self->reset_uart(self, GNSS_DEFAULT_BAUD_RATE);
		HAL_Delay(10);
		return GNSS_UART_ERROR;
	}

	for (num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
			&message_class, &message_id, payload, sizeof(payload), &buf_end);
			num_payload_bytes > 0;
			num_payload_bytes = uUbxProtocolDecode(buf_start, buf_length,
			&message_class, &message_id, payload, sizeof(payload), &buf_end))
	{
		// If true, this is a NAK message, and High Performance mode is not set
		if (message_class == 0x05 && message_id == 0x00) {
			return_code = GNSS_NAK_MESSAGE_RECEIVED;
			return return_code;
		}
		else if (message_class == 0x06 && message_id == 0x8B) {
			// Need to ensure the response is identical to the expected response
			for (int i = 0; i < HIGH_PERFORMANCE_RESPONSE_SIZE; i++) {

				if (payload[i] != high_performance_mode_response[i]) {
					return_code = GNSS_CONFIG_ERROR;
					return return_code;
				}
			}

			return_code = GNSS_SUCCESS;
			return return_code;
		}
		// Adjust pointers to continue searching the buffer
		buf_length -= buf_end - buf_start;
		buf_start = buf_end;
	}

	return GNSS_CONFIG_ERROR;
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

/**
 * Helper method to generate a timestamp from the RTC.
 *
 * @param self - GNSS struct
 * @return timestamp as uint32_t
 */
static uint32_t get_timestamp(GNSS* self)
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
	timestamp += julian_date_first_of_month * SECONDS_IN_DAY;
	timestamp += (rtc_date.Date - 1) * SECONDS_IN_DAY;
	timestamp += rtc_time.Hours * SECONDS_IN_HOUR;
	timestamp += rtc_time.Minutes * SECONDS_IN_MIN;
	timestamp += rtc_time.Seconds;
	// Not including fractions of a second
	return timestamp;
}

/**
 *
 *
 * @param self- GNSS struct
 * @param
 */
static void reset_struct_fields(GNSS* self)
{
	self->messages_processed = 0;
	self->v_north_sum = 0;
	self->v_east_sum = 0;
	self->v_down_sum = 0;
	self->current_latitude = 0;
	self->current_longitude = 0;
	self->sample_window_start_time = 0;
	self->sample_window_stop_time = 0;
	self->sample_window_freq = 0.0;
	self->total_samples = 0;
	self->total_samples_averaged = 0;
	self->number_cycles_without_data = 0;
	self->current_fix_is_good = false;
	self->all_resolution_stages_complete = false;
	self->is_configured = false;
	self->is_clock_set = false;
	self->rtc_error = false;
	self->all_samples_processed = false;
	self->timer_timeout = false;
}
