/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  ******************************************************************************
  *
  *TODO: update macros to be adjustable by some config.h file
  *TODO: update array size macros once known
  *TODO: write error handler function and call it in:
  	  	  - self test
  	  	  - hard fault
  	  	  - any other condition where bad things happen
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef enum thread_priorities{
 	HIGHEST = 0,
 	VERY_HIGH = 1,
 	HIGH = 2,
 	MID= 3,
 	LOW = 4,
 	LOWEST = 5
 }thread_priorities_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_EXTRA_LARGE_STACK_SIZE 4096
#define THREAD_LARGE_STACK_SIZE 2048
#define THREAD_MEDIUM_STACK_SIZE 1024
#define THREAD_SMALL_STACK_SIZE 512
#define UBX_QUEUE_SIZE 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern DMA_QListTypeDef GNSS_LL_Queue;
// The configuration struct
microSWIFT_configuration configuration;
// The SBD message we'll assemble
sbd_message_type_52 sbd_message;
// The primary byte pool from which all memory is allocated from
TX_BYTE_POOL *byte_pool;
// Our threads
TX_THREAD startup_thread;
TX_THREAD gnss_thread;
TX_THREAD imu_thread;
TX_THREAD ct_thread;
TX_THREAD waves_thread;
TX_THREAD iridium_thread;
TX_THREAD teardown_thread;
// We'll use flags to signal other threads to run/shutdown
TX_EVENT_FLAGS_GROUP thread_flags;
// The data structures for Waves
emxArray_real32_T *north;
emxArray_real32_T *east;
emxArray_real32_T *down;
//float* down;
//float* east;
//float* north;
// The primary DMA buffer for GNSS UBX messages
uint8_t* ubx_DMA_message_buf;
// Buffer for messages ready to process
uint8_t* ubx_message_process_buf;
// Iridium buffers
uint8_t* iridium_message;
uint8_t* iridium_response_message;
uint8_t* iridium_error_message;
// GNSS and Iridium structs
GNSS* gnss;
Iridium* iridium;
// Handles for all the STM32 peripherals
device_handles_t *device_handles;
// Only included if we will be using the IMU
#if IMU_ENABLED
int16_t* IMU_N_Array;
int16_t* IMU_E_Array;
int16_t* IMU_D_Array;
#endif
// Only included if there is a CT sensor
#if CT_ENABLED
CT* ct;
CHAR* ct_data;
ct_samples* samples_buf;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void startup_thread_entry(ULONG thread_input);
void gnss_thread_entry(ULONG thread_input);
void waves_thread_entry(ULONG thread_input);
void iridium_thread_entry(ULONG thread_input);
void teardown_thread_entry(ULONG thread_input);
// callback function to get GNSS DMA started
gnss_error_code_t start_GNSS_UART_DMA(GNSS* gnss_struct_ptr, uint8_t* buffer, size_t buffer_size);

#if IMU_ENABLED
void imu_thread_entry(ULONG thread_input);
#endif

#if CT_ENABLED
void ct_thread_entry(ULONG thread_input);
#endif
// Static helper functions
static void led_sequence(uint8_t sequence);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_ThreadX_MEM_POOL */
	(void)byte_pool;
	CHAR *pointer = TX_NULL;

	//
	// Allocate stack for the startup thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the startup thread. HIGHEST priority level and no preemption possible
	ret = tx_thread_create(&startup_thread, "startup thread", startup_thread_entry, 0, pointer,
			THREAD_EXTRA_LARGE_STACK_SIZE, HIGHEST, HIGHEST, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate stack for the gnss thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the gnss thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&gnss_thread, "gnss thread", gnss_thread_entry, 0, pointer,
		  THREAD_EXTRA_LARGE_STACK_SIZE, VERY_HIGH, VERY_HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate stack for the waves thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the waves thread. MID priority, no preemption-threshold
	ret = tx_thread_create(&waves_thread, "waves thread", waves_thread_entry, 0, pointer,
			THREAD_EXTRA_LARGE_STACK_SIZE, MID, MID, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate stack for the Iridium thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the Iridium thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&iridium_thread, "iridium thread", iridium_thread_entry, 0, pointer,
			THREAD_LARGE_STACK_SIZE, MID, MID, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate stack for the teardown thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_SMALL_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the teardown thread. HIGHEST priority, no preemption-threshold
	ret = tx_thread_create(&teardown_thread, "teardown thread", teardown_thread_entry, 0, pointer,
		  THREAD_SMALL_STACK_SIZE, LOWEST, LOWEST, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Create the event flags we'll use for triggering threads
	ret = tx_event_flags_create(&thread_flags, "thread_flags");
	if (ret != TX_SUCCESS) {
	  return ret;
	}
//	//
//	// The UBX message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_DMA_message_buf, UBX_MESSAGE_SIZE * 2, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The UBX process buffer
	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_message_process_buf, UBX_MESSAGE_SIZE * 2, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The Iridium message array -- add 2 to the size for the checksum
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_message, IRIDIUM_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH,
			TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The Iridium error message payload array
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_error_message, IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH,
				TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The Iridium response message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_response_message, IRIDIUM_MAX_RESPONSE_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The gnss struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &gnss, sizeof(GNSS), TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	//
	// The iridium struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium, sizeof(Iridium), TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
// Only if the IMU will be utilized
#if IMU_ENABLED
	//
	// Allocate stack for the imu thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the imu thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&imu_thread, "imu thread", imu_thread_entry, 0, pointer,
		  THREAD_LARGE_STACK_SIZE, HIGH, HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}

	ret = tx_byte_allocate(byte_pool, (VOID**) &IMU_N_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &IMU_E_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &IMU_D_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}

#endif
// Only is there is a CT sensor present
#if CT_ENABLED
	//
	// Allocate stack for the CT thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_MEDIUM_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the CT thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&ct_thread, "ct thread", ct_thread_entry, 0, pointer,
		  THREAD_MEDIUM_STACK_SIZE, VERY_HIGH, VERY_HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The ct struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &ct, sizeof(CT), TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	// The CT input data buffer array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ct_data, CT_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// The CT samples array
	ret = tx_byte_allocate(byte_pool, (VOID**) &samples_buf, TOTAL_CT_SAMPLES * sizeof(ct_samples),
			TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// The CT samples array
	ret = tx_byte_allocate(byte_pool, (VOID**) &sbd_message, sizeof(sbd_message_type_52),
			TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}

	north = argInit_1xUnbounded_real32_T(&configuration); //get_waves_float_array(&configuration);
	east  = argInit_1xUnbounded_real32_T(&configuration); //get_waves_float_array(&configuration);
	down  = argInit_1xUnbounded_real32_T(&configuration); //get_waves_float_array(&configuration);
#endif
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
  /* USER CODE BEGIN  Before_Kernel_Start */
void MX_ThreadX_Init(device_handles_t *handles)
{
  device_handles = handles;
  configuration.duty_cycle = DUTY_CYCLE;
  configuration.samples_per_window = TOTAL_SAMPLES_PER_WINDOW;
  configuration.iridium_max_transmit_time = IRIDIUM_MAX_TRANSMIT_TIME;
  configuration.gnss_max_acquisition_wait_time = GNSS_MAX_ACQUISITION_WAIT_TIME;
  configuration.gnss_sampling_rate = GNSS_SAMPLING_RATE;
  configuration.total_ct_samples = TOTAL_CT_SAMPLES;
  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
/**
  * @brief  startup_thread_entry
  *         This thread will start all peripherals and do a systems check to
  *         make sure we're good to start the processing cycle
  * @param  ULONG thread_input - unused
  * @retval void
  */
void startup_thread_entry(ULONG thread_input){
	// TODO: Check flash for first-time flag. If present, skip, otherwise run
	// TODO: set event flags to "ready" for all threads
	UINT threadx_return;
	int fail_counter;

	/*TODO: flash:
	 * 			  (1) Read some predetermined page and quadword.
	 * 			  (2) If the contents are 0xFFFFFFFF, it has not been written to
	 * 			  	(a) run this thread in its entirety
	 * 			  	(b) at the end of the thread, write a bit pattern to that
	 * 			  	    page and address. Maybe something like 0xAAAAAAAA.
	 * 			  (3) If the contents are 0xAAAAAAAA, skip this thread.
	 * 			  	(a) run a different, more watered down version that just
	 * 			  	    warms up the GNSS receiver until it is getting good
	 * 			  	    reception
	 */
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// GNSS STARTUP SEQUENCE /////////////////////////////////////////////
	// Initialize GNSS struct
	gnss_init(gnss, &configuration, device_handles->GNSS_uart, device_handles->GNSS_dma_handle,
			&thread_flags, &(ubx_message_process_buf[0]), device_handles->hrtc, north->data,
			east->data, down->data);
	// turn on the GNSS FET
	gnss->on_off(gnss, GPIO_PIN_SET);
	// Send the configuration commands to the GNSS unit.
	fail_counter = 0;
	while (fail_counter < MAX_SELF_TEST_RETRIES) {
		if (gnss->config(gnss) != GNSS_SUCCESS) {
			// Config didn't work, cycle power and try again
			gnss->cycle_power(gnss);
			fail_counter++;
		} else {
			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES){
		HAL_NVIC_SystemReset();
	}

	// Wait until we get a series of good UBX_NAV_PVT messages and are
	// tracking a good number of satellites before moving on
	fail_counter = 0;
	while (fail_counter < MAX_SELF_TEST_RETRIES) {
		if (gnss->self_test(gnss, start_GNSS_UART_DMA, ubx_DMA_message_buf, UBX_MESSAGE_SIZE)
				!= GNSS_SUCCESS) {
			// self_test failed, cycle power and try again
			gnss->cycle_power(gnss);
			fail_counter++;
		} else {
			gnss->is_configured = true;
			break;
		}

	}

	if (fail_counter == MAX_SELF_TEST_RETRIES){
		HAL_NVIC_SystemReset();
	}

	// If we made it here, the self test passed and we're ready to process messages
	threadx_return = tx_event_flags_set(&thread_flags, GNSS_READY, TX_OR);
	if (threadx_return != TX_SUCCESS) {
		HAL_Delay(10);
		tx_event_flags_set(&thread_flags, GNSS_READY, TX_OR);
	}
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////IRIDIUM STARTUP SEQUENCE ///////////////////////////////////////////////
	iridium_init(iridium, &configuration, device_handles->Iridium_uart,
			device_handles->Iridium_rx_dma_handle, device_handles->iridium_timer,
			device_handles->Iridium_tx_dma_handle, &thread_flags,
			device_handles->hrtc,(uint8_t*)iridium_message,
			(uint8_t*)iridium_error_message,
			(uint8_t*)iridium_response_message);
	// See if we can get an ack message from the modem
	if (iridium->self_test(iridium) != IRIDIUM_SUCCESS) {
		tx_event_flags_set(&thread_flags, MODEM_ERROR, TX_OR);
		// TODO: do something here
	}
	// Send the configuration settings to the modem
	if (iridium->config(iridium) != IRIDIUM_SUCCESS) {
		tx_event_flags_set(&thread_flags, MODEM_ERROR, TX_OR);
		// TODO: do something here
	}
	// We'll keep power to the modem but put it to sleep
	iridium->sleep(iridium, GPIO_PIN_RESET);

	// We got an ack and were able to config the Iridium modem
	threadx_return = tx_event_flags_set(&thread_flags, IRIDIUM_READY, TX_OR);
	if (threadx_return != TX_SUCCESS) {
		// TODO: create a "handle_tx_error" function and call it in here
		HAL_Delay(10);
		tx_event_flags_set(&thread_flags, IRIDIUM_READY, TX_OR);
	}

#if IMU_ENABLED
///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// IMU STARTUP SEQUENCE ///////////////////////////////////////////////
#endif

#if CT_ENABLED
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// CT STARTUP SEQUENCE ///////////////////////////////////////////////
	ct_init(ct, &configuration, device_handles->CT_uart, device_handles->CT_dma_handle,
			&thread_flags, ct_data, samples_buf);
	// Make sure we get good data from the CT sensor
	if (ct->self_test(ct, false) != CT_SUCCESS) {
		// TODO: cycle power to the board, do some stuff
		HAL_NVIC_SystemReset();
	}
	// We can turn off the CT sensor for now
	ct->on_off(ct, GPIO_PIN_RESET);

	// We received a good message from the CT sensor
	threadx_return = tx_event_flags_set(&thread_flags, CT_READY, TX_OR);
	if (threadx_return != TX_SUCCESS) {
		// TODO: create a "handle_tx_error" function and call it in here
		HAL_Delay(10);
		tx_event_flags_set(&thread_flags, CT_READY, TX_OR);
	}

#endif
	// We're done, suspend this thread
	tx_thread_suspend(&startup_thread);
}

/**
  * @brief  gnss_thread_entry
  *         Thread that governs the GNSS processing. Note that actuall message processing
  *         happens in interrupt context, so this thread is just acting as the traffic cop.
  *
  * @param  ULONG thread_input - unused
  * @retval void
  */
void gnss_thread_entry(ULONG thread_input){
	ULONG actual_flags;
	uint32_t timeout_start = 0, elapsed_time = 0;
	uint32_t timeout = configuration.gnss_max_acquisition_wait_time * MILLISECONDS_PER_MINUTE;
	// Wait until we get the ready flag
	tx_event_flags_get(&thread_flags, GNSS_READY, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

//	return_code = gnss->resolve_time(gnss);

	while (!gnss->all_samples_processed) {

		timeout_start = HAL_GetTick();
		while (elapsed_time < timeout) {
			elapsed_time = HAL_GetTick() - timeout_start;
		}

		if (!gnss->all_resolution_stages_complete) {
			// TODO: sleep for 1 hour by RTC
			HAL_Delay(100);
		}
		while (!gnss->all_samples_processed);
	}

	while (tx_event_flags_set(&thread_flags, GNSS_DONE, TX_OR) != TX_SUCCESS) {
		HAL_Delay(10);
	}

	tx_thread_suspend(&gnss_thread);
}

#if IMU_ENABLED
/**
  * @brief  imu_thread_entry
  *         Thread that governs the IMU velocity processing and building of
  *         uIMUArray, vIMUArray, zIMUArray arrays.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void imu_thread_entry(ULONG thread_input){
	/*
	 * Currently disabled.
	 * TODO: enable this function
	 */
//	ULONG actual_flags;
//	tx_event_flags_get(&thread_flags, IMU_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);
	tx_event_flags_set(&thread_flags, IMU_DONE, TX_OR);
}
#endif

#if CT_ENABLED
/**
  * @brief  ct_thread_entry
  *         This thread will handle the CT sensor, capture readings, and getting averages..
  *
  * @param  ULONG thread_input - unused
  * @retval void
  */
void ct_thread_entry(ULONG thread_input){
	ULONG actual_flags;
	ct_error_code_t return_code;
	uint32_t ct_parsing_error_counter = 0;

	tx_event_flags_get(&thread_flags, GNSS_DONE, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	// If the CT sensor doesn't respond, set the error flag and quit
	if (ct->self_test(ct, true) != CT_SUCCESS) {
		tx_event_flags_set(&thread_flags, CT_ERROR, TX_OR);
		tx_event_flags_set(&thread_flags, CT_DONE, TX_OR);
		tx_thread_suspend(&ct_thread);
	}

	// Take our samples
	while (ct->total_samples < configuration.total_ct_samples) {
		return_code = ct_parse_sample(ct);
		if (return_code == CT_PARSING_ERROR) {
			// If there are too many parsing errors, then there's something wrong
			if (++ct_parsing_error_counter == 10) {
				tx_event_flags_set(&thread_flags, CT_ERROR, TX_OR);
				tx_event_flags_set(&thread_flags, CT_DONE, TX_OR);
				tx_thread_suspend(&ct_thread);
			}
		}
	}

	// Got our samples, now average them
	return_code = ct->get_averages(ct);
	// Make sure something didn't go terribly wrong
	if (return_code == CT_NOT_ENOUGH_SAMPLES) {
		tx_event_flags_set(&thread_flags, CT_ERROR, TX_OR);
		tx_event_flags_set(&thread_flags, CT_DONE, TX_OR);
		tx_thread_suspend(&ct_thread);
	}

	while (tx_event_flags_set(&thread_flags, CT_DONE, TX_OR) != TX_SUCCESS) {
		HAL_Delay(10);
	}

	while (tx_event_flags_set(&thread_flags, WAVES_READY, TX_OR) != TX_SUCCESS) {
		HAL_Delay(10);
	}

	tx_thread_suspend(&ct_thread);
}
#endif

/**
  * @brief  waves_thread_entry
  *         This thread will run the GPSWaves algorithm.
  *
  * @param  ULONG thread_input - unused
  * @retval void
  */
void waves_thread_entry(ULONG thread_input){

	ULONG actual_flags;
	tx_event_flags_get(&thread_flags, WAVES_READY, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

	// Zero out the sbd message struct
	memset(&sbd_message, 0, sizeof(sbd_message));
	// Function return parameters
	real16_T E[42];
	real16_T Dp;
	real16_T Hs;
	real16_T Tp;
	real16_T b_fmax;
	real16_T b_fmin;
	signed char a1[42];
	signed char a2[42];
	signed char b1[42];
	signed char b2[42];
	unsigned char check[42];

	/* Call the entry-point 'NEDwaves_memlight'. */
	NEDwaves_memlight(north, east, down, gnss->sample_window_freq, &Hs, &Tp, &Dp, E,
					&b_fmin, &b_fmax, a1, b1, a2, b2, check);
	// TODO: create a function to assemble the Iridium payload here, to include
	//       changing the endianess of array E

	emxDestroyArray_real32_T(down);
	emxDestroyArray_real32_T(east);
	emxDestroyArray_real32_T(north);

	sbd_message.legacy_number_7 = '7';
	sbd_message.type = 52;
	sbd_message.port = 0;
	sbd_message.size = 327;
	sbd_message.Hs = Hs;
	sbd_message.Tp = Tp;
	sbd_message.Dp = Dp;
	for (int i = 0; i < 42; i++) {
		sbd_message.E_array[i] = E[i];
	}
	sbd_message.f_min = b_fmin;
	sbd_message.f_max = b_fmax;
	for (int i = 0; i < 42; i++) {
		sbd_message.a1_array[i] = a1[i];
	}
	for (int i = 0; i < 42; i++) {
		sbd_message.b1_array[i] = b1[i];
	}
	for (int i = 0; i < 42; i++) {
		sbd_message.a2_array[i] = a2[i];
	}
	for (int i = 0; i < 42; i++) {
		sbd_message.b2_array[i] = b2[i];
	}
	for (int i = 0; i < 42; i++) {
		sbd_message.cf_array[i] = check[i];
	}
	sbd_message.Lat = (float)gnss->current_latitude;
	sbd_message.Lon = (float)gnss->current_longitude;
	sbd_message.mean_salinity = floatToHalf(ct->averages.salinity);
	sbd_message.mean_temp = floatToHalf(ct->averages.temp);
	sbd_message.mean_voltage = floatToHalf(6.0);


	while (tx_event_flags_set(&thread_flags, WAVES_DONE, TX_OR) != TX_SUCCESS) {
		HAL_Delay(10);
	}

	tx_thread_suspend(&waves_thread);
}

/**
  * @brief  iridium_thread_entry
  *         This thread will handle message sending via Iridium modem.
  *
  * @param  ULONG thread_input - unused
  * @retval void
  */
void iridium_thread_entry(ULONG thread_input){
	ULONG actual_flags;
	ULONG required_flags = GNSS_DONE | WAVES_DONE | IRIDIUM_READY;
	ULONG error_flags = GNSS_ERROR | MODEM_ERROR | MEMORY_ALLOC_ERROR
			| DMA_ERROR | UART_ERROR;

#if CT_ENABLED
	required_flags |= CT_DONE;
	error_flags |= CT_ERROR;
#endif

#if IMU_ENABLED
	required_flags |= IMU_DONE;
	error_flags |= IMU_ERROR;
#endif

//	if (tx_event_flags_get(&thread_flags, error_flags, TX_OR, &actual_flags,
//			TX_TIMER_TICKS_PER_SECOND * ) == TX_SUCCESS) {
//		//TODO: send error message here
//	}

	tx_event_flags_get(&thread_flags, required_flags, TX_AND, &actual_flags, TX_WAIT_FOREVER);

	// The event flags contain an error message, figure it out
	if (actual_flags & error_flags) {
		// TODO: figure out how to send an error message, make sure to store some flag in
		//   	 flash so it only sends once
		iridium->transmit_error_message(iridium, "Sample error message");
	}

	// !!!!!!!!!!!!!!!!!!!
	// testing
	uint8_t test[327] =
		   {0x37,0x34,0x06,0x47,0x01,0xBB,0x40,0x4D,0x44,0x63,0x50,0x07,0x2D,0x7B,0x39,0x38,
			0x41,0x53,0x43,0x56,0x43,0xEC,0x44,0xDA,0x44,0x44,0x41,0xD1,0x40,0x2E,0x40,0xC6,
			0x3E,0xBA,0x3E,0x9D,0x3B,0x6E,0x38,0x3E,0x38,0xA6,0x35,0xB7,0x34,0x43,0x34,0xEC,
			0x35,0xD9,0x37,0x4E,0x34,0x81,0x32,0x81,0x30,0x69,0x30,0xA4,0x30,0xE4,0x2C,0x4B,
			0x2D,0xB9,0x2D,0x36,0x2C,0xC7,0x2B,0x10,0x2C,0x1D,0x2C,0x81,0x2D,0x2F,0x2A,0x4D,
			0x2A,0x6A,0x28,0x1E,0x29,0xF4,0x29,0xE9,0x28,0x75,0x28,0x82,0x24,0x49,0x25,0x00,
			0x21,0xD8,0x37,0xF8,0xF1,0x16,0x02,0xFA,0x07,0x0A,0x04,0x18,0x0D,0x0E,0x07,0x0B,
			0x04,0x07,0x05,0x04,0xF2,0xE4,0xE0,0xE0,0xE6,0xDD,0xE9,0xE9,0xF1,0xDE,0xF0,0xEF,
			0xED,0xFB,0xF3,0xF5,0xEC,0xF9,0xFE,0x07,0x01,0xF8,0x0F,0x00,0xFF,0xF6,0xEB,0x11,
			0x01,0x08,0xFD,0x07,0x00,0x08,0x0C,0x01,0x02,0x0A,0x0F,0xFC,0x01,0xF8,0xEE,0xD2,
			0xD2,0xD7,0xDA,0xE6,0xE5,0xD9,0xF0,0xF0,0xFA,0xEF,0xF9,0xF5,0xEF,0xE7,0xF4,0xF5,
			0xF9,0xFC,0xFE,0xF6,0x03,0xF3,0x03,0xF0,0xD5,0xE8,0xDD,0xD4,0xDF,0xE5,0xDE,0xED,
			0xE9,0xE3,0xE3,0xEA,0xD7,0xE8,0xDC,0xE9,0xDD,0xE7,0xE3,0xF0,0xF0,0x02,0xFD,0xEF,
			0xFA,0xFF,0xEF,0x00,0xF7,0xF1,0x18,0x08,0x07,0x01,0xF4,0x04,0x06,0xF8,0x0D,0x0C,
			0xF6,0x3C,0x30,0x2A,0x25,0x2C,0x2D,0x2F,0x3B,0x3C,0x30,0x35,0x28,0x22,0x2D,0x37,
			0x3E,0x28,0x3C,0x4A,0x4C,0x3E,0x3C,0x2F,0x3D,0x40,0x2C,0x35,0x33,0x2F,0x28,0x2E,
			0x24,0x33,0x28,0x30,0x1C,0x29,0x26,0x05,0x21,0x07,0x14,0x30,0xBB,0xFF,0xFF,0xD8,
			0xF8,0xD5,0x55,0x57,0x58,0x5B,0x5A,0x50,0x27,0x25,0x16,0x11,0x0E,0x0C,0x0B,0x0B,
			0x0C,0x0B,0x0C,0x0C,0x0B,0x0F,0x0F,0x0E,0x0F,0x12,0x15,0x13,0x0F,0x10,0x0B,0x0C,
			0x12,0x13,0x0E,0x07,0x0D,0xF6,0x0B,0xDB,0x41,0xA5,0x11,0xAA,0xC2,0x00,0x00,0x00,
			0x00,0x00,0x00,0x1D,0x64,0xC6,0x4E};

	iridium->current_lat = 47.655357637587834;
	iridium->current_lon = -122.32135545762652;

	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;

	rtc_date.Date = 3;
	rtc_date.Month = RTC_MONTH_APRIL;
	rtc_date.Year = 23;
	rtc_date.WeekDay = 1;

	rtc_time.Hours = 15;
	rtc_time.Minutes = 10;
	rtc_time.Seconds = 15;
	rtc_time.TimeFormat = RTC_HOURFORMAT_24;
	rtc_time.SecondFraction = 0;

	HAL_RTC_SetTime(device_handles->hrtc, &rtc_time, RTC_FORMAT_BCD);
	HAL_RTC_SetDate(device_handles->hrtc, &rtc_date, RTC_FORMAT_BCD);

	iridium->transmit_error_message(iridium, "Date should be: 4/3/2023 \r\n Time should be (roughly): 3:10:15 PM.");
	while(1){HAL_Delay(1000);}
	tx_thread_suspend(&startup_thread);

	memcpy(iridium->message_buffer, test, IRIDIUM_MESSAGE_PAYLOAD_SIZE);
	iridium->transmit_message(iridium);
	tx_thread_suspend(&startup_thread);


}

/**
  * @brief  teardown_thread_entry
  *         This thread will execute when either an error flag is set or all
  *         the done flags are set, indicating we are ready to shutdown until
  *         the next window.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void teardown_thread_entry(ULONG thread_input){
	// TODO: Figure out the right flag combinations to start this thread
	// 	For now, we'll just assume the right combo is that everything is done
	ULONG actual_flags, requested_flags;
	requested_flags =  	GNSS_DONE | IMU_DONE | CT_DONE | IRIDIUM_DONE | WAVES_DONE;
	tx_event_flags_get(&thread_flags, requested_flags, TX_OR, &actual_flags, TX_WAIT_FOREVER);
	UINT status;
	ULONG retreived_flags;
	ULONG done_flags_to_check = GNSS_DONE |
								IMU_DONE |
								CT_DONE |
								IRIDIUM_DONE |
								WAVES_DONE;

	ULONG error_flags_to_check = GNSS_ERROR |
								 IMU_ERROR |
								 CT_ERROR |
								 MODEM_ERROR |
								 MEMORY_ALLOC_ERROR;

	while(1) {
		retreived_flags = 0x0;
		// Start by checking if we get an error flag. The last argument of "1"
		// means we will check this every tick
		status = tx_event_flags_get(&thread_flags,
				error_flags_to_check,
				TX_OR, &retreived_flags, 1);
		// Clear out all bit positions except for the error bits
		retreived_flags &= 0x1F000;
		if ((status == TX_SUCCESS) && (retreived_flags & error_flags_to_check)) {
			// We received an error flag, restart and try again
			HAL_NVIC_SystemReset();
		}
		// Clear out all bit positions except for the done bits
		retreived_flags &= 0xF0;
		// Now we'll check the done flags
		status = tx_event_flags_get(&thread_flags,
						done_flags_to_check,
						TX_AND, &retreived_flags, 1);
		if ((status == TX_SUCCESS) && ~(retreived_flags ^ done_flags_to_check)) {
			// We received all the done bits, break out of the loop so we can
			// shut everything down
			break;
		}
	}
	// If we made it here, we received all the done bits and we're good to
	// dealloc memory and shutdown. We're not going to check the return value
	// because we're going to standby mode regardless, and all RAM will be lost.
//	tx_byte_release(&startup_thread);
//	tx_byte_release(&gnss_thread);
//	tx_byte_release(&imu_thread);
//	tx_byte_release(&ct_thread);
//	tx_byte_release(&waves_thread);
//	tx_byte_release(&iridium_thread);
//	tx_byte_release(&teardown_thread);
//	tx_byte_release(&thread_flags);
//	tx_byte_release(&uGNSSArray);
//	tx_byte_release(&vGNSSArray);
//	tx_byte_release(&zGNSSArray);
//	tx_byte_release(&uIMUArray);
//	tx_byte_release(&vIMUArray);
//	tx_byte_release(&zIMUArray);
//	tx_byte_release(&uWavesArray);
//	tx_byte_release(&vWavesArray);
//	tx_byte_release(&zWavesArray);
//	tx_byte_release(&wavesTempCopyArray);
//	tx_byte_release(&ubx_DMA_message_buf);
//	tx_byte_release(&ct_data);
//	tx_byte_release(&iridium_message);

	// TODO: figure out how to go into standby mode
	// This is just a placeholder for development/debugging purposes
	HAL_NVIC_SystemReset();
}

// TODO: Register callbacks with each respective element
/**
  * @brief  UART ISR callback
  *
  * @param  UART_HandleTypeDef *huart - pointer to the UART handle

  * @retval void
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	static uint8_t* read_ptr = &(gnss->ubx_process_buf[0]);
//	static write_ptr = &(gnss->ubx_process_buf[0]);
	HAL_StatusTypeDef HAL_return;
	if (huart->Instance == gnss->gnss_uart_handle->Instance) {
		if (!gnss->is_configured) {

			tx_event_flags_set(&thread_flags, GNSS_CONFIG_RECVD, TX_NO_WAIT);

		} else {
			memcpy(&(gnss->ubx_process_buf[0]), &(ubx_DMA_message_buf[0]), UBX_MESSAGE_SIZE);
			gnss->process_message(gnss);
		}
	}

	// CT sensor
	else if (huart->Instance == ct->ct_uart_handle->Instance) {
		tx_event_flags_set(&thread_flags, CT_MSG_RECVD, TX_OR);
	}

	// Iridium modem
	else if (huart->Instance == iridium->iridium_uart_handle->Instance) {
		tx_event_flags_set(&thread_flags, IRIDIUM_MSG_RECVD, TX_OR);
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// High frequency, low overhead ISR, no need to save/restore context
//	_tx_thread_context_save();

	if (htim->Instance == TIM16) {
		HAL_IncTick();
	}
	else if (htim->Instance == TIM17) {
		iridium->timer_timeout = true;
	}

//	_tx_thread_context_restore();
}

/**
  * @brief  Static function to flash a sequence of onboard LEDs to indicate
  * success or failure of self-test.
  *
  * @param  sequence: 	INITIAL_LED_SEQUENCE
  * 					TEST_PASSED_LED_SEQUENCE
  * 					TEST_NON_CIRTICAL_FAULT_LED_SEQUENCE
  * 					TEST_CRITICAL_FAULT_LED_SEQUENCE
  *
  * @retval Void
  */
static void led_sequence(led_sequence_t sequence)
{
	switch (sequence) {
		case INITIAL_LED_SEQUENCE:
			for (int i = 0; i < 5; i++){
				HAL_GPIO_WritePin(GPIOG, LED_RED_Pin, GPIO_PIN_SET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_SET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(GPIOG, LED_RED_Pin, GPIO_PIN_RESET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_RESET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				HAL_Delay(250);
			}
			break;

		case TEST_PASSED_LED_SEQUENCE:
			for (int i = 0; i < 10; i++){
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				HAL_Delay(500);
			}
			break;

		case TEST_NON_CIRTICAL_FAULT_LED_SEQUENCE:
			for (int i = 0; i < 10; i++){
				HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_RESET);
				HAL_Delay(500);
			}
			break;

		case TEST_CRITICAL_FAULT_LED_SEQUENCE:
			for (int i = 0; i < 10; i++){
				HAL_GPIO_WritePin(GPIOG, LED_RED_Pin, GPIO_PIN_SET);
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOG, LED_RED_Pin, GPIO_PIN_RESET);
				HAL_Delay(500);
			}
			break;

		default:
			break;
	}
}

/**
  * @brief  callback function to start GNSS UART DMA reception. Passed to GNSS->self_test
  *
  * @param  uart_handle - handle for uart port
  * 		buffer - buffer to store UBX messages in
  * 		buffer_size - capacity of the bufer
  *
  * @retval GNSS_SUCCESS or
  * 	    GNS_UART_ERROR
  */
gnss_error_code_t start_GNSS_UART_DMA(GNSS* gnss_struct_ptr, uint8_t* buffer, size_t msg_size)
{
	gnss_error_code_t return_code = GNSS_SUCCESS;
	HAL_StatusTypeDef hal_return_code;

	gnss->reset_uart(gnss, GNSS_DEFAULT_BAUD_RATE);

	memset(&(buffer[0]), 0, UBX_MESSAGE_SIZE * 2);

	HAL_UART_DMAStop(gnss_struct_ptr->gnss_uart_handle);

	hal_return_code = MX_GNSS_LL_Queue_Config();

	if (hal_return_code != HAL_OK) {
		return_code = GNSS_UART_ERROR;
	}

	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;

	if (HAL_DMAEx_List_Init(gnss_struct_ptr->gnss_dma_handle) != HAL_OK)
	{
		return_code = GNSS_UART_ERROR;
	}

	__HAL_LINKDMA(gnss_struct_ptr->gnss_uart_handle, hdmarx, *gnss_struct_ptr->gnss_dma_handle);

	hal_return_code = HAL_DMAEx_List_LinkQ(gnss_struct_ptr->gnss_dma_handle, &GNSS_LL_Queue);
	if (hal_return_code != HAL_OK) {
		return_code = GNSS_UART_ERROR;
	}

	hal_return_code = HAL_UART_Receive_DMA(gnss_struct_ptr->gnss_uart_handle,
			(uint8_t*)&(buffer[0]), msg_size);
		//  No need for the half-transfer complete interrupt, so disable it
	__HAL_DMA_DISABLE_IT(gnss->gnss_dma_handle, DMA_IT_HT);

	if (hal_return_code != HAL_OK) {
		return_code = GNSS_UART_ERROR;
	}

	return return_code;
}

/* USER CODE END 1 */
