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
// The configuration struct
microSWIFT_configuration configuration;
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
// The UBX message queue, fed by UART via DMA, processed by gnss
TX_QUEUE ubx_queue;
// The queue for CT sensor measurements
TX_QUEUE ct_queue;
// We'll use flags to signal other threads to run/shutdown
TX_EVENT_FLAGS_GROUP thread_flags;
// All our data to store/ process
int16_t* GNSS_N_Array;
int16_t* GNSS_E_Array;
int16_t* GNSS_D_Array;
volatile float* waves_N_Array;
volatile float* waves_E_Array;
volatile float* waves_D_Array;
CHAR ubx_DMA_message_buf[UBX_BUFFER_SIZE];
// queue messages to hold gnss data
char queue_message_1[UBX_BUFFER_SIZE];
char queue_message_2[UBX_BUFFER_SIZE];
char queue_message_3[UBX_BUFFER_SIZE];
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
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void startup_thread_entry(ULONG thread_input);
void gnss_thread_entry(ULONG thread_input);
void waves_thread_entry(ULONG thread_input);
void iridium_thread_entry(ULONG thread_input);
void teardown_thread_entry(ULONG thread_input);

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
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the startup thread. HIGHEST priority level and no preemption possible
	ret = tx_thread_create(&startup_thread, "startup thread", startup_thread_entry, 0, pointer,
			THREAD_LARGE_STACK_SIZE, HIGHEST, HIGHEST, TX_NO_TIME_SLICE, TX_AUTO_START);
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
	//
	// Create our UBX message queue
	ret = tx_queue_create(&ubx_queue, "ubx queue", TX_1_ULONG, pointer, (UBX_QUEUE_SIZE * sizeof(void*)));
	if (ret != TX_SUCCESS) {
		return ret;
	}
	//
	// create a memory pool for waves algorithm
	ret = memory_pool_init(pointer, 0x927C0); // 600,000 bytes (32 byte aligned)
	if (ret == -1) {
		return ret;
	}
	//
	// Allocate bytes for the sensor derived arrays
	ret = tx_byte_allocate(byte_pool, (VOID**) &GNSS_N_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &GNSS_E_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &GNSS_D_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate bytes for the GPSWaves processing arrays
	ret = tx_byte_allocate(byte_pool, (VOID**) &waves_N_Array, WAVES_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &waves_E_Array, WAVES_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &waves_D_Array, WAVES_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The UBX message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_DMA_message_buf, UBX_BUFFER_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The message queue buffers
	ret = tx_byte_allocate(byte_pool, (VOID**) &queue_message_1, UBX_BUFFER_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &queue_message_2, UBX_BUFFER_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	ret = tx_byte_allocate(byte_pool, (VOID**) &queue_message_3, UBX_BUFFER_SIZE, TX_NO_WAIT);
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
	// The CT data array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ct_data, CT_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
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
void MX_ThreadX_Init(device_handles_t *handles)
{
  /* USER CODE BEGIN  Before_Kernel_Start */
  device_handles = handles;
  configuration.duty_cycle = DUTY_CYCLE;
  configuration.samples_per_window = TOTAL_SAMPLES_PER_WINDOW;
  configuration.iridium_max_transmit_time = IRIDIUM_MAX_TRANSMIT_TIME;
  configuration.gnss_sampling_rate = GNSS_SAMPLING_RATE;
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
	int fail_counter = 0;

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
	led_sequence(INITIAL_LED_SEQUENCE);
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// GNSS STARTUP SEQUENCE /////////////////////////////////////////////
	// Initialize GNSS struct
	gnss_init(gnss, &configuration, device_handles->GNSS_uart, device_handles->GNSS_dma_handle,
			&thread_flags, &ubx_queue, device_handles->hrtc, GNSS_N_Array, GNSS_E_Array, GNSS_D_Array);
	// turn on the GNSS FET
	gnss->on_off(gnss, true);
	// Send the configuration commands to the GNSS unit.
	if (gnss->config(gnss) != GNSS_SUCCESS) {
		// TODO: cycle power to the board, do some stuff
		HAL_NVIC_SystemReset();
	}

	// Wait until we get a series of good UBX_NAV_PVT messages and are
	// tracking a good number of satellites before moving on
	if (gnss->self_test(gnss) != GNSS_SUCCESS) {
		// TODO: cycle power to the board, do some stuff
		HAL_NVIC_SystemReset();
	}

	// Start DMA reception. Must happen here in threadx land because
	// the destination array is not within GNSS access
	fail_counter = 0;
	while(fail_counter++ < 10) {
		if (HAL_UART_Receive_DMA(gnss->gnss_uart_handle,
				(uint8_t*)&(ubx_DMA_message_buf[0]), UBX_BUFFER_SIZE) == HAL_OK) {
			//  No need for the half-transfer complete interrupt, so disable it
			__HAL_DMA_DISABLE_IT(gnss->gnss_dma_handle, DMA_IT_HT);
			break;
		}
		if (gnss->reset_gnss_uart(gnss, 9600) != GNSS_SUCCESS){
			continue;
		}
	}
	// If we made it here, the self test passed and we're ready to process messages
	threadx_return = tx_event_flags_set(&thread_flags, GNSS_READY, TX_OR);
	if (threadx_return != TX_SUCCESS) {
		// TODO: create a "handle_tx_error" function and call it in here
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
	// Turn on the Iridium FET and set the sleep pin to off
	iridium->on_off(iridium, true);
	iridium->sleep(iridium, false);
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
	ct_init(ct, device_handles->CT_uart, device_handles->CT_dma_handle,
			device_handles->millis_timer, &thread_flags, ct_data);
	// Turn on the CT FET
	ct->on_off(ct, true);
	// Make sure we get good data from the CT sensor
	if (ct->self_test(ct) != CT_SUCCESS) {
		// TODO: cycle power to the board, do some stuff
		HAL_NVIC_SystemReset();
	}
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
  *         Thread that governs the GNSS message processing and building of
  *         uGNSSArray, vGNSSArray, zGNSSArray arrays.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void gnss_thread_entry(ULONG thread_input){
	UINT threadx_return;
	ULONG actual_flags;
	// Wait until we get the ready flag
	tx_event_flags_get(&thread_flags, GNSS_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	while(gnss->total_samples < TOTAL_SAMPLES_PER_WINDOW){
		// TODO: add a synchronization step in here that checks that we
		//       processed 10 messages in the last go. A simple prime number
		//       delay will do it
		// TODO: check queue full flag and do something?
		gnss->gnss_process_message(gnss);
	}
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
  *         This thread will handle the CT sensor, capture readings, and store
  *         in ct_data.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void ct_thread_entry(ULONG thread_input){
	ULONG actual_flags;
	ct_error_code_t result;
	// TODO: make sure this works with the GNSS_DONE flag
	tx_event_flags_get(&thread_flags, GNSS_DONE, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	while (ct->total_samples < REQUIRED_CT_SAMPLES) {
		result = ct_parse_sample(ct);
		if (result == CT_PARSING_ERROR) {
			// TODO: do something
		}
	}

	// got our samples, now average them
	result = ct->get_averages(ct);
	if (result == CT_NOT_ENOUGH_SAMPLES) {
		// TODO: do something
	}
}
#endif

/**
  * @brief  waves_thread_entry
  *         This thread will run the GPSWaves algorithm.
  *         // TODO: update comments here
  * @param  ULONG thread_input - unused
  * @retval void
  */
void waves_thread_entry(ULONG thread_input){

	ULONG actual_flags;
	tx_event_flags_get(&thread_flags, WAVES_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	emxArray_real32_T *down;
	emxArray_real32_T *east;
	emxArray_real32_T *north;
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
	/* Initialize function 'NEDwaves_memlight' input arguments. */

	/* TODO: remove the testing functions below and replace with the
	 * init functions with real data -- example below
	 *
	 * north = argInit_1xUnbounded_real32_t(GNSS_N_Array);
	 * */

	/* Initialize function input argument 'north'. */
	north = argInit_1xUnbounded_real32_T_down();
	/* Initialize function input argument 'east'. */
	east = argInit_1xUnbounded_real32_T_north_east();
	/* Initialize function input argument 'down'. */
	down = argInit_1xUnbounded_real32_T_north_east();
	/* Call the entry-point 'NEDwaves_memlight'. */
	NEDwaves_memlight(north, east, down, argInit_real_T(), &Hs, &Tp, &Dp, E,
					&b_fmin, &b_fmax, a1, b1, a2, b2, check);
	emxDestroyArray_real32_T(down);
	emxDestroyArray_real32_T(east);
	emxDestroyArray_real32_T(north);
}

/**
  * @brief  iridium_thread_entry
  *         This thread will handle message sending via Iridium modem. The
  *         buffer iridium_message is provided for message storage.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void iridium_thread_entry(ULONG thread_input){
	ULONG actual_flags;
	ULONG required_flags = GNSS_DONE | CT_DONE | WAVES_DONE | IRIDIUM_READY;
	ULONG error_flags = GNSS_ERROR | CT_ERROR | MODEM_ERROR | MEMORY_ALLOC_ERROR
			| DMA_ERROR | UART_ERROR;

#if IMU_ENABLED
	required_flags |= IMU_DONE;
	error_flags |= IMU_ERROR;
#endif

	tx_event_flags_get(&thread_flags, required_flags, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	// The event flags contain an error message, figure it out
	if (actual_flags & error_flags) {
		// TODO: figure out how to send an error message, make sure to store some flag in
		//   	 flash so it only sends once
		iridium->transmit_error_message(iridium, "Sample error message");
	}

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


	// !!!!!!!!!!!!!!!!!!!
	// testing
	iridium->current_lat = 47.655357637587834;
	iridium->current_lon = -122.32135545762652;

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
	ULONG done_flags_to_check = GNSS_DONE &
								IMU_DONE &
								CT_DONE &
								IRIDIUM_DONE &
								WAVES_DONE;
	ULONG error_flags_to_check = GNSS_ERROR &
								 IMU_ERROR &
								 CT_ERROR &
								 MODEM_ERROR &
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
	// Save the thread context
	_tx_thread_context_save();
	static uint32_t msg_counter = 0;
	// Need to make sure this is being called by USART3 (the GNSS UART port)
	if (huart->Instance == USART3) {
		if (!gnss->is_configured) {
			// Set the CONFIG_RECEIVED flag
			tx_event_flags_set(&thread_flags, GNSS_CONFIG_RECVD, TX_NO_WAIT);
			// GNSS has not yet been configured, the last DMA receive was for
			// the acknowledgment message, no need to restart DMA transfer
			_tx_thread_context_restore();
			return;
		} else {
			HAL_StatusTypeDef HAL_return;
			ULONG num_msgs_enqueued, available_space;
			// get info on the number of enqueued messages and available space
			tx_queue_info_get(&ubx_queue, TX_NULL, &num_msgs_enqueued,
					&available_space, TX_NULL, TX_NULL, TX_NULL);
			if (num_msgs_enqueued == 3) {
				// Queue is full, need to signal this condition and exit
				tx_event_flags_set(&thread_flags, UBX_QUEUE_FULL, TX_OR);
				_tx_thread_context_restore();
				return;
			}

			CHAR* current_msg;
			msg_counter = msg_counter % 3;
			// Find the right queue message pointer to assign to
			switch(msg_counter){
			case 0:
				current_msg = &(queue_message_1[0]);
				break;
			case 1:
				current_msg = &(queue_message_2[0]);
				break;
			case 2:
				current_msg = &(queue_message_3[0]);
				break;
			default:
				// TODO: figure out this error condition
				current_msg = &(queue_message_3[0]);
				break;
			}
			msg_counter++;

			memcpy(current_msg, ubx_DMA_message_buf, UBX_BUFFER_SIZE);
			tx_queue_send(&ubx_queue, &current_msg, TX_NO_WAIT);

			// Reinitialize the UART port and restart DMA receive
			HAL_return = HAL_UART_Init(gnss->gnss_uart_handle);
			if (HAL_return != HAL_OK) {
				// Something went wrong with reinitializing UART
				tx_event_flags_set(&thread_flags, UART_ERROR, TX_OR);
			}
			LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);
			HAL_return = HAL_UART_Receive_DMA(gnss->gnss_uart_handle,
					(uint8_t*)&(ubx_DMA_message_buf[0]), UBX_BUFFER_SIZE);
			if (HAL_return != HAL_OK) {
				// Something went wrong with restarting DMA
				tx_event_flags_set(&thread_flags, DMA_ERROR, TX_OR);
			}
		}
	}

	// CT sensor
	else if (huart->Instance == UART4) {
		tx_event_flags_set(&thread_flags, CT_MSG_RECVD, TX_OR);
	}

	// Iridium modem
	else if (huart->Instance == UART5) {
		tx_event_flags_set(&thread_flags, IRIDIUM_MSG_RECVD, TX_OR);
	}
	// Restore the thread context
	_tx_thread_context_restore();
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
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  if (htim->Instance == TIM17) {
	iridium->timer_timeout = true;
  }
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

/* USER CODE END 1 */
