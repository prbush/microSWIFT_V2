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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Our DMA linked list queue needed to use circular mode with GNSS UART DMA
extern DMA_QListTypeDef GNSS_LL_Queue;
// The configuration struct
microSWIFT_configuration configuration;
// The SBD message we'll assemble
sbd_message_type_52 sbd_message;
// The primary byte pool from which all memory is allocated from
TX_BYTE_POOL *byte_pool;
// Our threads
TX_THREAD watchdog_thread;
TX_THREAD startup_thread;
TX_THREAD gnss_thread;
TX_THREAD imu_thread;
TX_THREAD waves_thread;
TX_THREAD iridium_thread;
TX_THREAD end_of_cycle_thread;
// We'll use flags to dictate control flow between the threads
TX_EVENT_FLAGS_GROUP thread_control_flags;
// Flags for errors
TX_EVENT_FLAGS_GROUP error_flags;
// Our watchdog refresh semaphore
TX_SEMAPHORE watchdog_semaphore;
// The data structures for Waves
emxArray_real32_T *north;
emxArray_real32_T *east;
emxArray_real32_T *down;
// The primary DMA buffer for GNSS UBX messages
uint8_t* ubx_DMA_message_buf;
// Buffer for messages ready to process
uint8_t* ubx_message_process_buf;
// The configuration response buffer for GNSS
uint8_t* gnss_config_response_buf;
// Iridium buffers
uint8_t* iridium_response_message;
uint8_t* iridium_error_message;
uint32_t* adc_buf;
// Messages that failed to send are stored here
//extern __attribute__((section("NO_INIT"), zero_init))Iridium_message_storage* sbd_message_queue;
extern Iridium_message_storage sbd_message_queue;
// Structs
GNSS* gnss;
Iridium* iridium;
RF_Switch* rf_switch;
Battery* battery;
// Handles for all the STM32 peripherals
device_handles_t *device_handles;
// The watchdog hour elapsed timer flag
bool watchdog_hour_timer_elapsed = false;
// Only included if we will be using the IMU
#if IMU_ENABLED
int16_t* IMU_N_Array;
int16_t* IMU_E_Array;
int16_t* IMU_D_Array;
#endif
// Only included if there is a CT sensor
#if CT_ENABLED
TX_THREAD ct_thread;
CT* ct;
CHAR* ct_data;
ct_samples* samples_buf;
#endif


#define WAVES_MEM_POOL_SIZE 650000
__ALIGN_BEGIN UCHAR waves_byte_pool_buffer[WAVES_MEM_POOL_SIZE] __ALIGN_END;
TX_BYTE_POOL waves_byte_pool;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void SystemClock_Config(void);
// Static functions
static self_test_status_t initial_power_on_self_test(void);
static void led_sequence(uint8_t sequence);
static void jump_to_end_of_window(ULONG error_bits_to_set);
static void send_error_message(ULONG error_flags);
#if CT_ENABLED
static void jump_to_waves(void);
#endif
// GNSS DMA circular mode callback
gnss_error_code_t start_GNSS_UART_DMA(GNSS* gnss_struct_ptr, uint8_t* buffer, size_t buffer_size);
// Externally visible functions
void shut_it_all_down(void);
void register_watchdog_refresh(void);



#if IMU_ENABLED
void imu_thread_entry(ULONG thread_input);
#endif

#if CT_ENABLED
void ct_thread_entry(ULONG thread_input);
#endif
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
	byte_pool = memory_ptr;
#if WATCHDOG_ENABLED
	//
	// Create the watchdog refresh semaphore
	ret = tx_semaphore_create(&watchdog_semaphore, "watchdog semaphore", 0);
	//
	// Allocate stack for the watchdog thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_SMALL_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the watchdog thread. HIGHEST priority level and no preemption possible, auto-start
	ret = tx_thread_create(&watchdog_thread, "watchdog thread", watchdog_thread_entry, 0, pointer,
			THREAD_SMALL_STACK_SIZE, HIGHEST, HIGHEST, TX_NO_TIME_SLICE, TX_AUTO_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
#endif
	//
	// Allocate stack for the startup thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the startup thread. HIGH priority level and no preemption possible
	ret = tx_thread_create(&startup_thread, "startup thread", startup_thread_entry, 0, pointer,
			THREAD_EXTRA_LARGE_STACK_SIZE, HIGH, HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
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
		  THREAD_EXTRA_LARGE_STACK_SIZE, MID, MID, TX_NO_TIME_SLICE, TX_DONT_START);
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
			THREAD_EXTRA_LARGE_STACK_SIZE, MID, MID, TX_NO_TIME_SLICE, TX_DONT_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate stack for the Iridium thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the Iridium thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&iridium_thread, "iridium thread", iridium_thread_entry, 0, pointer,
			THREAD_EXTRA_LARGE_STACK_SIZE, MID, MID, TX_NO_TIME_SLICE, TX_DONT_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Allocate stack for the teardown thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the end of cycle thread. Priority and preemption is low to allow no preemption
	ret = tx_thread_create(&end_of_cycle_thread, "end of cycle thread", end_of_cycle_thread_entry,
			0, pointer, THREAD_EXTRA_LARGE_STACK_SIZE, VERY_HIGH, VERY_HIGH, TX_NO_TIME_SLICE, TX_DONT_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// Create the event flags we'll use for triggering threads
	ret = tx_event_flags_create(&thread_control_flags, "thread flags");
	if (ret != TX_SUCCESS) {
	  return ret;
	}
	//
	// Create the error flags we'll use for tracking errors
	ret = tx_event_flags_create(&error_flags, "error flags");
	if (ret != TX_SUCCESS) {
	  return ret;
	}
	//
	// The rf switch struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &rf_switch, sizeof(RF_Switch) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	//
	// The UBX message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_DMA_message_buf, (UBX_MESSAGE_SIZE * 2) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The UBX process buffer
	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_message_process_buf, (UBX_MESSAGE_SIZE * 2) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The GNSS config response buffer
	ret = tx_byte_allocate(byte_pool, (VOID**) &gnss_config_response_buf, CONFIG_BUFFER_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The gnss struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &gnss, sizeof(GNSS) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	//
	// The Iridium error message payload array
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_error_message, IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + 100
			+ IRIDIUM_CHECKSUM_LENGTH, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The Iridium response message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_response_message, IRIDIUM_MAX_RESPONSE_SIZE + 100,
			TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The iridium struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium, sizeof(Iridium) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	//
	// The current window SBD message
	ret = tx_byte_allocate(byte_pool, (VOID**) &sbd_message, sizeof(sbd_message_type_52) + 100,
			TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The battery struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &battery, sizeof(Battery) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	//
	// The battery adc buffer
	ret = tx_byte_allocate(byte_pool, (VOID**) &adc_buf, sizeof(uint32_t) * NUMBER_OF_ADC_SAMPLES, TX_NO_WAIT);
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
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the CT thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&ct_thread, "ct thread", ct_thread_entry, 0, pointer,
			THREAD_EXTRA_LARGE_STACK_SIZE, HIGH, HIGH, TX_NO_TIME_SLICE, TX_DONT_START);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	//
	// The ct struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &ct, sizeof(CT) + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
	// The CT input data buffer array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ct_data, CT_DATA_ARRAY_SIZE + 100, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// The CT samples array
	ret = tx_byte_allocate(byte_pool, (VOID**) &samples_buf, TOTAL_CT_SAMPLES * sizeof(ct_samples) + 100,
			TX_NO_WAIT);
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
  /* USER CODE BEGIN  Before_Kernel_Start */
void MX_ThreadX_Init(device_handles_t *handles)
{
  device_handles = handles;
  configuration.samples_per_window = TOTAL_SAMPLES_PER_WINDOW;
  configuration.iridium_max_transmit_time = IRIDIUM_MAX_TRANSMIT_TIME;
  configuration.gnss_max_acquisition_wait_time = GNSS_MAX_ACQUISITION_WAIT_TIME;
  configuration.gnss_sampling_rate = GNSS_SAMPLING_RATE;
  configuration.gnss_high_performance_mode = GNSS_HIGH_PERFORMANCE_MODE_ENABLED;
  configuration.total_ct_samples = TOTAL_CT_SAMPLES;

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Watchdog thread entry
  *         This thread will wait for the watchdog semaphore to be put before refreshing
  *         the IWDG.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void watchdog_thread_entry(ULONG thread_input){
	while(1) {
		tx_semaphore_get(&watchdog_semaphore, TX_WAIT_FOREVER);
		HAL_IWDG_Refresh(device_handles->watchdog_handle);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
	}
}

/**
  * @brief  Startup thread entry
  *         This thread will start all peripherals and do a systems check to
  *         make sure we're good to start the processing cycle
  * @param  ULONG thread_input - unused
  * @retval void
  */
void startup_thread_entry(ULONG thread_input){
	self_test_status_t self_test_status;
	ULONG actual_flags = 0;
	UINT tx_return;
	int fail_counter = 0;

	register_watchdog_refresh();

	// Zero out the sbd message struct
	memset(&sbd_message, 0, sizeof(sbd_message));

	// Set the watchdog reset or software reset flags
	if (device_handles->reset_reason & RCC_RESET_FLAG_IWDG){
		tx_event_flags_set(&error_flags, WATCHDOG_RESET, TX_OR);
	}

	if (device_handles->reset_reason & RCC_RESET_FLAG_SW){
		tx_event_flags_set(&error_flags, SOFTWARE_RESET, TX_OR);
	}

	waves_memory_pool_init(&waves_byte_pool);

	if (waves_memory_pool_create(&(waves_byte_pool_buffer[0]), WAVES_MEM_POOL_SIZE) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	// These structs are being allocated to the waves_byte_pool, and have no overlap with tx_app_byte_pool
	// Each struct has a float array written to by GNSS. These are freed after processing in waves thread.
	north = argInit_1xUnbounded_real32_T(&configuration);
	east  = argInit_1xUnbounded_real32_T(&configuration);
	down  = argInit_1xUnbounded_real32_T(&configuration);

	// Initialize the structs
	gnss_init(gnss, &configuration, device_handles->GNSS_uart, device_handles->GNSS_rx_dma_handle,
			device_handles->GNSS_tx_dma_handle, &thread_control_flags, &error_flags,
			device_handles->gnss_timer, &(ubx_message_process_buf[0]), &(gnss_config_response_buf[0]),
			device_handles->hrtc, north->data, east->data, down->data);

	iridium_init(iridium, &configuration, device_handles->Iridium_uart,
					device_handles->Iridium_rx_dma_handle, device_handles->iridium_timer,
					device_handles->Iridium_tx_dma_handle, &thread_control_flags, &error_flags,
					device_handles->hrtc, &sbd_message, iridium_error_message,
					iridium_response_message, &sbd_message_queue);

#if CT_ENABLED
	ct_init(ct, &configuration, device_handles->CT_uart, device_handles->CT_dma_handle,
					&thread_control_flags, &error_flags, ct_data, samples_buf);
#endif
	rf_switch_init(rf_switch);

	battery_init(battery, device_handles->battery_adc, &thread_control_flags, &error_flags);

	tx_return = tx_event_flags_get(&thread_control_flags, FULL_CYCLE_COMPLETE, TX_OR_CLEAR,
			&actual_flags, TX_NO_WAIT);
	// If this is a subsequent window, just setup the GNSS, skip the rest
	if (tx_return == TX_SUCCESS) {

		register_watchdog_refresh();

		HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);

		// Reset all threads
		tx_return = tx_thread_reset(&gnss_thread);
		if (tx_return == TX_NOT_DONE){
			tx_thread_terminate(&gnss_thread);
			tx_thread_reset(&gnss_thread);
		}
#if CT_ENABLED
		tx_return = tx_thread_reset(&ct_thread);
		if (tx_return == TX_NOT_DONE){
			tx_thread_terminate(&ct_thread);
			tx_thread_reset(&ct_thread);
		}
#endif
		tx_return = tx_thread_reset(&waves_thread);
		if (tx_return == TX_NOT_DONE){
			tx_thread_terminate(&waves_thread);
			tx_thread_reset(&waves_thread);
		}
		tx_return = tx_thread_reset(&iridium_thread);
		if (tx_return == TX_NOT_DONE){
			tx_thread_terminate(&iridium_thread);
			tx_thread_reset(&iridium_thread);
		}
		tx_return = tx_thread_reset(&end_of_cycle_thread);
		if (tx_return == TX_NOT_DONE){
			tx_thread_terminate(&end_of_cycle_thread);
			tx_thread_reset(&end_of_cycle_thread);
		}

		// Power up the RF switch
		rf_switch->power_on(rf_switch);

		// Check if there was a GNSS error. If so, reconfigure device
		tx_return = tx_event_flags_get(&thread_control_flags, GNSS_CONFIG_REQUIRED,
				TX_OR_CLEAR, &actual_flags, TX_NO_WAIT);
		if (tx_return == TX_SUCCESS) {
			fail_counter = 0;
			while (fail_counter < MAX_SELF_TEST_RETRIES) {

				register_watchdog_refresh();

				if (gnss->config(gnss) != GNSS_SUCCESS) {
					// Config didn't work, cycle power and try again
					gnss->cycle_power(gnss);
					fail_counter++;
				} else {
					break;
				}
			}
		}
		// If we couldn't configure the GNSS, send a reset vector
		if (fail_counter == MAX_SELF_TEST_RETRIES) {
			shut_it_all_down();
			HAL_NVIC_SystemReset();
		}

		// Kick off the GNSS thread
		if (tx_thread_resume(&gnss_thread) != TX_SUCCESS){
			shut_it_all_down();
			HAL_NVIC_SystemReset();
		}
	}

	// This is first time power up, test everything and flash LED sequence
	else
	{
		register_watchdog_refresh();
		// Flash some lights to let the user know its on and working
		led_sequence(INITIAL_LED_SEQUENCE);

		rf_switch->power_on(rf_switch);

		register_watchdog_refresh();

		self_test_status = initial_power_on_self_test();

		register_watchdog_refresh();

		switch (self_test_status){
			case SELF_TEST_PASSED:
				led_sequence(TEST_PASSED_LED_SEQUENCE);
				break;

			case SELF_TEST_NON_CRITICAL_FAULT:
				led_sequence(TEST_NON_CRITICAL_FAULT_LED_SEQUENCE);
				break;

			case SELF_TEST_CRITICAL_FAULT:
				shut_it_all_down();
				// Stay stuck here
				for (int i = 0; i < 25; i++) {
					register_watchdog_refresh();
					led_sequence(SELF_TEST_CRITICAL_FAULT);
				}
				HAL_NVIC_SystemReset();

			default:
				// If we got here, there's probably a memory corruption
				shut_it_all_down();
				// Stay stuck here
				while (1) {
					register_watchdog_refresh();
					led_sequence(SELF_TEST_CRITICAL_FAULT);
				}
		}

		register_watchdog_refresh();
		// Kick off the GNSS thread
		if (tx_thread_resume(&gnss_thread) != TX_SUCCESS){
			shut_it_all_down();
			HAL_NVIC_SystemReset();
		}
	}

	// We're done, terminate this thread
	tx_thread_terminate(&startup_thread);
}

/**
  * @brief  gnss_thread_entry
  *         Thread that governs the GNSS processing. Note that actual message processing
  *         happens in interrupt context, so this thread is just acting as the traffic cop.
  *
  * @param  ULONG thread_input - unused
  * @retval void
  */
void gnss_thread_entry(ULONG thread_input){

	gnss_error_code_t gnss_return_code;
	int number_of_no_sample_errors = 0;
	float last_lat = 0;
	float last_lon = 0;
	uint8_t sbd_port;
	UINT tx_return;
	ULONG actual_flags;
	int timer_ticks_to_get_message = round(((float)TX_TIMER_TICKS_PER_SECOND /
			(float)configuration.gnss_sampling_rate) + 1);
	uint16_t sample_window_timeout = ((configuration.samples_per_window / configuration.gnss_sampling_rate)
			/ 60) + 2;

	register_watchdog_refresh();

	// Grab the RF switch
	rf_switch->set_gnss_port(rf_switch);

	// Start the timer for resolution stages
	HAL_TIM_Base_Stop_IT(gnss->minutes_timer);
	gnss->reset_timer(gnss, configuration.gnss_max_acquisition_wait_time);
	HAL_TIM_Base_Start_IT(gnss->minutes_timer);

	// Wait until we get a series of good UBX_NAV_PVT messages and are
	// tracking a good number of satellites before moving on
	if (gnss->sync_and_start_reception(gnss, start_GNSS_UART_DMA, ubx_DMA_message_buf, UBX_MESSAGE_SIZE)
				!= GNSS_SUCCESS)
	{
		// If we were unable to get good GNSS reception and start the DMA transfer loop, then
		// go to sleep until the top of the next hour. Sleep will be handled in end_of_cycle_thread
		register_watchdog_refresh();
		jump_to_end_of_window(GNSS_RESOLUTION_ERROR);
	}
	else
	{
		gnss->is_configured = true;
	}

	while (!(gnss->all_resolution_stages_complete || gnss->timer_timeout)) {

		register_watchdog_refresh();
		tx_return = tx_event_flags_get(&thread_control_flags, (GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE),
				TX_OR_CLEAR, &actual_flags, timer_ticks_to_get_message);

		// Full message came through
		if ((tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE)) {
			gnss->process_message(gnss);
		}
		// Message was dropped or incomplete
		else if ((tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE)) {

			continue;

		}
		// Any other error code is indication of memory corruption
		else {

			register_watchdog_refresh();
			jump_to_end_of_window(MEMORY_CORRUPTION_ERROR);
		}


	}

	// If this evaluates to true, we were unable to get adequate GNSS reception to
	// resolve time and get at least 1 good sample. Go to sleep until the top of the next hour.
	// Sleep will be handled in end_of_cycle_thread
	if (gnss->timer_timeout) {
		register_watchdog_refresh();
		jump_to_end_of_window(GNSS_RESOLUTION_ERROR);
	}

	// We were able to resolve time within the given window of time. Now start the timer to ensure
	// the sample window doesn't take too long
	HAL_TIM_Base_Stop_IT(gnss->minutes_timer);
	gnss->reset_timer(gnss, sample_window_timeout);
	HAL_TIM_Base_Start_IT(gnss->minutes_timer);

	// Wait until all the samples have been processed
	while (!gnss->all_samples_processed){

		register_watchdog_refresh();

		tx_return = tx_event_flags_get(&thread_control_flags, GNSS_MSG_RECEIVED | GNSS_MSG_INCOMPLETE,
				TX_OR_CLEAR, &actual_flags, timer_ticks_to_get_message);

		// Full message came through
		if ((tx_return == TX_SUCCESS) && !(actual_flags & GNSS_MSG_INCOMPLETE)) {

			gnss->process_message(gnss);
			number_of_no_sample_errors = 0;

		}
		// Message was dropped or incomplete
		else if ((tx_return == TX_NO_EVENTS) || (actual_flags & GNSS_MSG_INCOMPLETE)) {

			gnss_return_code = gnss->get_running_average_velocities(gnss);

			if (gnss_return_code == GNSS_NO_SAMPLES_ERROR) {
				if (++number_of_no_sample_errors == configuration.gnss_sampling_rate * 60) {
					register_watchdog_refresh();
					jump_to_end_of_window(SAMPLE_WINDOW_ERROR);
				}

			}

		}
		// Any other error code is indication of memory corruption
		else {

			register_watchdog_refresh();
			jump_to_end_of_window(MEMORY_CORRUPTION_ERROR);
		}

		// If this evaluates to true, something hung up with GNSS sampling. End the sample window
		if (gnss->timer_timeout) {
			register_watchdog_refresh();
			jump_to_end_of_window(SAMPLE_WINDOW_ERROR);
		}
	}

	register_watchdog_refresh();

	// Stop the timer
	HAL_TIM_Base_Stop_IT(gnss->minutes_timer);
	// turn off the GNSS sensor
	gnss->on_off(gnss, GPIO_PIN_RESET);
	// Deinit UART and DMA to prevent spurious interrupts
	HAL_UART_DeInit(gnss->gnss_uart_handle);
	HAL_DMA_DeInit(gnss->gnss_rx_dma_handle);
	HAL_DMA_DeInit(gnss->gnss_tx_dma_handle);

	gnss->get_location(gnss, &last_lat, &last_lon);
	// Just to be overly sure about alignment
	memcpy(&sbd_message.Lat, &last_lat, sizeof(float));
	memcpy(&sbd_message.Lon, &last_lon, sizeof(float));
	// We're using the "port" field to encode how many samples were averaged divided by 10,
	// up to the limit of an uint8_t
	sbd_port = ((gnss->total_samples_averaged / 10) >= 255) ? 255 :
			(gnss->total_samples_averaged / 10);
	memcpy(&sbd_message.port, &sbd_port, sizeof(uint8_t));

	// Port the RF switch to the modem
	rf_switch->set_iridium_port(rf_switch);

	register_watchdog_refresh();

#if CT_ENABLED

	if (tx_thread_resume(&ct_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

#else

	if (tx_thread_resume(&waves_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

#endif

	tx_thread_terminate(&gnss_thread);
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
	 *
	 */
//	ULONG actual_flags;
//	tx_event_flags_get(&thread_flags, IMU_READY, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
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
//	ULONG actual_flags;
	ct_error_code_t ct_return_code;
	uint32_t ct_parsing_error_counter;
	real16_T half_salinity;
	real16_T half_temp;
	int fail_counter;

	register_watchdog_refresh();

	// Set the mean salinity and temp values to error values in the event the sensor fails
	half_salinity.bitPattern = CT_AVERAGED_VALUE_ERROR_CODE;
	half_temp.bitPattern = CT_AVERAGED_VALUE_ERROR_CODE;

	memcpy(&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
	memcpy(&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

	// The first message will have a different frame length from a header, so adjust the fail
	// counter appropriately
	fail_counter = -1;
	// Turn on the CT sensor, warm it up, and frame sync
	while (fail_counter < MAX_SELF_TEST_RETRIES) {

		register_watchdog_refresh();

		ct_return_code = ct->self_test(ct, false);
		if (ct_return_code != CT_SUCCESS) {
//			HAL_Delay(103);
			tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
			fail_counter++;

		} else {

			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES) {

		register_watchdog_refresh();
		jump_to_waves();
	}

	// Take our samples
	ct_parsing_error_counter = 0;
	while (ct->total_samples < configuration.total_ct_samples) {

		register_watchdog_refresh();
		ct_return_code = ct_parse_sample(ct);

		if (ct_return_code == CT_PARSING_ERROR) {
			ct_parsing_error_counter++;
		}

		if ((ct_parsing_error_counter >= 10) || (ct_return_code == CT_UART_ERROR)) {
			// If there are too many parsing errors or a UART error occurs, then
			// stop trying and
			register_watchdog_refresh();
			jump_to_waves();
		}

		if (ct_return_code == CT_DONE_SAMPLING) {
			break;
		}
	}

	register_watchdog_refresh();

	// Turn off the CT sensor
	ct->on_off(ct, GPIO_PIN_RESET);
	// Deinit UART and DMA to prevent spurious interrupts
	HAL_UART_DeInit(ct->ct_uart_handle);
	HAL_DMA_DeInit(ct->ct_dma_handle);

	// Got our samples, now average them
	ct_return_code = ct->get_averages(ct);
	// Make sure something didn't go terribly wrong
	if (ct_return_code == CT_NOT_ENOUGH_SAMPLES) {
		register_watchdog_refresh();
		jump_to_waves();
	}

	// Now set the mean salinity and temp values to the real ones
	half_salinity = floatToHalf((float)ct->averages.salinity);
	half_temp = floatToHalf((float)ct->averages.temp);

	memcpy(&sbd_message.mean_salinity, &half_salinity, sizeof(real16_T));
	memcpy(&sbd_message.mean_temp, &half_temp, sizeof(real16_T));

	register_watchdog_refresh();

	if (tx_thread_resume(&waves_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_thread_terminate(&ct_thread);
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

	register_watchdog_refresh();

	/* Call the entry-point 'NEDwaves_memlight'. */
	NEDwaves_memlight(north, east, down, gnss->sample_window_freq, &Hs, &Tp, &Dp, E,
					&b_fmin, &b_fmax, a1, b1, a2, b2, check);

	emxDestroyArray_real32_T(down);
	emxDestroyArray_real32_T(east);
	emxDestroyArray_real32_T(north);

	// Delete the memory pool to fix the memory leak in NEDwaves_memlight
	if (waves_memory_pool_delete() != TX_SUCCESS) {
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	memcpy(&sbd_message.Hs, &Hs, sizeof(real16_T));
	memcpy(&sbd_message.Tp, &Tp, sizeof(real16_T));
	memcpy(&sbd_message.Dp, &Dp, sizeof(real16_T));
	memcpy(&(sbd_message.E_array[0]), &(E[0]), 42 * sizeof(real16_T));
	memcpy(&sbd_message.f_min, &b_fmin, sizeof(real16_T));
	memcpy(&sbd_message.f_max, &b_fmax, sizeof(real16_T));
	memcpy(&(sbd_message.a1_array[0]), &(a1[0]), 42 * sizeof(signed char));
	memcpy(&(sbd_message.b1_array[0]), &(b1[0]), 42 * sizeof(signed char));
	memcpy(&(sbd_message.a2_array[0]), &(a2[0]), 42 * sizeof(signed char));
	memcpy(&(sbd_message.b2_array[0]), &(b2[0]), 42 * sizeof(signed char));
	memcpy(&(sbd_message.cf_array[0]), &(check[0]), 42 * sizeof(unsigned char));

	register_watchdog_refresh();

	if (tx_thread_resume(&iridium_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_thread_terminate(&waves_thread);
}

/**
  * @brief  iridium_thread_entry
  *         This thread will handle message sending via Iridium modem.
  *
  * @param  ULONG thread_input - unused
  * @retval void
  */
void iridium_thread_entry(ULONG thread_input){
	iridium_error_code_t iridium_return_code;
	ULONG actual_error_flags = 0;
	ULONG error_occured_flags = GNSS_ERROR | MODEM_ERROR | MEMORY_ALLOC_ERROR |
			DMA_ERROR | UART_ERROR | RTC_ERROR | WATCHDOG_RESET | SOFTWARE_RESET |
			GNSS_RESOLUTION_ERROR;
	UINT tx_return;
	ULONG actual_flags;
	int fail_counter;
	char ascii_7 = '7';
	uint8_t sbd_type = 52;
	uint16_t sbd_size = 327;
	real16_T voltage;
	float sbd_timestamp = iridium->get_timestamp(iridium);

	register_watchdog_refresh();

	// Check if we are skipping this message
	tx_return = tx_event_flags_get(&error_flags, GNSS_EXITED_EARLY, TX_OR_CLEAR,
			&actual_flags, TX_NO_WAIT);

	if (tx_return == TX_SUCCESS) {
		iridium->skip_current_message = true;
	}

	// If this message was skipped and there's nothing in the queue, exit and jump to end_of_cycle_thread
	if (iridium->skip_current_message && iridium->storage_queue->num_msgs_enqueued == 0) {
		// Turn off the modem and RF switch
		iridium->sleep(iridium, GPIO_PIN_RESET);
		iridium->on_off(iridium, GPIO_PIN_RESET);
		rf_switch->power_off(rf_switch);
		// Deinit UART and DMA to prevent spurious interrupts
		HAL_UART_DeInit(iridium->iridium_uart_handle);
		HAL_DMA_DeInit(iridium->iridium_rx_dma_handle);
		HAL_DMA_DeInit(iridium->iridium_tx_dma_handle);
		HAL_TIM_Base_Stop_IT(iridium->timer);

		// Resume EOC thread
		if (tx_thread_resume(&end_of_cycle_thread) != TX_SUCCESS){
			shut_it_all_down();
			HAL_NVIC_SystemReset();
		}

		tx_thread_terminate(&iridium_thread);
	}

	// Port the RF switch to the modem
	rf_switch->set_iridium_port(rf_switch);

	// Grab the battery voltage
	battery->start_conversion(battery);
	battery->get_voltage(battery, &voltage);
	battery->shutdown_adc();
	memcpy(&sbd_message.mean_voltage, &voltage, sizeof(real16_T));

	// finish filling out the SBD message
	memcpy(&sbd_message.legacy_number_7, &ascii_7, sizeof(char));
	memcpy(&sbd_message.type, &sbd_type, sizeof(uint8_t));
	memcpy(&sbd_message.size, &sbd_size, sizeof(uint16_t));
	memcpy(&sbd_message.timestamp, &sbd_timestamp, sizeof(float));

	// Last posit was placed in the SBD message in GNSS thread, copy that over to
	// the Iridium struct
	memcpy(&iridium->current_lat, &sbd_message.Lat, sizeof(float));
	memcpy(&iridium->current_lon, &sbd_message.Lon, sizeof(float));

	// This will turn on the modem and make sure the caps are charged
	iridium->charge_caps(iridium, IRIDIUM_TOP_UP_CAP_CHARGE_TIME);

	fail_counter = 0;
	while (fail_counter < MAX_SELF_TEST_RETRIES) {

		register_watchdog_refresh();
		// See if we can get an ack message from the modem
		iridium_return_code = iridium->self_test(iridium);
		if (iridium_return_code != IRIDIUM_SUCCESS) {

			iridium->cycle_power(iridium);
//			HAL_Delay(10);
			tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
			fail_counter++;

		} else {

			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES) {
		// If we couldn't get a response from the modem, shut everything down and force reset.
		// But save the current message first
		if (!iridium->skip_current_message) {
			iridium->queue_add(iridium, iridium->current_message);
		}
		shut_it_all_down();
//		HAL_Delay(10);
		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
		HAL_NVIC_SystemReset();
	}

	register_watchdog_refresh();

	iridium->transmit_message(iridium);

	register_watchdog_refresh();

	// Check for error messages
	tx_event_flags_get(&error_flags, error_occured_flags, TX_OR_CLEAR, &actual_error_flags, TX_NO_WAIT);

	// If there was a GNSS error or it could not resolve in time, make sure to terminate the thread
	if ((actual_error_flags & GNSS_RESOLUTION_ERROR) || (actual_error_flags & GNSS_ERROR)){
		// Set the event flag so we know to reconfigure in the next window
		tx_event_flags_set(&thread_control_flags, GNSS_CONFIG_REQUIRED, TX_OR);
	}

	// If we have an error flag, send an error message
	if (actual_error_flags) {
		register_watchdog_refresh();
		send_error_message(actual_error_flags);

		register_watchdog_refresh();
	}

	// If something went wrong with the RTC, we'll reset
	if (actual_error_flags & RTC_ERROR) {
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	// Turn off the modem and RF switch
	iridium->sleep(iridium, GPIO_PIN_RESET);
	iridium->on_off(iridium, GPIO_PIN_RESET);
	rf_switch->power_off(rf_switch);
	// Deinit UART and DMA to prevent spurious interrupts
	HAL_UART_DeInit(iridium->iridium_uart_handle);
	HAL_DMA_DeInit(iridium->iridium_rx_dma_handle);
	HAL_DMA_DeInit(iridium->iridium_tx_dma_handle);
	HAL_TIM_Base_Stop_IT(iridium->timer);

	if (tx_thread_resume(&end_of_cycle_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_thread_terminate(&iridium_thread);
}

/**
  * @brief  teardown_thread_entry
  *         This thread will execute when either an error flag is set or all
  *         the done flags are set, indicating we are ready to shutdown until
  *         the next window.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void end_of_cycle_thread_entry(ULONG thread_input){
	RTC_AlarmTypeDef alarm = {0};
	RTC_TimeTypeDef initial_rtc_time;
	RTC_TimeTypeDef rtc_time;
	RTC_DateTypeDef rtc_date;
	uint32_t wake_up_minute;
	UINT tx_return;

	// Must put this thread to sleep for a short while to allow other threads to terminate
	tx_thread_sleep(1);

	register_watchdog_refresh();

	// Just to be overly sure everything is off
	shut_it_all_down();
//	HAL_Delay(100);
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);

//	Clear any pending interrupts, See Errata section 2.2.4

#if CT_ENABLED
	HAL_NVIC_DisableIRQ(UART4_IRQn);
	HAL_NVIC_ClearPendingIRQ(UART4_IRQn);
#endif

	HAL_NVIC_DisableIRQ(GPDMA1_Channel0_IRQn);
	HAL_NVIC_DisableIRQ(GPDMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(GPDMA1_Channel2_IRQn);
	HAL_NVIC_DisableIRQ(GPDMA1_Channel3_IRQn);
	HAL_NVIC_DisableIRQ(GPDMA1_Channel4_IRQn);
	HAL_NVIC_DisableIRQ(UART5_IRQn);
	HAL_NVIC_DisableIRQ(LPUART1_IRQn);

	HAL_NVIC_ClearPendingIRQ(RTC_S_IRQn);
	HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel0_IRQn);
	HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel1_IRQn);
	HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel2_IRQn);
	HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel3_IRQn);
	HAL_NVIC_ClearPendingIRQ(GPDMA1_Channel4_IRQn);
	HAL_NVIC_ClearPendingIRQ(UART5_IRQn);
	HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);

	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_CLEAR_WUTF);
	__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_CLEAR_ALRAF);
	HAL_NVIC_ClearPendingIRQ(RTC_IRQn);

//	// Only used for low power modes lower than stop2.
//	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);

	HAL_RTC_GetTime(device_handles->hrtc, &initial_rtc_time, RTC_FORMAT_BIN);
	// Must call GetDate to keep the RTC happy, even if you don't use it
	HAL_RTC_GetDate(device_handles->hrtc, &rtc_date, RTC_FORMAT_BIN);

#ifdef SHORT_SLEEP
	wake_up_minute = initial_rtc_time.Minutes >= 59 ? (initial_rtc_time.Minutes + 1) - 60 :
			(initial_rtc_time.Minutes + 1);
#else
	wake_up_minute = 0;
#endif

	HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);

	while (rtc_time.Minutes != wake_up_minute) {

		// Get the date and time
		HAL_RTC_GetTime(device_handles->hrtc, &rtc_time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(device_handles->hrtc, &rtc_date, RTC_FORMAT_BIN);

		// We should be restarting the window at the top of the hour. If the initial time and just
		// checked time differ in hours, then we should start a new window. This should never occur,
		// but just as a second safety
		if (initial_rtc_time.Hours != rtc_time.Hours){
			SystemClock_Config();
			register_watchdog_refresh();
			HAL_ResumeTick();
			HAL_ICACHE_Enable();
			HAL_PWREx_DisableRAMsContentStopRetention(PWR_SRAM4_FULL_STOP_RETENTION);
			HAL_PWREx_DisableRAMsContentStopRetention(PWR_ICACHE_FULL_STOP_RETENTION);
			HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM1_FULL_STOP_RETENTION);
			HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM2_FULL_STOP_RETENTION);
			HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM3_FULL_STOP_RETENTION);
			break;
		}

		// Set the alarm to wake up the processor in 25 seconds
		alarm.Alarm = RTC_ALARM_A;
		alarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
		alarm.AlarmTime = rtc_time;
		alarm.AlarmTime.Seconds = (rtc_time.Seconds >= 35) ? ((rtc_time.Seconds + 25) - 60) : (rtc_time.Seconds + 25);
		alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
		alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;

		// If something goes wrong setting the alarm, force an RTC reset and go to the next window.
		// With luck, the RTC will get set again on the next window and everything will be cool.
		if (HAL_RTC_SetAlarm_IT(device_handles->hrtc, &alarm, RTC_FORMAT_BIN) != HAL_OK) {
			shut_it_all_down();
			HAL_NVIC_SystemReset();
		}

		register_watchdog_refresh();
		// See errata regarding ICACHE access on wakeup, section 2.2.11
		HAL_ICACHE_Disable();
		HAL_SuspendTick();

		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

		register_watchdog_refresh();
		// Restore clocks to the same config as before stop2 mode
		SystemClock_Config();
		HAL_ResumeTick();
		HAL_ICACHE_Enable();
		HAL_PWREx_DisableRAMsContentStopRetention(PWR_SRAM4_FULL_STOP_RETENTION);
		HAL_PWREx_DisableRAMsContentStopRetention(PWR_ICACHE_FULL_STOP_RETENTION);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM1_FULL_STOP_RETENTION);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM2_FULL_STOP_RETENTION);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM3_FULL_STOP_RETENTION);

	}

	register_watchdog_refresh();

	// Disable the RTC Alarm and clear flags
	HAL_RTC_DeactivateAlarm(device_handles->hrtc, RTC_ALARM_A);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_CLEAR_WUTF);
	__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_CLEAR_ALRAF);
	HAL_NVIC_ClearPendingIRQ(RTC_IRQn);

#if CT_ENABLED
	HAL_NVIC_EnableIRQ(UART4_IRQn);
#endif
	HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);
	HAL_NVIC_EnableIRQ(UART5_IRQn);
	HAL_NVIC_EnableIRQ(LPUART1_IRQn);
//	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);

	// Reset and resume the startup thread
	tx_return = tx_thread_reset(&startup_thread);
	if (tx_return != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_return = tx_thread_resume(&startup_thread);
	if (tx_return != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_event_flags_set(&thread_control_flags, FULL_CYCLE_COMPLETE, TX_OR);
	tx_thread_terminate(&end_of_cycle_thread);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == gnss->gnss_uart_handle->Instance) {
		if (Size < UBX_NAV_PVT_MESSAGE_LENGTH) {
			gnss->get_running_average_velocities(gnss);
			tx_event_flags_set(&thread_control_flags, GNSS_MSG_INCOMPLETE, TX_OR);
		} else {
			memcpy(&(gnss->ubx_process_buf[0]), &(ubx_DMA_message_buf[0]), UBX_MESSAGE_SIZE);
			tx_event_flags_set(&thread_control_flags, GNSS_MSG_RECEIVED, TX_OR);
		}
	}
}

/**
  * @brief  UART ISR callback
  *
  * @param  UART_HandleTypeDef *huart - pointer to the UART handle

  * @retval void
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Context is saved/restored in upstream Interrupt call chain
	if (huart->Instance == gnss->gnss_uart_handle->Instance) {
		if (!gnss->is_configured) {

			tx_event_flags_set(&thread_control_flags, GNSS_CONFIG_RECVD, TX_OR);

		} else {
			memcpy(&(gnss->ubx_process_buf[0]), &(ubx_DMA_message_buf[0]), UBX_MESSAGE_SIZE);
			tx_event_flags_set(&thread_control_flags, GNSS_MSG_RECEIVED, TX_OR);
		}
	}

	// CT sensor
#if CT_ENABLED
	else if (huart->Instance == ct->ct_uart_handle->Instance) {
		tx_event_flags_set(&thread_control_flags, CT_MSG_RECVD, TX_OR);
	}
#endif

	// Iridium modem
	else if (huart->Instance == iridium->iridium_uart_handle->Instance) {
		tx_event_flags_set(&thread_control_flags, IRIDIUM_MSG_RECVD, TX_OR);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == gnss->gnss_uart_handle->Instance) {
		tx_event_flags_set(&thread_control_flags, GNSS_TX_COMPLETE, TX_OR);
	} else if (huart->Instance == iridium->iridium_uart_handle->Instance) {
		tx_event_flags_set(&thread_control_flags, IRIDIUM_TX_COMPLETE, TX_OR);
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  *
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// High frequency, low overhead ISR, no need to save/restore context
	if (htim->Instance == TIM4) {
		HAL_IncTick();
	}
	else if (htim->Instance == TIM17) {
		iridium->timer_timeout = true;
	}
	else if (htim->Instance == TIM16) {
		gnss->timer_timeout = true;
	}
	else if (htim->Instance == TIM15) {
		watchdog_hour_timer_elapsed = true;
	}
}

/**
  * @brief  RTC alarm interrupt callback
  *
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	// Low overhead ISR does not require save/restore context
	// Clear the alarm flag, flash an LED in debug mode
	HAL_PWR_EnableBkUpAccess();
	__HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_CLEAR_ALRAF);
	// Clear the Wake-up timer flag too (Errata 2.2.4)
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_CLEAR_WUTF);

}



void HAL_IWDG_EarlyWakeupCallback(IWDG_HandleTypeDef *hiwdg)
{
	shut_it_all_down();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint32_t number_of_conversions = 0;
	static uint64_t v_sum = 0;

	if (number_of_conversions < NUMBER_OF_ADC_SAMPLES) {
		uint32_t sample = HAL_ADC_GetValue(hadc);
		adc_buf[number_of_conversions] = sample;
		v_sum += (sample * ADC_MICROVOLTS_PER_BIT) + battery->calibration_offset;
		number_of_conversions++;
	}
	else {
		// Write the voltage to the battery struct
		battery->voltage = (float)((((float)v_sum / (float)NUMBER_OF_ADC_SAMPLES) + ADC_CALIBRATION_CONSTANT_MICROVOLTS) / MICROVOLTS_PER_VOLT);
		// Reset static variables
		v_sum = 0;
		number_of_conversions = 0;
		// Set the conversion complete flag
		tx_event_flags_set(&thread_control_flags, BATTERY_VOLTAGE_CONVERSION_COMPLETE, TX_OR);
		// Done with our samples, shut it down.
		HAL_ADC_Stop_IT(hadc);
	}

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	// Set the ADC conversion error flag and move on
	tx_event_flags_set(&error_flags, ADC_CONVERSION_ERROR, TX_OR);
}

///**
//  * @brief  Delay in blocking mode. This is to ensure HAL_delay will work even if SysTick is disabled.
//  *
//  * @note   !!! This only works with a main clock speed of 12mHz !!!
//  *
//  * @param  Delay : Delay in milliseconds
//  * @retval None
//  */
//void HAL_Delay(uint32_t Delay)
//{
//	__IO int dummy_variable = 0;
//	__IO int i;
//
//	// Compiler cannot optimize any of this away.
//	for (i = 0; i < Delay * 532; i++) {
//		i++;
//		dummy_variable = i;
//		i--;
//	}
//	i = dummy_variable;
//}

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
			for (int i = 0; i < 10; i++){
				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//				HAL_Delay(250);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 4);
				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
//				HAL_Delay(250);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 4);
				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_RESET);
//				HAL_Delay(250);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 4);
				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
//				HAL_Delay(250);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 4);
			}
			break;

		case TEST_PASSED_LED_SEQUENCE:
			for (int i = 0; i < 5; i++){
				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
//				HAL_Delay(1000);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
//				HAL_Delay(1000);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
			}
			break;

		case TEST_NON_CRITICAL_FAULT_LED_SEQUENCE:
			for (int i = 0; i < 20; i++){
				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
//				HAL_Delay(250);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 4);
				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
//				HAL_Delay(250);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 4);
			}
			break;

		case TEST_CRITICAL_FAULT_LED_SEQUENCE:
			for (int i = 0; i < 10; i++){
				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_RESET);
//				HAL_Delay(500);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//				HAL_Delay(500);
				tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2);
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

	register_watchdog_refresh();

	gnss->reset_uart(gnss, GNSS_DEFAULT_BAUD_RATE);

	memset(&(buffer[0]), 0, UBX_MESSAGE_SIZE * 2);

	HAL_UART_DMAStop(gnss_struct_ptr->gnss_uart_handle);

	hal_return_code = MX_GNSS_LL_Queue_Config();

	if (hal_return_code != HAL_OK) {
		return_code = GNSS_UART_ERROR;
	}

	gnss_struct_ptr->gnss_rx_dma_handle->InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
	gnss_struct_ptr->gnss_rx_dma_handle->InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
	gnss_struct_ptr->gnss_rx_dma_handle->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
	gnss_struct_ptr->gnss_rx_dma_handle->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
	gnss_struct_ptr->gnss_rx_dma_handle->InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;

	if (HAL_DMAEx_List_Init(gnss_struct_ptr->gnss_rx_dma_handle) != HAL_OK)
	{
		return_code = GNSS_UART_ERROR;
	}

	__HAL_LINKDMA(gnss_struct_ptr->gnss_uart_handle, hdmarx, *gnss_struct_ptr->gnss_rx_dma_handle);

	hal_return_code = HAL_DMAEx_List_LinkQ(gnss_struct_ptr->gnss_rx_dma_handle, &GNSS_LL_Queue);
	if (hal_return_code != HAL_OK) {
		return_code = GNSS_UART_ERROR;
	}

	hal_return_code = HAL_UARTEx_ReceiveToIdle_DMA(gnss_struct_ptr->gnss_uart_handle,
			(uint8_t*)&(buffer[0]), msg_size);
		//  No need for the half-transfer complete interrupt, so disable it
	__HAL_DMA_DISABLE_IT(gnss->gnss_rx_dma_handle, DMA_IT_HT);

	if (hal_return_code != HAL_OK) {
		return_code = GNSS_UART_ERROR;
	}

	return return_code;
}

/**
  * @brief  Power down all peripheral FETs and set RF switch to GNSS input
  *
  * @param  void
  *
  * @retval void
  */
void shut_it_all_down(void)
{
	// Shut down Iridium modem
	HAL_GPIO_WritePin(GPIOD, IRIDIUM_OnOff_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, GPIO_PIN_RESET);
	// Shut down GNSS
	HAL_GPIO_WritePin(GPIOG, GNSS_FET_Pin, GPIO_PIN_RESET);
	// Reset RF switch GPIOs. This will set it to be ported to the modem (safe case)
	HAL_GPIO_WritePin(GPIOD, RF_SWITCH_VCTL_Pin, GPIO_PIN_RESET);
	// Turn off power to the RF switch
	HAL_GPIO_WritePin(GPIOD, RF_SWITCH_EN_Pin, GPIO_PIN_RESET);
	// Shut down CT sensor
	HAL_GPIO_WritePin(GPIOG, CT_FET_Pin, GPIO_PIN_RESET);

}


/**
  * @brief  Put to the watchdog semaphore so the watchdog thread can refresh IWDG
  *
  * @param  void
  *
  * @retval void
  */
void register_watchdog_refresh(void)
{
#if WATCHDOG_ENABLED
	if (!watchdog_hour_timer_elapsed) {
		tx_semaphore_put(&watchdog_semaphore);
	}
#endif
}




/**
  * @brief  Test communication with each peripheral.
  *
  * @param  void
  *
  * @retval SELF_TEST_PASSED
  * 		SELF_TEST_NON_CRITICAL_FAULT --> if CT or IMU failed
  * 		SELF_TEST_CRITICAL_FAULT --> if GNSS or Iridium modem
  */
static self_test_status_t initial_power_on_self_test(void)
{
	self_test_status_t return_code;
	gnss_error_code_t gnss_return_code;
	iridium_error_code_t iridium_return_code;
#if CT_ENABLED
	ct_error_code_t ct_return_code;
#endif
	int fail_counter;

	///////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////// GNSS STARTUP SEQUENCE /////////////////////////////////////////////
	// turn on the GNSS FET
	gnss->on_off(gnss, GPIO_PIN_SET);
//	HAL_Delay(100);
	tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
	// Send the configuration commands to the GNSS unit.
	fail_counter = 0;
	while (fail_counter < MAX_SELF_TEST_RETRIES * 2) {

		register_watchdog_refresh();

		gnss_return_code = gnss->config(gnss);
		if (gnss_return_code != GNSS_SUCCESS) {
			// Config didn't go through, try again
			fail_counter++;

		} else {

			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES) {

		return_code = SELF_TEST_CRITICAL_FAULT;
		tx_event_flags_set(&error_flags, GNSS_ERROR, TX_OR);
		return return_code;
	}

	// If we made it here, the self test passed and we're ready to process messages
	tx_event_flags_set(&thread_control_flags, GNSS_READY, TX_OR);

	///////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////IRIDIUM STARTUP SEQUENCE ///////////////////////////////////////////////
	// Only do this on initial power up, else leave it alone!
	iridium->queue_flush(iridium);
	// Turn on the modem and charge up the caps
	iridium->charge_caps(iridium, IRIDIUM_INITIAL_CAP_CHARGE_TIME);

	// Send over an ack message and make sure we get a response
	fail_counter = 0;
	while (fail_counter < MAX_SELF_TEST_RETRIES) {

		register_watchdog_refresh();
		// See if we can get an ack message from the modem
		iridium_return_code = iridium->self_test(iridium);
		if (iridium_return_code != IRIDIUM_SUCCESS) {

			iridium->cycle_power(iridium);
//			HAL_Delay(10);
			tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
			fail_counter++;

		} else {

			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES) {

		return_code = SELF_TEST_CRITICAL_FAULT;
		tx_event_flags_set(&error_flags, MODEM_ERROR, TX_OR);
		return return_code;
	}

	// Send the configuration settings to the modem
	fail_counter = 0;
	while (fail_counter < MAX_SELF_TEST_RETRIES) {

		register_watchdog_refresh();

		iridium_return_code = iridium->config(iridium);
		if (iridium_return_code != IRIDIUM_SUCCESS) {

			iridium->cycle_power(iridium);
//			HAL_Delay(10);
			tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
			fail_counter++;

		} else {

			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES) {

		return_code = SELF_TEST_CRITICAL_FAULT;
		tx_event_flags_set(&error_flags, MODEM_ERROR, TX_OR);
		return return_code;
	}

	// We'll keep power to the modem but put it to sleep
	iridium->sleep(iridium, GPIO_PIN_RESET);

	// We got an ack and were able to config the Iridium modem
	tx_event_flags_set(&thread_control_flags, IRIDIUM_READY, TX_OR);

#if IMU_ENABLED
	///////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////// IMU STARTUP SEQUENCE ///////////////////////////////////////////////
#endif

#if CT_ENABLED
	///////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////// CT STARTUP SEQUENCE ///////////////////////////////////////////////
	// Make sure we get good data from the CT sensor
	// The first message will have a different frame length from a header, so adjust the fail
	// counter appropriately
	fail_counter = -1;
	while (fail_counter < MAX_SELF_TEST_RETRIES) {

		register_watchdog_refresh();

		ct_return_code = ct->self_test(ct, false);
		if (ct_return_code != CT_SUCCESS) {

//			HAL_Delay(103);
			tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
			fail_counter++;

		} else {

			return_code = SELF_TEST_PASSED;
			break;
		}
	}

	if (fail_counter == MAX_SELF_TEST_RETRIES) {

		return_code = SELF_TEST_NON_CRITICAL_FAULT;
		tx_event_flags_set(&error_flags, CT_ERROR, TX_OR);
	}

	// We can turn off the CT sensor for now
	ct->on_off(ct, GPIO_PIN_RESET);

	// Regardless of if the self-test passed, we'll still set it as ready and try again
	// in the sample window
	tx_event_flags_set(&thread_control_flags, CT_READY, TX_OR);
#else
	return_code = SELF_TEST_PASSED;
#endif

	return return_code;
}

/**
  * @brief  Break out of the GNSS thread and jump to end_of_cycle_thread
  *
  * @param  thread_to_terminate - thread which called this
  *
  * @retval void
  */
static void jump_to_end_of_window(ULONG error_bits_to_set)
{
	gnss->on_off(gnss, GPIO_PIN_RESET);
	tx_event_flags_set(&error_flags, (error_bits_to_set | GNSS_EXITED_EARLY), TX_OR);
	// Deinit UART and DMA to prevent spurious interrupts
	HAL_UART_DeInit(gnss->gnss_uart_handle);
	HAL_DMA_DeInit(gnss->gnss_rx_dma_handle);
	HAL_DMA_DeInit(gnss->gnss_tx_dma_handle);
	HAL_TIM_Base_Stop_IT(gnss->minutes_timer);

	if (waves_memory_pool_delete() != TX_SUCCESS) {
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}


	if (tx_thread_resume(&iridium_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_thread_terminate(&gnss_thread);
}

#if CT_ENABLED
/**
  * @brief  Break out of the CT thread and jump to Waves thread
  *
  * @param  void
  *
  * @retval void
  */
static void jump_to_waves(void)
{
	ct->on_off(ct, GPIO_PIN_RESET);
	// Deinit UART and DMA to prevent spurious interrupts
	HAL_UART_DeInit(ct->ct_uart_handle);
	HAL_DMA_DeInit(ct->ct_dma_handle);

	if (tx_thread_resume(&waves_thread) != TX_SUCCESS){
		shut_it_all_down();
		HAL_NVIC_SystemReset();
	}

	tx_thread_terminate(&ct_thread);
}
#endif

/**
  * @brief  If an error was detected along the way, send an error (Type 99) message.
  *
  * @param  error_flags - retreived error flags
  *
  * @retval void
  */
static void send_error_message(ULONG error_flags)
{
	iridium_error_code_t return_code;
	char error_message[ERROR_MESSAGE_MAX_LENGTH] = {0};
	const char* watchdog_reset = "WATCHDOG RESET. ";
	const char* software_reset = "SOFTWARE RESET. ";
	const char* gnss_error = "GNSS ERROR. ";
	const char* gnss_resolution_error = "GNSS RESOLUTION ERROR. ";
	const char* sample_window_error = "SAMPLE WINDOW ERROR. ";
	const char* memory_corruption_error = "MEMORY CORRUPTION ERROR. ";
	const char* memory_alloc_error = "MEMORY ALLOC ERROR. ";
	const char* dma_error =  "DMA ERROR. ";
	const char* uart_error = "UART ERROR. ";
	const char* rtc_error = "RTC ERROR. ";
	char* string_ptr = &(error_message[0]);

	if (error_flags & WATCHDOG_RESET) {

		if ((string_ptr - &(error_message[0])) + strlen(watchdog_reset)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, watchdog_reset, strlen(watchdog_reset));
			string_ptr += strlen(watchdog_reset);
		}

	}

	if (error_flags & SOFTWARE_RESET) {

		if ((string_ptr - &(error_message[0])) + strlen(software_reset)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, software_reset, strlen(software_reset));
			string_ptr += strlen(software_reset);
		}

	}

	if (error_flags & GNSS_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(gnss_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, gnss_error, strlen(gnss_error));
			string_ptr += strlen(gnss_error);
		}
	}

	if (error_flags & GNSS_RESOLUTION_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(gnss_resolution_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, gnss_resolution_error, strlen(gnss_resolution_error));
			string_ptr += strlen(gnss_resolution_error);
		}
	}

	if (error_flags & SAMPLE_WINDOW_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(sample_window_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, sample_window_error, strlen(sample_window_error));
			string_ptr += strlen(sample_window_error);
		}
	}

	if (error_flags & MEMORY_CORRUPTION_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(memory_corruption_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, memory_corruption_error, strlen(memory_corruption_error));
			string_ptr += strlen(memory_corruption_error);
		}
	}

	if (error_flags & MEMORY_ALLOC_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(memory_alloc_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, memory_alloc_error, strlen(memory_alloc_error));
			string_ptr += strlen(memory_alloc_error);
		}
	}

	if (error_flags & DMA_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(dma_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, dma_error, strlen(dma_error));
			string_ptr += strlen(dma_error);
		}
	}

	if (error_flags & UART_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(uart_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, uart_error, strlen(uart_error));
			string_ptr += strlen(uart_error);
		}
	}

	if (error_flags & RTC_ERROR)
	{
		if ((string_ptr - &(error_message[0])) + strlen(rtc_error)
				<= ERROR_MESSAGE_MAX_LENGTH)
		{
			memcpy(string_ptr, rtc_error, strlen(rtc_error));
			string_ptr += strlen(rtc_error);
		}
	}

	return_code = iridium->transmit_error_message(iridium, error_message);

	if ((return_code == IRIDIUM_SUCCESS) && (device_handles->reset_reason != 0))
	{
		// Only want to send this message once, so clear reset_reason
		device_handles->reset_reason = 0;
	}
}


/* USER CODE END 1 */
