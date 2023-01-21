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
  *TODO:
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
// gps_thread, imu_thread, and waves_thread will get a large stack, startup_thread
// and teardown_thread will get small stacks.
#define THREAD_EXTRA_LARGE_STACK_SIZE 4096
#define THREAD_LARGE_STACK_SIZE 2048
#define THREAD_SMALL_STACK_SIZE 512
// Sensor data arrays -> 2bytes * 8192 samples = 16384 bytes, which is 32 byte aligned.
#define SENSOR_DATA_ARRAY_SIZE (TOTAL_SAMPLES_PER_WINDOW * sizeof(int16_t))
// Waves arrays -> 4 bytes * 8192 samples = 32786 bytes, which is 32 byte aligned.
#define WAVES_ARRAY_SIZE 32768
// Size of the CT data array TODO: figure out exact size needed
#define CT_DATA_ARRAY_SIZE 512
// Size of an Iridium message TODO: figure this out
#define IRIDIUM_MESSAGE_SIZE 340
// The size of our queue
#define UBX_QUEUE_SIZE 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
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
// We'll use flags to signal other threads to run/shutdown
TX_EVENT_FLAGS_GROUP thread_flags;
// All our data to store/ process
int16_t* GNSS_N_Array;
int16_t* GNSS_E_Array;
int16_t* GNSS_D_Array;
//int16_t* IMU_N_Array;
//int16_t* IMU_E_Array;
//int16_t* IMU_D_Array;
volatile float* waves_N_Array;
volatile float* waves_E_Array;
volatile float* waves_D_Array;
CHAR ubx_DMA_message_buf[UBX_BUFFER_SIZE];

char queue_message_1[UBX_BUFFER_SIZE];
char queue_message_2[UBX_BUFFER_SIZE];
char queue_message_3[UBX_BUFFER_SIZE];

volatile CHAR ct_data;
volatile CHAR iridium_message;
GNSS* gnss;

UART_HandleTypeDef* gnss_uart;
DMA_HandleTypeDef* dma_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void startup_thread_entry(ULONG thread_input);
void gnss_thread_entry(ULONG thread_input);
void imu_thread_entry(ULONG thread_input);
void ct_thread_entry(ULONG thread_input);
void waves_thread_entry(ULONG thread_input);
void iridium_thread_entry(ULONG thread_input);
void teardown_thread_entry(ULONG thread_input);

// Static helper functions
static void reset_GNSS_uart();
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

	//
	// Allocate stack for the CT thread
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_SMALL_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the CT thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&ct_thread, "ct thread", ct_thread_entry, 0, pointer,
		  THREAD_SMALL_STACK_SIZE, VERY_HIGH, VERY_HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
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
	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_SMALL_STACK_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// Create the Iridium thread. VERY_HIGH priority, no preemption-threshold
	ret = tx_thread_create(&iridium_thread, "iridium thread", iridium_thread_entry, 0, pointer,
		  THREAD_SMALL_STACK_SIZE, MID, MID, TX_NO_TIME_SLICE, TX_AUTO_START);
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
//	ret = tx_byte_allocate(byte_pool, (VOID**) &IMU_N_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	ret = tx_byte_allocate(byte_pool, (VOID**) &IMU_E_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	ret = tx_byte_allocate(byte_pool, (VOID**) &IMU_D_Array, SENSOR_DATA_ARRAY_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
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

	// The UBX message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_DMA_message_buf, UBX_BUFFER_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}

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


	// The CT data array
	ret = tx_byte_allocate(byte_pool, (VOID**) &ct_data, CT_DATA_ARRAY_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// The Iridium message array
	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_message, IRIDIUM_MESSAGE_SIZE, TX_NO_WAIT);
	if (ret != TX_SUCCESS){
	  return ret;
	}
	// The gnss struct
	ret = tx_byte_allocate(byte_pool, (VOID**) &gnss, sizeof(GNSS), TX_NO_WAIT);
	if (ret != TX_SUCCESS){
		return ret;
	}
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
void MX_ThreadX_Init(UART_HandleTypeDef* gnss_uart_handle, DMA_HandleTypeDef* handle_GPDMA1_Channel0)
{
  /* USER CODE BEGIN  Before_Kernel_Start */
  gnss_uart = gnss_uart_handle;
  dma_handle = handle_GPDMA1_Channel0;
  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/**
  * @brief  App_ThreadX_LowPower_Timer_Setup
  * @param  count : TX timer count
  * @retval None
  */
void App_ThreadX_LowPower_Timer_Setup(ULONG count)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Timer_Setup */

  /* USER CODE END  App_ThreadX_LowPower_Timer_Setup */
}

/**
  * @brief  App_ThreadX_LowPower_Enter
  * @param  None
  * @retval None
  */
void App_ThreadX_LowPower_Enter(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Enter */

  /* USER CODE END  App_ThreadX_LowPower_Enter */
}

/**
  * @brief  App_ThreadX_LowPower_Exit
  * @param  None
  * @retval None
  */
void App_ThreadX_LowPower_Exit(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Exit */

  /* USER CODE END  App_ThreadX_LowPower_Exit */
}

/**
  * @brief  App_ThreadX_LowPower_Timer_Adjust
  * @param  None
  * @retval Amount of time (in ticks)
  */
ULONG App_ThreadX_LowPower_Timer_Adjust(void)
{
  /* USER CODE BEGIN  App_ThreadX_LowPower_Timer_Adjust */
  return 0;
  /* USER CODE END  App_ThreadX_LowPower_Timer_Adjust */
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
	// TODO: figure out self-check
	// TODO: set event flags to "ready" for all threads except waves, Iridium
	HAL_StatusTypeDef HAL_return;
	UINT threadx_return;
	int fail_counter = 0;
	/* WAVES TEST */
//	tx_event_flags_set(&thread_flags, WAVES_READY, TX_OR);
//
//	tx_event_flags_get(&thread_flags, GNSS_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);
	/* END WAVES TEST */


	// Initialize GNSS struct
	gnss_init(gnss, gnss_uart, dma_handle, &thread_flags, &ubx_queue,
			GNSS_N_Array, GNSS_E_Array, GNSS_D_Array);
	// Send the configuration commands to the GNSS unit.
	while (gnss->config(gnss) != GNSS_SUCCESS) {
		if (++fail_counter == 10) {
			// TODO: cycle power to the board, do some stuff
		}
	}
	// Start GNSS DMA reception
	fail_counter = 0;
	while (HAL_UART_Receive_DMA(gnss->gnss_uart_handle,
			(uint8_t*)&(ubx_DMA_message_buf[0]), UBX_BUFFER_SIZE) != HAL_OK) {

		reset_GNSS_uart();
		LL_DMA_ResetChannel(GPDMA1, LL_DMA_CHANNEL_0);

		if (++fail_counter == 10) {
			// TODO: cycle power to the board, do some stuff
		}
	}
	//  No need for the half-transfer complete interrupt, so disable it
	__HAL_DMA_DISABLE_IT(dma_handle, DMA_IT_HT);
	//	__HAL_UART_ENABLE_IT(gnss->gnss_uart_handle, UART_IT_IDLE);

	// Wait until we get valid UBX messages in before we move on
	fail_counter = 0;
	while (gnss->self_test(gnss) != GNSS_SUCCESS) {
		if (++fail_counter == 10) {
			// TODO: cycle power to the board, do some stuff
		}
	}

	// We received a bunch of good quality messages, GNSS is good
	threadx_return = tx_event_flags_set(&thread_flags, GNSS_READY, TX_OR);
	if (threadx_return != TX_SUCCESS) {
		// TODO: create a "handle_tx_error" function and call it in here
		HAL_Delay(10);
		tx_event_flags_set(&thread_flags, GNSS_READY, TX_OR);
	}
	// This thread will suspend on exit and will not be restarted
}

/**
  * @brief  gnss_thread_entry
  *         Thread that governs the GNSS message processing and building of
  *         uGNSSArray, vGNSSArray, zGNSSArray arrays.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void gnss_thread_entry(ULONG thread_input){
	UINT threadx_retern;
	ULONG actual_flags;
	// Make sure we have the ready flag
	tx_event_flags_get(&thread_flags, GNSS_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);

	while(1){
			gnss->gnss_process_message(gnss);
	}
}

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

/**
  * @brief  ct_thread_entry
  *         This thread will handle the CT sensor, capture readings, and store
  *         in ct_data.
  * @param  ULONG thread_input - unused
  * @retval void
  */
void ct_thread_entry(ULONG thread_input){
	ULONG actual_flags;
	tx_event_flags_get(&thread_flags, CT_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);
}

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
	tx_event_flags_get(&thread_flags, IRIDIUM_READY, TX_OR, &actual_flags, TX_WAIT_FOREVER);
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

/**
  * @brief  UART ISR callback
  *
  * @param  UART_HandleTypeDef *huart - pointer to the UART handle

  * @retval void
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Save the thread context
	_tx_thread_context_save();
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
			UINT ret;
			// get info on the number of enqueued messages and available space
			tx_queue_info_get(&ubx_queue, TX_NULL, &num_msgs_enqueued,
					&available_space, TX_NULL, TX_NULL, TX_NULL);

			CHAR* current_msg;
			// Find the right queue message pointer to assign to
			switch(num_msgs_enqueued){
			case 0:
				current_msg = &(queue_message_1[0]);
				break;
			case 1:
				current_msg = &(queue_message_2[0]);
				break;
			case 2:
				current_msg = &(queue_message_3[0]);
				break;
			case 3:
				// TODO: figure out this error condition
				current_msg = &(queue_message_3[0]);
//				return;
			default:
				current_msg = &(queue_message_1[0]);
				break;
			}

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
	// Restore the thread context
	_tx_thread_context_restore();
}


static void reset_GNSS_uart()
{
	if (HAL_UART_DeInit(gnss->gnss_uart_handle) != HAL_OK) {
		Error_Handler();
	}
	gnss->gnss_uart_handle->Instance = USART3;
	gnss->gnss_uart_handle->Init.BaudRate = 9600;
	gnss->gnss_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
	gnss->gnss_uart_handle->Init.StopBits = UART_STOPBITS_1;
	gnss->gnss_uart_handle->Init.Parity = UART_PARITY_NONE;
	gnss->gnss_uart_handle->Init.Mode = UART_MODE_TX_RX;
	gnss->gnss_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	gnss->gnss_uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
	gnss->gnss_uart_handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	gnss->gnss_uart_handle->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	gnss->gnss_uart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(gnss->gnss_uart_handle) != HAL_OK)
	{
	Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(gnss->gnss_uart_handle,
			UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(gnss->gnss_uart_handle,
			UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(gnss->gnss_uart_handle) != HAL_OK)
	{
	Error_Handler();
	}
}

/* USER CODE END 1 */
