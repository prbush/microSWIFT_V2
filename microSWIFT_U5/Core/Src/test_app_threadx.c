///*
// * test_app_threadx.c
// *
// *  Created on: May 17, 2023
// *      Author: Phil
// */
//
//
///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file    app_threadx.c
//  * @author  MCD Application Team
//  * @brief   ThreadX applicative file
//  ******************************************************************************
//    * @attention
//  *
//  * Copyright (c) 2022 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  ******************************************************************************
//  *
//  */
//#include "tx_api.h"
//#include "main.h"
//#include <math.h>
//#include "stm32u5xx_hal.h"
//#include "stm32u5xx_ll_dma.h"
//#include "stdint.h"
//#include "stdbool.h"
//#include "string.h"
//#include "Peripherals/gnss.h"
//#include "Peripherals/battery.h"
//#include "Peripherals/ct_sensor.h"
//#include "Peripherals/rf_switch.h"
//#include "Peripherals/imu.h"
//#include "Peripherals/iridium.h"
//#include "NEDWaves/mem_replacements.h"
//#include "configuration.h"
//#include "linked_list.h"
//
//// Waves files
//#include "NEDWaves/NEDwaves_memlight.h"
//#include "NEDWaves/NEDwaves_memlight_emxAPI.h"
//#include "NEDWaves/NEDwaves_memlight_terminate.h"
//#include "NEDWaves/NEDwaves_memlight_types.h"
//#include "NEDWaves/rt_nonfinite.h"
//#include "NEDWaves/rtwhalf.h"
//
//
//typedef enum control_flags{
// 	// Ready states
// 	GNSS_READY = 1 << 0,
// 	IMU_READY = 1 << 1,
// 	CT_READY = 1 << 2,
// 	IRIDIUM_READY = 1 << 3,
// 	WAVES_READY = 1 << 4,
// 	// Done states
// 	GNSS_DONE = 1 << 5,
// 	IMU_DONE = 1 << 6,
// 	CT_DONE = 1 << 7,
// 	IRIDIUM_DONE = 1 << 8,
//	WAVES_DONE = 1 << 9,
//	FULL_CYCLE_COMPLETE = 1 << 10,
//	// DMA reception flags
//	GNSS_CONFIG_RECVD = 1 << 11,
//	CT_MSG_RECVD = 1 << 12,
//	IRIDIUM_MSG_RECVD = 1 << 13
//}control_flags_t;
//
//typedef enum error_flags{
// 	GNSS_ERROR = 1 << 1,
// 	IMU_ERROR = 1 << 2,
// 	CT_ERROR = 1 << 3,
// 	MODEM_ERROR = 1 << 4,
// 	MEMORY_ALLOC_ERROR = 1 << 5,
// 	DMA_ERROR = 1 << 6,
// 	UART_ERROR = 1 << 7,
//	RTC_ERROR = 1 << 8
//}error_flags_t;
//
//typedef enum led_sequence{
//	INITIAL_LED_SEQUENCE = 1,
//	TEST_PASSED_LED_SEQUENCE = 2,
//	TEST_NON_CRITICAL_FAULT_LED_SEQUENCE = 3,
//	TEST_CRITICAL_FAULT_LED_SEQUENCE = 4
//}led_sequence_t;
//
//typedef enum self_test_status{
//	SELF_TEST_PASSED = 0,
//	SELF_TEST_NON_CRITICAL_FAULT = 1,
//	SELF_TEST_CRITICAL_FAULT = 2
//}self_test_status_t;
//
//#define THREAD_EXTRA_LARGE_STACK_SIZE 4096
//#define THREAD_LARGE_STACK_SIZE 2048
//#define THREAD_MEDIUM_STACK_SIZE 1024
//#define THREAD_SMALL_STACK_SIZE 512
//#define THREAD_EXTRA_SMALL_STACK_SIZE 256
//
//#define MAX_SELF_TEST_RETRIES 3
//
//#define MAX_ALLOWABLE_WINDOW_TIME_IN_MINUTES 60
//
//UINT App_ThreadX_Init(VOID *memory_ptr);
//
//typedef enum thread_priorities{
//	HIGHEST = 0,
//	VERY_HIGH = 1,
//	HIGH = 2,
//	MID= 3,
//	LOW = 4,
//	LOWEST = 5
//}thread_priorities_t;
//
//void watchdog_thread_entry(ULONG thread_input);
//void test_thread_entry(ULONG thread_input);
//gnss_error_code_t start_GNSS_UART_DMA(GNSS* gnss_struct_ptr, uint8_t* buffer, size_t buffer_size);
//static void led_sequence(led_sequence_t sequence);
//void shut_it_all_down(void);
//void turn_it_all_on(void);
//
//TX_EVENT_FLAGS_GROUP thread_control_flags;
//TX_EVENT_FLAGS_GROUP error_flags;
//TX_SEMAPHORE window_started_semaphone;
//extern DMA_QListTypeDef GNSS_LL_Queue;
//microSWIFT_configuration configuration;
//sbd_message_type_52 sbd_message;
//TX_BYTE_POOL *byte_pool;
//TX_THREAD watchdog_thread;
//TX_THREAD test_thread;
//emxArray_real32_T *north;
//emxArray_real32_T *east;
//emxArray_real32_T *down;
//uint8_t* ubx_DMA_message_buf;
//uint8_t* ubx_message_process_buf;
//uint8_t* iridium_message;
//uint8_t* iridium_response_message;
//uint8_t* iridium_error_message;
//Iridium_message_storage* sbd_message_queue;
//GNSS* gnss;
//Iridium* iridium;
//RF_Switch* rf_switch;
//device_handles_t *device_handles;
//CT* ct;
//CHAR* ct_data;
//ct_samples* samples_buf;
//
//
//UINT App_ThreadX_Init(VOID *memory_ptr)
//{
//  UINT ret = TX_SUCCESS;
//  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
//
//   /* USER CODE BEGIN App_ThreadX_MEM_POOL */
//	(void)byte_pool;
//	CHAR *pointer = TX_NULL;
//
//	//
//	// Allocate stack for the watchdog thread
//	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_SMALL_STACK_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	// Create the startup thread. HIGHEST priority level and no preemption possible
//	ret = tx_thread_create(&watchdog_thread, "watchdog thread", watchdog_thread_entry, 0, pointer,
//			THREAD_SMALL_STACK_SIZE, HIGHEST, HIGHEST, TX_NO_TIME_SLICE, TX_AUTO_START);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// Allocate stack for the startup thread
//	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_EXTRA_LARGE_STACK_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	// Create the startup thread. HIGHEST priority level and no preemption possible
//	ret = tx_thread_create(&test_thread, "test thread", test_thread_entry, 0, pointer,
//			THREAD_EXTRA_LARGE_STACK_SIZE, VERY_HIGH, VERY_HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// Create the watchdog window started semaphone
//	ret = tx_semaphore_create(&window_started_semaphone, "window started semaphore", 0);
//	if (ret != TX_SUCCESS) {
//	  return ret;
//	}
//	//
//	// Create the event flags we'll use for triggering threads
//	ret = tx_event_flags_create(&thread_control_flags, "thread flags");
//	if (ret != TX_SUCCESS) {
//	  return ret;
//	}
//	//
//	// Create the error flags we'll use for tracking errors
//	ret = tx_event_flags_create(&error_flags, "error flags");
//	if (ret != TX_SUCCESS) {
//	  return ret;
//	}
//	//
//	// The rf switch struct
//	ret = tx_byte_allocate(byte_pool, (VOID**) &rf_switch, sizeof(RF_Switch), TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//		return ret;
//	}
//	//
//	// The UBX message array
//	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_DMA_message_buf, UBX_MESSAGE_SIZE * 2, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// The UBX process buffer
//	ret = tx_byte_allocate(byte_pool, (VOID**) &ubx_message_process_buf, UBX_MESSAGE_SIZE * 2, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// The gnss struct
//	ret = tx_byte_allocate(byte_pool, (VOID**) &gnss, sizeof(GNSS), TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//		return ret;
//	}
//	//
//	// The Iridium message array -- add 2 to the size for the checksum
//	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_message, IRIDIUM_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH,
//			TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// The Iridium error message payload array
//	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_error_message, IRIDIUM_ERROR_MESSAGE_PAYLOAD_SIZE + IRIDIUM_CHECKSUM_LENGTH,
//				TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// The Iridium response message array
//	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium_response_message, IRIDIUM_MAX_RESPONSE_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// The Iridium message storage
//	ret = tx_byte_allocate(byte_pool, (VOID**) &sbd_message_queue, sizeof(Iridium_message_storage), TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	//
//	// The iridium struct
//	ret = tx_byte_allocate(byte_pool, (VOID**) &iridium, sizeof(Iridium), TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//		return ret;
//	}
//// Only if the IMU will be utilized
//#if IMU_ENABLED
//	//
//	// Allocate stack for the imu thread
//	ret = tx_byte_allocate(byte_pool, (VOID**) &pointer, THREAD_LARGE_STACK_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	// Create the imu thread. VERY_HIGH priority, no preemption-threshold
//	ret = tx_thread_create(&imu_thread, "imu thread", imu_thread_entry, 0, pointer,
//		  THREAD_LARGE_STACK_SIZE, HIGH, HIGH, TX_NO_TIME_SLICE, TX_AUTO_START);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//
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
//
//#endif
//// Only is there is a CT sensor present
//#if CT_ENABLED
//	//
//	// The ct struct
//	ret = tx_byte_allocate(byte_pool, (VOID**) &ct, sizeof(CT), TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//		return ret;
//	}
//	// The CT input data buffer array
//	ret = tx_byte_allocate(byte_pool, (VOID**) &ct_data, CT_DATA_ARRAY_SIZE, TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	// The CT samples array
//	ret = tx_byte_allocate(byte_pool, (VOID**) &samples_buf, TOTAL_CT_SAMPLES * sizeof(ct_samples),
//			TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//	// The CT samples array
//	ret = tx_byte_allocate(byte_pool, (VOID**) &sbd_message, sizeof(sbd_message_type_52),
//			TX_NO_WAIT);
//	if (ret != TX_SUCCESS){
//	  return ret;
//	}
//
//	// Zero out the sbd message struct
//	memset(&sbd_message, 0, sizeof(sbd_message));
//#endif
//
//  return ret;
//}
//
//
//void MX_ThreadX_Init(device_handles_t *handles)
//{
//  device_handles = handles;
//  configuration.duty_cycle = 60;
//  configuration.samples_per_window = 8192;
//  configuration.iridium_max_transmit_time = 99;
//  configuration.gnss_max_acquisition_wait_time = 10;
//  configuration.gnss_sampling_rate = 5;
//  configuration.total_ct_samples = 10;
//
//  tx_kernel_enter();
//}
//
//void watchdog_thread_entry(ULONG thread_input)
//{
//	UINT tx_return;
//	uint32_t window_start_time = 0;
//	uint32_t elapsed_time = 0;
//	uint32_t max_sample_window_duration = MAX_ALLOWABLE_WINDOW_TIME_IN_MINUTES * SECONDS_IN_MIN
//			* MILLISECONDS_PER_MINUTE;
//	uint32_t max_initial_wait_time = 1 * SECONDS_IN_MIN * MILLISECONDS_PER_MINUTE;
//
//	while(elapsed_time < max_sample_window_duration) {
//
//		tx_return = tx_semaphore_get(&window_started_semaphone, TX_NO_WAIT);
//
//		if (tx_return == TX_SUCCESS) {
//			window_start_time = HAL_GetTick();
//		}
//
//		if (window_start_time == 0) {
//			elapsed_time = HAL_GetTick() + max_initial_wait_time;
//		} else {
//			elapsed_time = HAL_GetTick() - window_start_time;
//		}
//
//		HAL_IWDG_Refresh(device_handles->watchdog_handle);
//		tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
//	}
//	tx_thread_terminate(&watchdog_thread);
//
//}
//
//void test_thread_entry(ULONG thread_input){
//	int fail_counter;
//
//	tx_semaphore_put(&window_started_semaphone);
//
//	north = argInit_1xUnbounded_real32_T(&configuration);
//	east  = argInit_1xUnbounded_real32_T(&configuration);
//	down  = argInit_1xUnbounded_real32_T(&configuration);
//
//	rf_switch_init(rf_switch);
//	gnss_init(gnss, &configuration, device_handles->GNSS_uart, device_handles->GNSS_dma_handle,
//			&thread_control_flags, &error_flags, &(ubx_message_process_buf[0]), device_handles->hrtc,
//			north->data, east->data, down->data);
//	iridium_init(iridium, &configuration, device_handles->Iridium_uart,
//					device_handles->Iridium_rx_dma_handle, device_handles->iridium_timer,
//					device_handles->Iridium_tx_dma_handle, &thread_control_flags, &error_flags,
//					device_handles->hrtc, &sbd_message, iridium_error_message,
//					iridium_response_message, sbd_message_queue);
//#if CT_ENABLED
//	ct_init(ct, &configuration, device_handles->CT_uart, device_handles->CT_dma_handle,
//					&thread_control_flags, &error_flags, ct_data, samples_buf);
//#endif
//
//	turn_it_all_on();
//	HAL_Delay(1000);
//	shut_it_all_down();
//
//	///////////////////////////////////////////////////////////////////////////////////////////////
//	/////////////////////////// GNSS STARTUP SEQUENCE /////////////////////////////////////////////
//	rf_switch->set_gnss_port(rf_switch);
//	gnss->on_off(gnss, GPIO_PIN_SET);
//	fail_counter = 0;
//	while (fail_counter < MAX_SELF_TEST_RETRIES) {
//		if (gnss->config(gnss) != GNSS_SUCCESS) {
//			gnss->cycle_power(gnss);
//			fail_counter++;
//		} else {
//			break;
//		}
//	}
//	if (fail_counter == MAX_SELF_TEST_RETRIES) {
//		HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//		while(1);
//	}
//	if (gnss->sync_and_start_reception(gnss, start_GNSS_UART_DMA, ubx_DMA_message_buf, UBX_MESSAGE_SIZE)
//				!= GNSS_SUCCESS)
//	{
//		HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//		while(1);
//	}
//	gnss->on_off(gnss, GPIO_PIN_RESET);
//
//	///////////////////////////////////////////////////////////////////////////////////////////////
//	///////////////////////IRIDIUM STARTUP SEQUENCE ///////////////////////////////////////////////
//	rf_switch->set_iridium_port(rf_switch);
//	iridium->queue_flush(iridium);
//	if (iridium->self_test(iridium) != IRIDIUM_SUCCESS)
//	{
//		HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//		while(1);
//	}
//	if (iridium->config(iridium) != IRIDIUM_SUCCESS)
//	{
//		HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//		while(1);
//	}
//	iridium->transmit_error_message(iridium, "This is a test message.");
//	iridium->sleep(iridium, GPIO_PIN_RESET);
//	iridium->on_off(iridium, GPIO_PIN_RESET);
//	///////////////////////////////////////////////////////////////////////////////////////////////
//	/////////////////////////// CT STARTUP SEQUENCE ///////////////////////////////////////////////
//	if (ct->self_test(ct, false) != CT_SUCCESS) {
//		HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//		while(1);
//	}
//	ct->on_off(ct, GPIO_PIN_RESET);
//
//	led_sequence(INITIAL_LED_SEQUENCE);
//	led_sequence(TEST_PASSED_LED_SEQUENCE);
//	led_sequence(TEST_NON_CRITICAL_FAULT_LED_SEQUENCE);
//	led_sequence(TEST_CRITICAL_FAULT_LED_SEQUENCE);
//
//	tx_thread_terminate(&test_thread);
//}
//
//static void led_sequence(led_sequence_t sequence)
//{
//	switch (sequence) {
//		case INITIAL_LED_SEQUENCE:
//			for (int i = 0; i < 10; i++){
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//				HAL_Delay(250);
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
//				HAL_Delay(250);
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_RESET);
//				HAL_Delay(250);
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
//				HAL_Delay(250);
//			}
//			break;
//
//		case TEST_PASSED_LED_SEQUENCE:
//			for (int i = 0; i < 5; i++){
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
//				HAL_Delay(1000);
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
//				HAL_Delay(1000);
//			}
//			break;
//
//		case TEST_NON_CRITICAL_FAULT_LED_SEQUENCE:
//			for (int i = 0; i < 20; i++){
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_SET);
//				HAL_Delay(250);
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_GREEN_Pin, GPIO_PIN_RESET);
//				HAL_Delay(250);
//			}
//			break;
//
//		case TEST_CRITICAL_FAULT_LED_SEQUENCE:
//			for (int i = 0; i < 10; i++){
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_RESET);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(GPIOF, EXT_LED_RED_Pin, GPIO_PIN_SET);
//				HAL_Delay(500);
//			}
//			break;
//
//		default:
//			break;
//	}
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	// Context is saved/restored in upstream Interrupt call chain
//	if (huart->Instance == gnss->gnss_uart_handle->Instance) {
//		if (!gnss->is_configured) {
//
//			tx_event_flags_set(&thread_control_flags, GNSS_CONFIG_RECVD, TX_NO_WAIT);
//
//		} else {
//			memcpy(&(gnss->ubx_process_buf[0]), &(ubx_DMA_message_buf[0]), UBX_MESSAGE_SIZE);
//			gnss->process_message(gnss);
//		}
//	}
//
//	// CT sensor
//	else if (huart->Instance == ct->ct_uart_handle->Instance) {
//		tx_event_flags_set(&thread_control_flags, CT_MSG_RECVD, TX_OR);
//	}
//
//	// Iridium modem
//	else if (huart->Instance == iridium->iridium_uart_handle->Instance) {
//		tx_event_flags_set(&thread_control_flags, IRIDIUM_MSG_RECVD, TX_OR);
//	}
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	// High frequency, low overhead ISR, no need to save/restore context
//	if (htim->Instance == TIM16) {
//		HAL_IncTick();
//	}
//	else if (htim->Instance == TIM17) {
//		iridium->timer_timeout = true;
//	}
//}
//
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
//
//gnss_error_code_t start_GNSS_UART_DMA(GNSS* gnss_struct_ptr, uint8_t* buffer, size_t msg_size)
//{
//	gnss_error_code_t return_code = GNSS_SUCCESS;
//	HAL_StatusTypeDef hal_return_code;
//
//	gnss->reset_uart(gnss, GNSS_DEFAULT_BAUD_RATE);
//
//	memset(&(buffer[0]), 0, UBX_MESSAGE_SIZE * 2);
//
//	HAL_UART_DMAStop(gnss_struct_ptr->gnss_uart_handle);
//
//	hal_return_code = MX_GNSS_LL_Queue_Config();
//
//	if (hal_return_code != HAL_OK) {
//		return_code = GNSS_UART_ERROR;
//	}
//
//	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.Priority = DMA_LOW_PRIORITY_HIGH_WEIGHT;
//	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
//	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
//	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
//	gnss_struct_ptr->gnss_dma_handle->InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
//
//	if (HAL_DMAEx_List_Init(gnss_struct_ptr->gnss_dma_handle) != HAL_OK)
//	{
//		return_code = GNSS_UART_ERROR;
//	}
//
//	__HAL_LINKDMA(gnss_struct_ptr->gnss_uart_handle, hdmarx, *gnss_struct_ptr->gnss_dma_handle);
//
//	hal_return_code = HAL_DMAEx_List_LinkQ(gnss_struct_ptr->gnss_dma_handle, &GNSS_LL_Queue);
//	if (hal_return_code != HAL_OK) {
//		return_code = GNSS_UART_ERROR;
//	}
//
//	hal_return_code = HAL_UART_Receive_DMA(gnss_struct_ptr->gnss_uart_handle,
//			(uint8_t*)&(buffer[0]), msg_size);
//		//  No need for the half-transfer complete interrupt, so disable it
//	__HAL_DMA_DISABLE_IT(gnss->gnss_dma_handle, DMA_IT_HT);
//
//	if (hal_return_code != HAL_OK) {
//		return_code = GNSS_UART_ERROR;
//	}
//
//	return return_code;
//}
//
//void shut_it_all_down(void)
//{
//	HAL_GPIO_WritePin(GPIOD, IRIDIUM_OnOff_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, GPIO_PIN_RESET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOG, GNSS_FET_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOG, CT_FET_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOG, RF_SWITCH_VCTL_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOG, RF_SWITCH_EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(10);
//}
//
//void turn_it_all_on(void)
//{
//	HAL_GPIO_WritePin(GPIOF, BUS_5V_FET_Pin, GPIO_PIN_SET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(GPIOD, IRIDIUM_FET_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOD, IRIDIUM_OnOff_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOG, GNSS_FET_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOG, CT_FET_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOG, RF_SWITCH_VCTL_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOG, RF_SWITCH_EN_Pin, GPIO_PIN_SET);
//	HAL_Delay(10);
//}
//
///* USER CODE END 1 */
