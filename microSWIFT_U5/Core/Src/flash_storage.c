/*
 * flash_storage.c
 *
 *  Created on: Oct 3, 2023
 *      Author: Phil
 */

#include "flash_storage.h"
// Static variables
static Flash_storage* self; // Private object pointer
static FLASH_EraseInitTypeDef erase_init_struct;
static uint32_t page_error;

// Static object functions
static flash_storage_error_code_t flash_storage_write_sample_window(float* north_array,
		float* east_array, float* down_array);
static flash_storage_error_code_t flash_storage_write_bookkeeping(
		Flash_storage_bookkeeping* bookkeeping_to_write);

// Helper functions
static uint32_t get_flash_page(uint32_t addr);
static void flash_prologue(void);
static void flash_epilogue(void);
static flash_storage_error_code_t test_bookkeeping_page(void);
static flash_storage_error_code_t write_array(float* input_array, uint32_t array_size);
static flash_storage_error_code_t check_array(float* input_array, float* flash_array,
		uint32_t array_size);



void flash_storage_init(Flash_storage* flash_storage_struct_ptr,
		microSWIFT_configuration* global_config)
{
	Flash_storage_bookkeeping bookkeeping_temp;

	self = flash_storage_struct_ptr;
	self->global_config = global_config;
	self->write_sample_window = flash_storage_write_sample_window;
	self->write_bookkeeping = flash_storage_write_bookkeeping;
	self->bookkeeping = (Flash_storage_bookkeeping*)ADDR_FLASH_PAGE_127;
	self->flash_error_occured = false;


	/*	Must check if the flash page allocated for bookkeeping has
		any data written to it. If not, initialize everything. Otherwise
		leave it alone. */
	if ((test_bookkeeping_page() == FLASH_BOOKKEEPING_EMPTY) || CLEAR_USER_FLASH) {

		bookkeeping_temp.flash_page_addr = ADDR_FLASH_PAGE_127;
		bookkeeping_temp.cycle_count = 0;
		bookkeeping_temp.num_pages_written = 0;
		bookkeeping_temp.first_empty_page = ADDR_FLASH_PAGE_128;

		erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
		erase_init_struct.Banks       = FLASH_BANK_2;
		erase_init_struct.Page        = get_flash_page(bookkeeping_temp.first_empty_page);
		erase_init_struct.NbPages     = FLASH_PAGE_NB;

		flash_prologue();

		// Erase the pages
		if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK) {
			self->flash_error_occured = true;
			flash_epilogue();
			return;
		}

		if (self->write_bookkeeping(&bookkeeping_temp) != FLASH_SUCCESS) {
			self->flash_error_occured = true;
			return;
		}
	}
}

static flash_storage_error_code_t flash_storage_write_sample_window(float* north_array,
		float* east_array, float* down_array)
{
	flash_storage_error_code_t return_code = FLASH_STORAGE_FULL;

	if (self->flash_error_occured) {
		return_code = FLASH_UNKNOWN_ERROR;
		return return_code;
	}

	// The write_array function will ensure there is enough space prior to writing

	register_watchdog_refresh();

	// Write the North array
	return_code = write_array(north_array, self->global_config->samples_per_window);
	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

	register_watchdog_refresh();

	// Write the East Array
	return_code = write_array(east_array, self->global_config->samples_per_window);
	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

	register_watchdog_refresh();

	// Write the Down array
	return_code = write_array(down_array, self->global_config->samples_per_window);
	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

	register_watchdog_refresh();

	return return_code;
}





/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t get_flash_page(uint32_t addr)
{
  uint32_t page = 0;

  if (addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

static void flash_prologue(void)
{
	HAL_ICACHE_Invalidate();
	HAL_ICACHE_Disable();
	HAL_FLASH_Unlock();
}

static void flash_epilogue(void)
{
	HAL_FLASH_Unlock();
	HAL_ICACHE_Enable();
	HAL_ICACHE_Invalidate();
}

static flash_storage_error_code_t test_bookkeeping_page(void)
{
	flash_storage_error_code_t return_code = FLASH_BOOKKEEPING_NOT_EMPTY;
	Flash_storage_bookkeeping* bookkeeping_ptr =
			(Flash_storage_bookkeeping*)ADDR_FLASH_PAGE_127;

	if (bookkeeping_ptr->flash_page_addr != ADDR_FLASH_PAGE_127) {
		return_code = FLASH_BOOKKEEPING_EMPTY;
	}

	return return_code;
}

static flash_storage_error_code_t write_array(float* input_array, uint32_t array_size)
{
	flash_storage_error_code_t return_code = FLASH_SUCCESS;
	Flash_storage_bookkeeping bookkeeping_temp;
	float* input_array_ptr = input_array;
	uint32_t number_of_pages_per_array =
			(self->global_config->samples_per_window * sizeof(float)) / FLASH_PAGE_SIZE;
	uint32_t start_address = self->bookkeeping->first_empty_page;
	uint32_t current_address = start_address;
	uint32_t end_address = current_address + (number_of_pages_per_array * FLASH_PAGE_SIZE);
	uint32_t size_of_burst = 8 * (4 * sizeof(uint32_t)); // 8 quadwords

	if ((FLASH_PAGE_NB - self->bookkeeping->num_pages_written) < number_of_pages_per_array) {
		return_code = FLASH_STORAGE_FULL;
		return return_code;
	}

	erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init_struct.Banks       = FLASH_BANK_2;
	erase_init_struct.Page        = get_flash_page(current_address);
	erase_init_struct.NbPages     = number_of_pages_per_array;

	flash_prologue();

	// Erase the pages
	if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK) {
		self->flash_error_occured = true;
		flash_epilogue();
		return_code = FLASH_ERASE_ERROR;
		return return_code;
	}

	// Write the data 1 burst (8 quadwords) at a time
	while (current_address < end_address) {

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BURST, current_address,
				((uint32_t)input_array_ptr)) != HAL_OK)
		{
			return_code = FLASH_PROGRAM_ERROR;
			break;
		}
		// current_address is raw uint32_t as an address
		current_address += size_of_burst;
		// input_array is a float pointer
		input_array_ptr += size_of_burst / sizeof(float);
	}

	flash_epilogue();

	return_code = check_array(input_array, (float*)start_address,
			self->global_config->samples_per_window);

	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

	if (return_code == FLASH_SUCCESS) {

		bookkeeping_temp.flash_page_addr = ADDR_FLASH_PAGE_127;
		bookkeeping_temp.cycle_count = self->bookkeeping->cycle_count + 1;
		bookkeeping_temp.num_pages_written = self->bookkeeping->num_pages_written
				+ number_of_pages_per_array;
		bookkeeping_temp.first_empty_page = end_address;

		return_code = self->write_bookkeeping(&bookkeeping_temp);
	}

//	return_code = check_array(input_array, (float*)start_address,
//			self->global_config->samples_per_window);

	return return_code;
}

static flash_storage_error_code_t check_array(float* input_array, float* flash_array,
		uint32_t array_size)
{
	flash_storage_error_code_t return_code = FLASH_SUCCESS;

	for (int i = 0; i < array_size; i++) {
		if (flash_array[i] != input_array[i]) {
			return_code = FLASH_CORRUPTION_ERROR;
			return return_code;
		}
	}

	return return_code;
}

static flash_storage_error_code_t flash_storage_write_bookkeeping(
		Flash_storage_bookkeeping* bookkeeping_to_write)
{
	flash_storage_error_code_t return_code = FLASH_SUCCESS;

	erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init_struct.Banks       = FLASH_BANK_1;
	erase_init_struct.Page        = get_flash_page(bookkeeping_to_write->flash_page_addr);
	erase_init_struct.NbPages     = 1;

	flash_prologue();

	// Erase the bookkeeping page
	if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK) {
		flash_epilogue();
		return_code = FLASH_ERASE_ERROR;
		return return_code;
	}
	// Write the new bookkeeping data
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, bookkeeping_to_write->flash_page_addr, ((uint32_t)bookkeeping_to_write)) != HAL_OK) {
		flash_epilogue();
		return_code = FLASH_PROGRAM_ERROR;
		return return_code;
	}

	flash_epilogue();
	return return_code;
}

