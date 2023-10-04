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





// Helper functions
static uint32_t get_flash_page(uint32_t addr);
static void flash_prologue(void);
static void flash_epilogue(void);
static flash_storage_error_code_t test_bookkeeping_page(void);
static flash_storage_error_code_t write_array(float* input_array, uint32_t array_size);


void flash_storage_init(Flash_storage* flash_storage_struct_ptr,
		microSWIFT_configuration* global_config)
{
	Flash_storage_bookkeeping bookkeeping_temp;

	self = flash_storage_struct_ptr;
	self->global_config = global_config;
	self->write_sample_window = flash_storage_write_sample_window;


	/*	Must check if the flash page allocated for bookkeeping has
		any data written to it. If not, initialize everything. Otherwise
		leave it alone. */
	if (test_bookkeeping_page() == FLASH_BOOKKEEPING_EMPTY) {

		bookkeeping_temp.flash_page_addr = ADDR_FLASH_PAGE_127;
		bookkeeping_temp.cycle_count = 0;
		bookkeeping_temp.num_pages_written = 0;
		bookkeeping_temp.first_empty_page = ADDR_FLASH_PAGE_128;

		erase_init_struct.TypeErase   = FLASH_TYPEERASE_PAGES;
		erase_init_struct.Banks       = FLASH_BANK_1;
		erase_init_struct.Page        = get_flash_page(bookkeeping_temp.flash_page_addr);
		erase_init_struct.NbPages     = 1;

		flash_prologue();

		if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK) {
			self->flash_error_occured = true;
			flash_epilogue();
			return;
		}

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, bookkeeping_temp.flash_page_addr, ((uint32_t)&bookkeeping_temp)) != HAL_OK) {
			self->flash_error_occured = true;
			flash_epilogue();
			return;
		}

		self->flash_error_occured = false;

		flash_epilogue();
	}

}

static flash_storage_error_code_t flash_storage_write_sample_window(float* north_array,
		float* east_array, float* down_array)
{
	flash_storage_error_code_t return_code = FLASH_STORAGE_FULL;

	// TODO: do a check to see if there is space remaining for the storage window.
	// If not, exit early.
	// Simple solution
	if (self->bookkeeping.num_pages_written == 128) {
		return_code = FLASH_STORAGE_FULL;
		return return_code;
	}

	// Write the North array
	return_code = write_array(north_array, self->global_config->samples_per_window);
	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

	// Write the East Array
	return_code = write_array(east_array, self->global_config->samples_per_window);
	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

	// Write the Down array
	return_code = write_array(down_array, self->global_config->samples_per_window);
	if (return_code != FLASH_SUCCESS) {
		self->flash_error_occured = true;
		return return_code;
	}

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
	HAL_ICACHE_Disable();
	HAL_FLASH_Unlock();
}

static void flash_epilogue(void)
{
	HAL_FLASH_Unlock();
	HAL_ICACHE_Enable();
}

static flash_storage_error_code_t test_bookkeeping_page(void)
{
	flash_storage_error_code_t return_code = FLASH_BOOKKEEPING_NOT_EMPTY;
	Flash_storage_bookkeeping* bookkeeping_ptr = (Flash_storage_bookkeeping*)ADDR_FLASH_PAGE_127;

	if (bookkeeping_ptr->flash_page_addr != ADDR_FLASH_PAGE_127) {
		return_code = FLASH_BOOKKEEPING_EMPTY;
	}

	return return_code;
}

static flash_storage_error_code_t write_array(float* input_array, uint32_t array_size)
{
	flash_storage_error_code_t return_code = FLASH_SUCCESS;
	uint32_t number_of_pages_per_array =
			(self->global_config->samples_per_window * sizeof(float)) / FLASH_PAGE_SIZE;
//	uint32_t start_address = self->bookkeeping->first_empty_page;
//	uint32_t space_per_array = number_of_pages_per_array * FLASH_PAGE_SIZE;
	uint32_t current_address = self->bookkeeping.first_empty_page;
	uint32_t end_address = current_address + (number_of_pages_per_array * FLASH_PAGE_SIZE);
	float quad_word[4] = {0};

	// TODO: test current_address to ensure there is space

	flash_prologue();

	while (current_address < end_address) {
		quad_word[0] = *input_array;
		input_array++;
		quad_word[1] = *input_array;
		input_array++;
		quad_word[2] = *input_array;
		input_array++;
		quad_word[3] = *input_array;
		input_array++;

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, current_address,
				((uint32_t)quad_word)) != HAL_OK)
		{
			return_code = FLASH_PROGRAM_ERROR;
			break;
		}

		current_address += 4 * sizeof(uint32_t);
	}

	flash_epilogue();

	if (return_code == FLASH_SUCCESS) {
		self->bookkeeping.num_pages_written += number_of_pages_per_array;
		self->bookkeeping.first_empty_page = end_address;
	}

	return return_code;
}

