/*
 * flash_storage.h
 *
 *  Created on: Oct 3, 2023
 *      Author: Phil
 */

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#include "stm32u5xx_hal.h"
#include "configuration.h"

#define ADDR_FLASH_PAGE_127   0x080FE000 /* Base @ of Page 127, 8 Kbytes */
#define ADDR_FLASH_PAGE_128	  0x08100000 /* First page of bank 2 */
#define FLASH_NON_INIT_VALUE  0xFFFFFFFF

typedef enum flash_storage_error_code {
	FLASH_SUCCESS = 0,
	FLASH_ERASE_ERROR = -1,
	FLASH_PROGRAM_ERROR = -2,
	FLASH_BOOKKEEPING_EMPTY = -3,
	FLASH_BOOKKEEPING_NOT_EMPTY = -4,
	FLASH_STORAGE_FULL = -5,
	FLASH_UNKNOWN_ERROR = -6
}flash_storage_error_code_t;

typedef struct Flash_storage_bookkeeping {
	uint32_t flash_page_addr;
	uint32_t cycle_count;
	uint32_t num_pages_written;
	uint32_t first_empty_page;
}Flash_storage_bookkeeping;

typedef struct Flash_storage {
	Flash_storage_bookkeeping 	bookkeeping;
	bool 						flash_error_occured;
	microSWIFT_configuration* 	global_config;

	flash_storage_error_code_t 	(*write_sample_window)(float* north_array,
			float* east_array, float* down_array);
}Flash_storage;

void flash_storage_init(Flash_storage* flash_storage_struct_ptr,
		microSWIFT_configuration* global_config);

// watchdog refresh function
extern void 		 register_watchdog_refresh();

#endif /* INC_FLASH_STORAGE_H_ */
