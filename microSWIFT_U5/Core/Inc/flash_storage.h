/*
 * flash_storage.h
 *
 *  Created on: Oct 3, 2023
 *      Author: Phil
 */

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#include "stm32u5xx_hal.h"

typedef struct Flash_storage {

}Flash_storage;

typedef struct Flash_storage_bookkeeping {
	uint32_t flash_page_addr = ADDR_FLASH_PAGE_127;
	uint32_t cycle_count = 0;

}Flash_storage_bookkeeping_t;

#define ADDR_FLASH_PAGE_127   ((uint32_t)0x080FE000) /* Base @ of Page 127, 8 Kbytes */

#endif /* INC_FLASH_STORAGE_H_ */
